import math
import struct
import time

import board
import busio
import adafruit_gps
import adafruit_ltr390
import adafruit_sht31d
import adafruit_ssd1306
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32

# ---------------------------------------------------------------- settings
LPS_ADDR = 0x5C
INA_ADDR = 0x40
SCREEN_W = 128
SCREEN_H = 64
GRAVITY = 9.80665
LAUNCH_G = 3.0
FREEFALL_G = 0.3
SHUNT_OHMS = 0.015  # CHECK your board
I2C_HZ = 100000
FRAME_MS = 200      # one CSV line + display refresh every 200 ms
PAGE_MS = 3000      # display page changes every 3 s
NUM_PAGES = 5
USE_MAG = False     # True to also read the BNO085 magnetometer

CSV_HEADER = (
    "ms,ax,ay,az,gx,gy,gz,qi,qj,qk,qr,hPa,tempC,rh,uvs,vbus,amps,lat,lon,altM,"
    "sats,yaw,pitch,roll,gTotal,relAltM,linX,linY,linZ,magX,magY,magZ,"
    "headingDeg,als,dewC,watts,energyWh,maxG,maxGyroDps,flightS,freeFall"
)

i2c = busio.I2C(board.SCL, board.SDA, frequency=I2C_HZ)


# ---------------------------------------------------------------- helpers
def millis():
    return time.monotonic_ns() // 1000000


def safe(read, default):
    """Run read(); return default if the sensor is missing or the bus glitches."""
    try:
        return read()
    except Exception:
        return default


def read_regs(addr, reg, count):
    buf = bytearray(count)
    while not i2c.try_lock():
        pass
    try:
        i2c.writeto_then_readfrom(addr, bytes([reg]), buf)
    finally:
        i2c.unlock()
    return buf


def write_reg(addr, reg, value):
    while not i2c.try_lock():
        pass
    try:
        i2c.writeto(addr, bytes([reg, value]))
    finally:
        i2c.unlock()


def read_int16(addr, reg):
    return struct.unpack(">h", read_regs(addr, reg, 2))[0]


def read_pressure_hpa():
    b = read_regs(LPS_ADDR, 0x28, 3)
    raw = b[0] | (b[1] << 8) | (b[2] << 16)
    if raw & 0x800000:
        raw -= 1 << 24
    return raw / 4096.0


# ---------------------------------------------------------------- display
oled = None
for _addr in (0x3C, 0x3D):
    try:
        oled = adafruit_ssd1306.SSD1306_I2C(SCREEN_W, SCREEN_H, i2c, addr=_addr)
        break
    except (ValueError, OSError):
        pass
print("OLED ok" if oled else "OLED FAIL")


def show(lines):
    """Draw up to 8 lines of 16 characters."""
    if oled is None:
        return
    try:
        oled.poweron()
        oled.fill(0)
        for row, text in enumerate(lines):
            oled.text(text, 0, row * 8, 1)
        oled.show()
    except OSError:
        pass


checklist = []


def report(name, ok):
    text = name + (" ok" if ok else " FAIL")
    print(text)
    checklist.append(text)
    show(checklist[-8:])


def start(name, make):
    try:
        device = make()
        report(name, True)
        return device
    except Exception as err:
        print(name, "error:", err)
        report(name, False)
        return None


# ---------------------------------------------------------------- startup
while not i2c.try_lock():
    pass
found = i2c.scan()
i2c.unlock()
print("I2C devices:", " ".join(hex(a) for a in found))


def make_bno():
    sensor = BNO08X_I2C(i2c, address=0x4A)
    sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    sensor.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
    if USE_MAG:
        sensor.enable_feature(BNO_REPORT_MAGNETOMETER)
    return sensor


def make_lps():
    if read_regs(LPS_ADDR, 0x0F, 1)[0] != 0xB4:
        raise RuntimeError("wrong chip id")
    write_reg(LPS_ADDR, 0x11, 0x08)
    write_reg(LPS_ADDR, 0x10, 0x1C)
    return True


lsm = start("LSM6DSO32", lambda: LSM6DSO32(i2c, address=0x6A))
bno = start("BNO085", make_bno)
sht = start("SHT31", lambda: adafruit_sht31d.SHT31D(i2c, address=0x44))
ltr = start("LTR390", lambda: adafruit_ltr390.LTR390(i2c))
gps = start("GNSS", lambda: adafruit_gps.GPS_GtopI2C(i2c, address=0x42))
start("LPS28DFW", make_lps)

time.sleep(1.5)
print(CSV_HEADER)

# ---------------------------------------------------------------- state
bno_vals = {
    "quat": (0.0, 0.0, 0.0, 0.0),
    "lin": (0.0, 0.0, 0.0),
    "mag": (0.0, 0.0, 0.0),
}
uvs = 0.0
als = 0.0
uv_mode = True
ground_hpa = 0.0
max_alt = 0.0
max_g = 0.0
max_gyro = 0.0
energy_wh = 0.0
launch_ms = None
last_frame = None


def poll_sensors():
    """Called as often as possible so the BNO085 and GPS buffers never overflow."""
    if bno is not None:
        try:
            bno_vals["quat"] = bno.quaternion
            bno_vals["lin"] = bno.linear_acceleration
            if USE_MAG:
                bno_vals["mag"] = bno.magnetic
        except (RuntimeError, KeyError):
            pass  # that report has not arrived yet
    if gps is not None:
        gps.update()


# ---------------------------------------------------------------- main loop
while True:
    try:
        poll_sensors()

        now = millis()
        if last_frame is not None and now - last_frame < FRAME_MS:
            continue
        dt = 0.0 if last_frame is None else (now - last_frame) / 1000.0
        last_frame = now

        # read the sensors
        acc_x, acc_y, acc_z = safe(lambda: lsm.acceleration, (0.0, 0.0, 0.0))
        gyro_x, gyro_y, gyro_z = safe(lambda: lsm.gyro, (0.0, 0.0, 0.0))
        hpa = safe(read_pressure_hpa, 0.0)
        temp_c = safe(lambda: sht.temperature, 0.0)
        rh = safe(lambda: sht.relative_humidity, 0.0)
        vbus = safe(lambda: read_int16(INA_ADDR, 0x05), 0) * 0.003125
        amps = safe(lambda: read_int16(INA_ADDR, 0x04), 0) * 0.000005 / SHUNT_OHMS

        # UV and ambient light take turns, one reading per frame
        if ltr is not None:
            if uv_mode:
                uvs = safe(lambda: ltr.uvs, uvs)
            else:
                als = safe(lambda: ltr.light, als)
            uv_mode = not uv_mode

        lat = lon = alt = 0.0
        sats = 0
        if gps is not None and gps.has_fix:
            lat = gps.latitude or 0.0
            lon = gps.longitude or 0.0
            alt = gps.altitude_m or 0.0
            sats = gps.satellites or 0

        qi, qj, qk, qr = bno_vals["quat"]
        lin_x, lin_y, lin_z = bno_vals["lin"]
        mag_x, mag_y, mag_z = bno_vals["mag"]

        # derived values
        roll = math.degrees(math.atan2(2 * (qr * qi + qj * qk), 1 - 2 * (qi * qi + qj * qj)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (qr * qj - qk * qi)))))
        yaw = math.degrees(math.atan2(2 * (qr * qk + qi * qj), 1 - 2 * (qj * qj + qk * qk)))
        g_total = math.sqrt(acc_x ** 2 + acc_y ** 2 + acc_z ** 2) / GRAVITY
        gyro_dps = math.degrees(math.sqrt(gyro_x ** 2 + gyro_y ** 2 + gyro_z ** 2))
        mag_total = math.sqrt(mag_x ** 2 + mag_y ** 2 + mag_z ** 2)
        heading = math.degrees(math.atan2(mag_y, mag_x)) % 360

        if ground_hpa == 0 and hpa > 300:
            ground_hpa = hpa
        rel_alt = 0.0
        if ground_hpa > 0 and hpa > 0:
            rel_alt = 44330.0 * (1.0 - (hpa / ground_hpa) ** 0.1903)
        max_alt = max(max_alt, rel_alt)

        dew_c = 0.0
        if rh > 0:
            gam = math.log(rh / 100.0) + (17.62 * temp_c) / (243.12 + temp_c)
            dew_c = 243.12 * gam / (17.62 - gam)

        watts = vbus * amps
        energy_wh += watts * dt / 3600.0

        max_g = max(max_g, g_total)
        max_gyro = max(max_gyro, gyro_dps)
        if launch_ms is None and g_total > LAUNCH_G:
            launch_ms = now
        flight_s = 0.0 if launch_ms is None else (now - launch_ms) / 1000.0
        free_fall = g_total < FREEFALL_G

        # one CSV line
        row = [
            str(now),
            f"{acc_x:.2f}", f"{acc_y:.2f}", f"{acc_z:.2f}",
            f"{gyro_x:.4f}", f"{gyro_y:.4f}", f"{gyro_z:.4f}",
            f"{qi:.4f}", f"{qj:.4f}", f"{qk:.4f}", f"{qr:.4f}",
            f"{hpa:.2f}", f"{temp_c:.2f}", f"{rh:.2f}",
            f"{uvs:.0f}",
            f"{vbus:.2f}", f"{amps:.2f}",
            f"{lat:.6f}", f"{lon:.6f}", f"{alt:.2f}",
            str(sats),
            f"{yaw:.1f}", f"{pitch:.1f}", f"{roll:.1f}",
            f"{g_total:.2f}", f"{rel_alt:.1f}",
            f"{lin_x:.2f}", f"{lin_y:.2f}", f"{lin_z:.2f}",
            f"{mag_x:.1f}", f"{mag_y:.1f}", f"{mag_z:.1f}",
            f"{heading:.0f}", f"{als:.0f}", f"{dew_c:.1f}",
            f"{watts:.2f}", f"{energy_wh:.3f}",
            f"{max_g:.2f}", f"{max_gyro:.0f}", f"{flight_s:.1f}",
            "1" if free_fall else "0",
        ]
        print(",".join(row))

        # display page (changes every 3 seconds)
        page = (now // PAGE_MS) % NUM_PAGES
        if page == 0:
            lines = [
                f"Ax {acc_x:.1f}",
                f"Ay {acc_y:.1f}",
                f"Az {acc_z:.1f}",
                f"Total {g_total:.2f} g",
                f"P  {hpa:.1f} hPa",
                f"T  {temp_c:.1f}C H {rh:.0f}%",
                f"Dew {dew_c:.1f} C",
                f"GPS {sats} sats",
            ]
        elif page == 1:
            lines = [
                "ORIENTATION",
                f"Yaw   {yaw:.1f} deg",
                f"Pitch {pitch:.1f} deg",
                f"Roll  {roll:.1f} deg",
                f"Gx {gyro_x:.2f} r/s",
                f"Gy {gyro_y:.2f} r/s",
                f"Gz {gyro_z:.2f} r/s",
                f"Rate {gyro_dps:.0f} dps",
            ]
        elif page == 2:
            lines = [
                "MOTION / COMPASS",
                f"LinX {lin_x:.2f} m/s2",
                f"LinY {lin_y:.2f} m/s2",
                f"LinZ {lin_z:.2f} m/s2",
                f"Mag {mag_total:.1f} uT",
                f"Head {heading:.0f} deg",
                "FreeFall " + ("YES" if free_fall else "no"),
                "Launch " + ("YES" if launch_ms is not None else "no"),
            ]
        elif page == 3:
            lines = [
                "ALTITUDE / LIGHT",
                f"Alt {rel_alt:.1f} m",
                f"Max {max_alt:.1f} m",
                f"P   {hpa:.2f} hPa",
                f"UV  {uvs:.0f}",
                f"Lt  {als:.0f}",
                f"GPSalt {alt:.0f} m",
                f"Lat {lat:.4f}",
            ]
        else:
            lines = [
                "POWER / FLIGHT",
                f"V {vbus:.2f} V",
                f"I {amps:.2f} A",
                f"P {watts:.2f} W",
                f"E {energy_wh:.3f} Wh",
                f"MaxG {max_g:.2f}",
                f"MaxRate {max_gyro:.0f} dps",
                f"Flight {flight_s:.1f} s",
            ]
        show(lines)

    except Exception as err:
        print("ERROR:", err)