
#include <Wire.h>                    // Arduino's I2C library: lets us talk to chips over the SDA/SCL wires.
#include <Adafruit_GFX.h>            // Adafruit's drawing library (text, lines); the display library needs it.
#include <Adafruit_SSD1306.h>        // Driver for the SSD1306 chip that is on most 0.96" I2C displays.
#include <Adafruit_LSM6DSO32.h>      // Driver for the LSM6DSO32 accelerometer + gyroscope.
#include <Adafruit_BNO08x.h>         // Driver for the BNO085 orientation sensor.
#include <Adafruit_SHT31.h>          // Driver for the SHT31 temperature/humidity sensor.
#include <Adafruit_LTR390.h>         // Driver for the LTR390 UV/light sensor.
#include <SparkFun_u-blox_GNSS_v3.h> // Driver for the u-blox NEO-F10N GPS module.

#define LPS_ADDR 0x5C                // I2C address of the LPS28DFW pressure sensor (0x prefix = hexadecimal number).
#define INA_ADDR 0x40                // I2C address of the INA238 power monitor.
#define SCREEN_W 128                 // Display width in pixels.
#define SCREEN_H 64                  // Display height in pixels.
#define RAD2DEG 57.29578             // Multiply radians by this to get degrees (180 / pi).
#define GRAVITY 9.80665              // Standard gravity in m/s^2; dividing an acceleration by this gives g.
#define LAUNCH_G 3.0                 // Total acceleration (in g) above which we say "launch detected".
#define FREEFALL_G 0.3               // Total acceleration (in g) below which we say "free fall".

const float SHUNT_OHMS = 0.015;      // Resistance of the INA238's shunt resistor; used to turn voltage into current. CHECK your board.

Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1); // Make a display object: size, which I2C bus (Wire), reset pin -1 = none.
Adafruit_LSM6DSO32 lsm;              // Make an (empty) accelerometer/gyro object named lsm.
Adafruit_BNO08x bno(-1);             // Make a BNO085 object; -1 means "no reset pin wired".
Adafruit_SHT31 sht(&Wire);           // Make an SHT31 object and tell it to use the Wire bus.
Adafruit_LTR390 ltr;                 // Make a UV/light sensor object.
SFE_UBLOX_GNSS gnss;                 // Make a GPS object.
sh2_SensorValue_t bnoVal;            // A struct (a bundle of variables) that the BNO085 fills with its latest report.

bool gnssOk = false;                 // True only if the GPS module answered at startup; lets us skip it when it is not connected.
bool bnoOk = false;                  // True if the BNO085 started.
float qi, qj, qk, qr;                // The 4 numbers of the quaternion (a way to store 3D rotation). Global = kept between loop() runs.
float linX, linY, linZ;              // BNO085 linear acceleration (gravity removed), m/s^2.
float gravX, gravY, gravZ;           // BNO085 gravity vector, m/s^2.
float magX, magY, magZ;              // BNO085 magnetic field, microtesla.
float accX, accY, accZ;              // LSM6DSO32 acceleration in m/s^2 (includes gravity).
float hpa, tempC, rh;                // Latest pressure, temperature and humidity values.
float uvs = 0, als = 0;              // Latest UV count and ambient-light count from the LTR390.
float groundHpa = 0;                 // Pressure at power-on; altitude is measured relative to this. 0 = not captured yet.
float maxAlt = 0;                    // Highest relative altitude seen so far, meters.
float maxG = 0;                      // Highest total acceleration seen so far, in g.
float maxGyro = 0;                   // Highest rotation rate seen so far, degrees/second.
float energyWh = 0;                  // Battery energy used so far, watt-hours (adds up power over time).
uint32_t launchMs = 0;               // millis() at the moment launch was detected; 0 = not launched yet.
uint32_t lastFrame = 0;              // millis() of the previous full update, used to measure time between frames.

// ---- Helper: read one byte from a register on an I2C chip -----------------
// A "register" is a numbered memory cell inside the chip. We ask for cell reg.
uint8_t rd8(uint8_t addr, uint8_t reg) {   // Function returning a 0-255 number; inputs: chip address and register number.
  Wire.beginTransmission(addr);            // Start talking to the chip at this address.
  Wire.write(reg);                         // Say which register we want to read.
  Wire.endTransmission(false);             // Finish sending but keep the bus held ("false") so we can read next.
  Wire.requestFrom(addr, (uint8_t)1);      // Ask the chip for 1 byte; (uint8_t) converts the 1 to the type the function expects.
  return Wire.read();                      // Take the received byte out of the buffer and hand it back to the caller.
}

// ---- Helper: write one byte into a register -------------------------------
void wr8(uint8_t addr, uint8_t reg, uint8_t v) { // No return value (void); inputs: chip, register, value to store.
  Wire.beginTransmission(addr);            // Start talking to the chip.
  Wire.write(reg);                         // First byte sent = which register.
  Wire.write(v);                           // Second byte sent = the value to put in it.
  Wire.endTransmission();                  // Finish and release the bus.
}

// ---- Helper: read a signed 16-bit number (two bytes) ----------------------
int16_t rd16(uint8_t addr, uint8_t reg) {  // int16_t = whole number from -32768 to 32767.
  Wire.beginTransmission(addr);            // Start talking to the chip.
  Wire.write(reg);                         // Choose the register.
  Wire.endTransmission(false);             // Keep the bus held for the read.
  Wire.requestFrom(addr, (uint8_t)2);      // Ask for 2 bytes.
  int16_t hi = Wire.read();                // First byte received is the high (big) half.
  return (hi << 8) | Wire.read();          // "<< 8" slides it left 8 bits; "|" merges it with the low byte read next.
}

// ---- Helper: read pressure from the LPS28DFW in hPa -----------------------
float readPressureHpa() {                  // Returns a decimal number.
  Wire.beginTransmission(LPS_ADDR);        // Talk to the pressure sensor.
  Wire.write(0x28);                        // Register 0x28 is where the 3-byte pressure result starts.
  Wire.endTransmission(false);             // Hold the bus.
  Wire.requestFrom((uint8_t)LPS_ADDR, (uint8_t)3); // Ask for 3 bytes (24-bit result).
  int32_t p = Wire.read();                 // Lowest byte first; int32_t has room for 4 bytes.
  p |= (int32_t)Wire.read() << 8;          // "|=" means "p = p | ..."; adds the middle byte shifted up 8 bits.
  p |= (int32_t)Wire.read() << 16;         // Adds the top byte shifted up 16 bits.
  if (p & 0x800000) p |= 0xFF000000;       // If the 24-bit number is negative (top bit set), fill upper bits so it is negative in 32 bits.
  return p / 4096.0;                       // The chip counts 4096 steps per hPa; dividing by a decimal gives a decimal answer.
}

// ---- Helper: ask the BNO085 for the data streams we want ------------------
void enableBnoReports() {                  // Called at startup and again if the BNO085 ever resets itself.
  bno.enableReport(SH2_ROTATION_VECTOR, 20000);     // Orientation quaternion, one report every 20000 microseconds (50 Hz).
  bno.enableReport(SH2_LINEAR_ACCELERATION, 20000); // Acceleration with gravity removed.
  bno.enableReport(SH2_GRAVITY, 20000);             // Which way gravity points, in the sensor's frame.
  //bno.enableReport(SH2_MAGNETIC_FIELD, 20000);      // Magnetic field (compass data).
}

// ---- Helper: print "ok" or "FAIL" on the display and Serial --------------
void report(const char* name, bool ok) {   // const char* = a piece of text; bool = true/false.
  Serial.print(name);                      // Send the sensor name to the Serial Monitor (no new line).
  Serial.println(ok ? " ok" : " FAIL");    // "a ? b : c" means: if a is true use b, otherwise c. println adds a new line.
  oled.print(name);                        // Also write the name into the display's memory.
  oled.println(ok ? " ok" : " FAIL");      // And the result.
  oled.display();                          // Push the display's memory to the actual screen so it appears.
}

// ===========================================================================
// setup() runs ONCE when the board powers on or resets.
// ===========================================================================
void setup() {
  Serial.begin(115200);                    // Open the USB serial port at 115200 bits per second (set the Monitor to match).
  while (!Serial && millis() < 3000) {}    // Wait up to 3 s for the PC to connect; "!" means NOT, "&&" means AND, {} = do nothing.
  Wire.begin();                            // Start the I2C bus on the STEMMA QT connector.
  Wire.setClock(100000);                   // Run the bus at 100,000 Hz (standard mode): slower but more forgiving of long wires and many devices.

  Serial.print("I2C devices:");            // Print a label, no new line, so the addresses follow on the same line.
  for (uint8_t ad = 1; ad < 127; ad++) {   // "for" loop: start ad at 1, repeat while ad < 127, add 1 each time (ad++).
    Wire.beginTransmission(ad);            // Try to talk to address ad.
    if (Wire.endTransmission() == 0) {     // 0 means a chip acknowledged (answered) at that address.
      Serial.print(" 0x"); Serial.print(ad, HEX); // Print the address in hexadecimal.
    }
  }
  Serial.println();                        // End the line.

  bool oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Try the display at 0x3C; begin() returns true if it worked.
  if (!oledOk) oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3D); // If not, try the other common address 0x3D.
  Serial.println(oledOk ? "OLED ok" : "OLED FAIL"); // Tell the Serial Monitor whether the display started.
  oled.clearDisplay();                     // Erase the display's memory.
  oled.setTextSize(1);                     // Text scale 1 = 6x8 pixel characters (21 characters per line, 8 lines).
  oled.setTextColor(SSD1306_WHITE);        // Draw text as lit pixels (the screen is single-color).
  oled.setCursor(0, 0);                    // Put the text cursor at the top-left corner (x = 0, y = 0).

  report("LSM6DSO32", lsm.begin_I2C(0x6A, &Wire)); // begin_I2C(address, bus) returns true if the chip answered.

  bnoOk = bno.begin_I2C(0x4A, &Wire);      // Start the BNO085 and remember true/false.
  if (bnoOk) enableBnoReports();           // If it started, ask for its data streams.
  report("BNO085", bnoOk);                 // Show the result.

  report("SHT31", sht.begin(0x44));        // Start the temperature/humidity sensor at address 0x44.

  bool l = ltr.begin(&Wire);               // Start the UV/light sensor on Wire.
  if (l) {                                 // Only configure it if it started; { } holds the two lines below.
    ltr.setMode(LTR390_MODE_UVS);          // Start in UV-measuring mode; loop() alternates between UV and light.
    ltr.setGain(LTR390_GAIN_3);            // Choose amplification level 3 (x3), good for bright light.
  }
  report("LTR390", l);                     // Show the result.

  bool g = gnss.begin(Wire, 0x42);         // Start the GPS module over I2C; 0x42 is the u-blox address.
  gnssOk = g;                              // Remember whether it answered, so loop() can skip it when it is missing.
  if (g) {                                 // Only configure it if it answered.
    gnss.setI2COutput(COM_TYPE_UBX);       // Use u-blox's binary protocol on I2C (what the library expects).
    gnss.setNavigationFrequency(1);        // Compute a position once per second.
  }
  report("GNSS", g);                       // Show the result.

  bool p = rd8(LPS_ADDR, 0x0F) == 0xB4;    // Register 0x0F is the chip ID; "==" compares; the LPS28DFW's ID is 0xB4.
  if (p) {                                 // If the right chip answered, configure it.
    wr8(LPS_ADDR, 0x11, 0x08);             // CTRL_REG2: set the BDU bit so high and low bytes always belong to the same reading.
    wr8(LPS_ADDR, 0x10, 0x1C);             // CTRL_REG1: sample 10 times/second with averaging (this also turns measuring on).
  }
  report("LPS28DFW", p);                   // Show the result.

  delay(1500);                             // Pause 1.5 s (1500 milliseconds) so you can read the checklist.
  Serial.println("ms,ax,ay,az,gx,gy,gz,qi,qj,qk,qr,hPa,tempC,rh,uvs,vbus,amps,lat,lon,altM,sats,yaw,pitch,roll,gTotal,relAltM,linX,linY,linZ,magX,magY,magZ,headingDeg,als,dewC,watts,energyWh,maxG,maxGyroDps,flightS,freeFall"); // CSV header row: column names.
}

// ===========================================================================
// loop() runs over and over, forever, after setup() finishes.
// ===========================================================================
void loop() {
  if (bnoOk) {                             // Only poll the BNO085 if it started.
    if (bno.wasReset()) enableBnoReports();// If the chip restarted itself, it forgot our requests, so ask again.
    if (bno.getSensorEvent(&bnoVal)) {     // Is a new BNO report waiting? (fills bnoVal)
      switch (bnoVal.sensorId) {           // "switch" jumps to the case that matches which kind of report arrived.
        case SH2_ROTATION_VECTOR:          // Orientation report.
          qi = bnoVal.un.rotationVector.i; // Copy the quaternion's i part out of the report struct (the "." walks into nested fields).
          qj = bnoVal.un.rotationVector.j; // j part.
          qk = bnoVal.un.rotationVector.k; // k part.
          qr = bnoVal.un.rotationVector.real; // Real (w) part.
          break;                           // "break" leaves the switch so the next cases are not also run.
        case SH2_LINEAR_ACCELERATION:      // Acceleration without gravity.
          linX = bnoVal.un.linearAcceleration.x; // X part.
          linY = bnoVal.un.linearAcceleration.y; // Y part.
          linZ = bnoVal.un.linearAcceleration.z; // Z part.
          break;                           // Done with this case.
        case SH2_GRAVITY:                  // Gravity direction.
          gravX = bnoVal.un.gravity.x;     // X part.
          gravY = bnoVal.un.gravity.y;     // Y part.
          gravZ = bnoVal.un.gravity.z;     // Z part.
          break;                           // Done with this case.
    /*    case SH2_MAGNETIC_FIELD:           // Compass data.
          magX = bnoVal.un.magneticField.x;// X part in microtesla.
          magY = bnoVal.un.magneticField.y;// Y part.
          magZ = bnoVal.un.magneticField.z;// Z part.
          break;   */                        // Done with this case.
      }
    }
  }

  static uint32_t last = 0;                // "static" keeps this value between runs of loop(); stores when we last printed.
  if (millis() - last < 200) return;       // Less than 200 ms since the last full update? Leave early; loop() restarts right away, which keeps polling the BNO.
  last = millis();                         // 200 ms have passed: remember the current time for the next check.
  float dt = (millis() - lastFrame) / 1000.0; // Seconds since the previous full update (used for energy used).
  if (lastFrame == 0) dt = 0;              // The very first frame has no previous frame, so count no time.
  lastFrame = millis();                    // Remember this frame's time.

  // ---- read the sensors ---------------------------------------------------
  sensors_event_t a, g, t;                 // Three structs that will hold accelerometer, gyro and temperature readings.
  lsm.getEvent(&a, &g, &t);                // Ask the LSM6DSO32 to fill all three; & passes the variables themselves so it can change them.
  accX = a.acceleration.x;                 // Save X acceleration (m/s^2).
  accY = a.acceleration.y;                 // Save Y acceleration.
  accZ = a.acceleration.z;                 // Save Z acceleration.
  hpa = readPressureHpa();                 // Read pressure using our helper function.
  tempC = sht.readTemperature();           // Read temperature in degrees Celsius.
  rh = sht.readHumidity();                 // Read relative humidity in percent.
  float vbus = rd16(INA_ADDR, 0x05) * 0.003125;          // Register 0x05 = bus voltage; each count is 3.125 millivolts = 0.003125 V.
  float amps = rd16(INA_ADDR, 0x04) * 0.000005 / SHUNT_OHMS; // Register 0x04 = shunt voltage (5 microvolts/count); amps = volts / ohms.

  if (ltr.newDataAvailable()) {            // Has the LTR390 finished a new measurement?
    static bool uvMode = true;             // Remembers which mode we are in: true = UV, false = ambient light.
    if (uvMode) {                          // We were measuring UV.
      uvs = ltr.readUVS();                 // Save the UV count.
      ltr.setMode(LTR390_MODE_ALS);        // Switch to ambient light for the next measurement.
    } else {                               // We were measuring light.
      als = ltr.readALS();                 // Save the light count.
      ltr.setMode(LTR390_MODE_UVS);        // Switch back to UV.
    }
    uvMode = !uvMode;                      // "!" flips true/false so we alternate each time.
  }

  float lat = 0, lon = 0, alt = 0;         // Start at 0 so we never print garbage when there is no fix.
  int sats = 0;                            // Satellite count, 0 until the GPS answers.
  if (gnssOk && gnss.getPVT() && gnss.getFixType() >= 2) { // getPVT() asks for fresh data; fix type 2 or more = a real 2D/3D position.
    lat = gnss.getLatitude() / 1e7;        // GPS gives degrees x 10,000,000 as an integer; 1e7 means 1 x 10^7.
    lon = gnss.getLongitude() / 1e7;       // Same for longitude.
    alt = gnss.getAltitudeMSL() / 1000.0;  // Altitude comes in millimeters; dividing by 1000.0 makes meters.
    sats = gnss.getSIV();                  // Number of satellites used in the position.
  }

  // ---- calculate derived values ---------------------------------------------
  float roll  = atan2(2 * (qr * qi + qj * qk), 1 - 2 * (qi * qi + qj * qj)) * RAD2DEG; // Roll (tilt side to side) from the quaternion, radians converted to degrees.
  float pitch = asin(constrain(2 * (qr * qj - qk * qi), -1.0, 1.0)) * RAD2DEG;          // Pitch (nose up/down); constrain keeps asin's input legal (-1 to 1).
  float yaw   = atan2(2 * (qr * qk + qi * qj), 1 - 2 * (qj * qj + qk * qk)) * RAD2DEG;  // Yaw (spin about the vertical axis).
  float gTotal = sqrt(accX * accX + accY * accY + accZ * accZ) / GRAVITY; // Total acceleration in g (square root of x^2+y^2+z^2); 1.00 sitting still.
  float gyroDps = sqrt(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z) * RAD2DEG; // Total rotation rate in degrees/second.
  float magTotal = sqrt(magX * magX + magY * magY + magZ * magZ); // Strength of the magnetic field in microtesla (Earth is about 25-65).
  float heading = atan2(magY, magX) * RAD2DEG; // Compass heading from the magnetic field; only accurate when the board is held level.
  if (heading < 0) heading += 360;        // Turn a negative angle into 0-360.

  if (groundHpa == 0 && hpa > 300) groundHpa = hpa; // The first sensible pressure reading becomes "ground level".
  float relAlt = 0;                        // Altitude above the starting point, meters.
  if (groundHpa > 0) relAlt = 44330.0 * (1.0 - pow(hpa / groundHpa, 0.1903)); // Standard barometric formula; pow(a,b) = a to the power b.
  maxAlt = fmaxf(maxAlt, relAlt);          // fmaxf returns the larger of two decimals; keeps the highest altitude seen.

  float dewC = 0;                          // Dew point: the temperature at which water would start to condense.
  if (rh > 0) {                            // log(0) is not allowed, so only calculate when humidity is above 0.
    float gam = log(rh / 100.0) + (17.62 * tempC) / (243.12 + tempC); // Magnus formula, step 1 (log = natural logarithm).
    dewC = 243.12 * gam / (17.62 - gam);   // Magnus formula, step 2.
  }

  float watts = vbus * amps;               // Electrical power in watts = volts x amps.
  energyWh += watts * dt / 3600.0;         // Add this frame's energy; dt seconds / 3600 = hours.

  maxG = fmaxf(maxG, gTotal);              // Keep the highest total acceleration seen.
  maxGyro = fmaxf(maxGyro, gyroDps);       // Keep the highest rotation rate seen.
  if (launchMs == 0 && gTotal > LAUNCH_G) launchMs = millis(); // First time acceleration passes LAUNCH_G, record the moment as launch.
  float flightS = (launchMs == 0) ? 0 : (millis() - launchMs) / 1000.0; // Seconds since launch (0 before launch).
  bool freeFall = gTotal < FREEFALL_G;     // True when almost no acceleration is felt (falling freely).

  // ---- print one CSV line -------------------------------------------------
  Serial.print(millis()); Serial.print(',');   // print() writes a value without a new line; ',' separates CSV columns.
  Serial.print(accX); Serial.print(',');       // Acceleration X.
  Serial.print(accY); Serial.print(',');       // Acceleration Y.
  Serial.print(accZ); Serial.print(',');       // Acceleration Z.
  Serial.print(g.gyro.x); Serial.print(',');   // Rotation rate X (rad/s).
  Serial.print(g.gyro.y); Serial.print(',');   // Rotation rate Y.
  Serial.print(g.gyro.z); Serial.print(',');   // Rotation rate Z.
  Serial.print(qi); Serial.print(',');         // Quaternion i.
  Serial.print(qj); Serial.print(',');         // Quaternion j.
  Serial.print(qk); Serial.print(',');         // Quaternion k.
  Serial.print(qr); Serial.print(',');         // Quaternion real.
  Serial.print(hpa); Serial.print(',');        // Pressure.
  Serial.print(tempC); Serial.print(',');      // Temperature.
  Serial.print(rh); Serial.print(',');         // Humidity.
  Serial.print(uvs, 0); Serial.print(',');     // UV count.
  Serial.print(vbus); Serial.print(',');       // Battery bus voltage.
  Serial.print(amps); Serial.print(',');       // Current draw.
  Serial.print(lat, 6); Serial.print(',');     // The second input (6) = show 6 digits after the decimal point.
  Serial.print(lon, 6); Serial.print(',');     // Longitude with 6 decimals.
  Serial.print(alt); Serial.print(',');        // GPS altitude in meters.
  Serial.print(sats); Serial.print(',');       // Satellite count.
  Serial.print(yaw, 1); Serial.print(',');     // Yaw in degrees (1 decimal).
  Serial.print(pitch, 1); Serial.print(',');   // Pitch in degrees.
  Serial.print(roll, 1); Serial.print(',');    // Roll in degrees.
  Serial.print(gTotal, 2); Serial.print(',');  // Total acceleration in g.
  Serial.print(relAlt, 1); Serial.print(',');  // Altitude from pressure, relative to start.
  Serial.print(linX, 2); Serial.print(',');    // Linear acceleration X (gravity removed).
  Serial.print(linY, 2); Serial.print(',');    // Linear acceleration Y.
  Serial.print(linZ, 2); Serial.print(',');    // Linear acceleration Z.
  Serial.print(magX, 1); Serial.print(',');    // Magnetic field X.
  Serial.print(magY, 1); Serial.print(',');    // Magnetic field Y.
  Serial.print(magZ, 1); Serial.print(',');    // Magnetic field Z.
  Serial.print(heading, 0); Serial.print(','); // Compass heading in degrees.
  Serial.print(als, 0); Serial.print(',');     // Ambient light count.
  Serial.print(dewC, 1); Serial.print(',');    // Dew point.
  Serial.print(watts, 2); Serial.print(',');   // Power in watts.
  Serial.print(energyWh, 3); Serial.print(','); // Energy used in watt-hours.
  Serial.print(maxG, 2); Serial.print(',');    // Highest acceleration so far.
  Serial.print(maxGyro, 0); Serial.print(','); // Highest rotation rate so far.
  Serial.print(flightS, 1); Serial.print(','); // Seconds since launch.
  Serial.println(freeFall ? 1 : 0);            // 1 if in free fall, 0 if not; println ends the CSV line.

  // ---- draw one page on the display (the page changes every 3 seconds) ---------
  oled.ssd1306_command(SSD1306_DISPLAYON);     // Send "display on" every frame, in case a noisy bus glitch switched the screen off.
  oled.clearDisplay();                         // Wipe the display memory before drawing the new frame.
  oled.setCursor(0, 0);                        // Back to the top-left.
  int page = (millis() / 3000) % 5;            // Time in 3-second steps; "% 5" (remainder after dividing by 5) cycles 0,1,2,3,4,0,1,...
  if (page == 0) {                             // PAGE 0: acceleration and environment.
    oled.print("Ax ");  oled.println(accX, 1); // Write text, then the value with 1 decimal; println moves to the next line.
    oled.print("Ay ");  oled.println(accY, 1); // Y acceleration.
    oled.print("Az ");  oled.println(accZ, 1); // Z acceleration.
    oled.print("Total "); oled.print(gTotal, 2); oled.println(" g"); // Total acceleration in g.
    oled.print("P  ");  oled.print(hpa, 1); oled.println(" hPa"); // Pressure with units.
    oled.print("T  ");  oled.print(tempC, 1); oled.print("C H "); oled.print(rh, 0); oled.println("%"); // Temperature and humidity.
    oled.print("Dew "); oled.print(dewC, 1); oled.println(" C"); // Dew point.
    oled.print("GPS "); oled.print(sats); oled.println(" sats"); // GPS satellite count.
  } else if (page == 1) {                      // PAGE 1: orientation and rotation rates.
    oled.println("ORIENTATION");               // Page title.
    oled.print("Yaw   "); oled.print(yaw, 1); oled.println(" deg"); // Yaw angle.
    oled.print("Pitch "); oled.print(pitch, 1); oled.println(" deg"); // Pitch angle.
    oled.print("Roll  "); oled.print(roll, 1); oled.println(" deg"); // Roll angle.
    oled.print("Gx "); oled.print(g.gyro.x, 2); oled.println(" r/s"); // Gyro X in radians per second.
    oled.print("Gy "); oled.print(g.gyro.y, 2); oled.println(" r/s"); // Gyro Y.
    oled.print("Gz "); oled.print(g.gyro.z, 2); oled.println(" r/s"); // Gyro Z.
    oled.print("Rate "); oled.print(gyroDps, 0); oled.println(" dps"); // Total rotation rate in degrees/second.
  } else if (page == 2) {                      // PAGE 2: gravity-free motion and compass.
    oled.println("MOTION / COMPASS");          // Page title.
    oled.print("LinX "); oled.print(linX, 2); oled.println(" m/s2"); // Linear acceleration X (gravity removed).
    oled.print("LinY "); oled.print(linY, 2); oled.println(" m/s2"); // Linear acceleration Y.
    oled.print("LinZ "); oled.print(linZ, 2); oled.println(" m/s2"); // Linear acceleration Z.
    oled.print("Mag "); oled.print(magTotal, 1); oled.println(" uT"); // Magnetic field strength.
    oled.print("Head "); oled.print(heading, 0); oled.println(" deg"); // Compass heading.
    oled.print("FreeFall "); oled.println(freeFall ? "YES" : "no"); // Free-fall flag.
    oled.print("Launch "); oled.println(launchMs ? "YES" : "no"); // Launch-detected flag.
  } else if (page == 3) {                      // PAGE 3: altitude and light.
    oled.println("ALTITUDE / LIGHT");          // Page title.
    oled.print("Alt "); oled.print(relAlt, 1); oled.println(" m"); // Altitude above start, from pressure.
    oled.print("Max "); oled.print(maxAlt, 1); oled.println(" m"); // Highest altitude so far.
    oled.print("P   "); oled.print(hpa, 2); oled.println(" hPa"); // Pressure again, more decimals.
    oled.print("UV  "); oled.println(uvs, 0);  // UV count.
    oled.print("Lt  "); oled.println(als, 0);  // Ambient light count.
    oled.print("GPSalt "); oled.print(alt, 0); oled.println(" m"); // GPS altitude (0 with no fix).
    oled.print("Lat "); oled.println(lat, 4);  // GPS latitude, 4 decimals.
  } else {                                     // PAGE 4: power and flight summary.
    oled.println("POWER / FLIGHT");            // Page title.
    oled.print("V "); oled.print(vbus, 2); oled.println(" V"); // Bus voltage.
    oled.print("I "); oled.print(amps, 2); oled.println(" A"); // Current.
    oled.print("P "); oled.print(watts, 2); oled.println(" W"); // Power.
    oled.print("E "); oled.print(energyWh, 3); oled.println(" Wh"); // Energy used so far.
    oled.print("MaxG "); oled.println(maxG, 2);// Highest total acceleration.
    oled.print("MaxRate "); oled.print(maxGyro, 0); oled.println(" dps"); // Highest rotation rate.
    oled.print("Flight "); oled.print(flightS, 1); oled.println(" s"); // Seconds since launch.
  }
  oled.display();                              // Send the finished frame to the screen. Nothing shows until this runs.
}
