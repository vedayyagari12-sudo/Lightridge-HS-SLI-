"""
Note that this code isn't complete. 
It contains bugs and a invalid path to the external SSD.
"""



import os
import time
from picamera2 import Picamera2

# 1. Define the storage directory on your External SSD
# REPLACE this path with your actual SSD mount path from 'df -h'
SSD_TARGET_DIR = "/media/pi/YOUR_SSD_NAME/captured_images"

# Create the directory if it doesn't exist using the os module
if not os.path.exists(SSD_TARGET_DIR):
    os.makedirs(SSD_TARGET_DIR)
    print(f"Created target directory: {SSD_TARGET_DIR}")
else:
    print(f"Saving images to: {SSD_TARGET_DIR}")

# 2. Initialize Picamera2
picam2 = Picamera2()

# 3. Configure for 4K Resolution (3840 x 2160) using a continuous streaming profile
# This profile maintains high frame throughput without slow still-photo mode switching
camera_config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (3840, 2160)},
    controls={"FrameRate": 10.0} # Lower bound frame rate to ensure steady exposure
)

picam2.configure(camera_config)
picam2.start()

# Allow the camera sensor auto-exposure/white balance to settle
time.sleep(2)

print("Starting 3 Hz capture loop. Press Ctrl+C in Thonny to stop.")

# Target interval for 3 Hz (3 pictures per second)
INTERVAL = 1.0 / 3.0
img_counter = 0

try:
    next_capture_time = time.time()
   
    while True:
        current_time = time.time()
       
        # Precise timing loop execution
        if current_time >= next_capture_time:
            img_counter += 1
           
            # Construct file path using the os module
            filename = f"image_4k_{img_counter:05d}.jpg"
            filepath = os.path.join(SSD_TARGET_DIR, filename)
           
            # Capture a single high-quality frame directly out of the stream pipeline
            # This completely avoids encoding a video file while preserving 4K clarity
            picam2.capture_file(filepath, quality=95)
           
            print(f"Saved: {filename} at {time.strftime('%H:%M:%S')}")
           
            # Schedule the next exact 3 Hz interval mark
            next_capture_time += INTERVAL
           
        # Tiny sleep to prevent maximizing CPU utilization while waiting
        time.sleep(0.005)

except KeyboardInterrupt:
    print("\nCapture stopped by user.")

finally:
    # Clean up and close camera resources safely
    picam2.stop()
    print("Camera released safely.")
# 
# Not updated