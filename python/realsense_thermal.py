import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import time,board,busio
import numpy as np
import adafruit_mlx90640
from scipy import ndimage
import scipy.misc
import lcm
from thermal_depth_t import thermal_depth_t

def current_utime(): return int(time.time() * 1e6)

# Configure thermal camera
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000) # setup I2C
mlx = adafruit_mlx90640.MLX90640(i2c) # begin MLX90640 with I2C comm
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ # set refresh rate
mlx_shape = (24,32)
frame = np.zeros((24*32,)) # setup array for storing all 768 temperatures

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

# Start streaming
profile = pipeline.start(config)
# device = profile.get_device() # type: rs.device
# depthSensor = device.first_depth_sensor() # type: rs.depth_sensor
# if depthSensor.supports(rs.option.depth_units):
#     depthSensor.set_option(rs.option.depth_units, 1.000)

# Initialize LCM handler
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")


try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        if not depth_frame:
            continue
        
        try:
                mlx.getFrame(frame) # read MLX temperatures into frame var
                thermal_array = (np.reshape(frame,mlx_shape)) # reshape to 24x32
                # thermal_array = thermal_array.astype(np.intc)
                thermal_array = np.asanyarray(thermal_array, dtype = float)
                thermal_array = cv2.flip(thermal_array, 1)

        except ValueError:
                continue # if error, just read again

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data(), dtype = float)
         
        depth_matrix = np.array([[ 1.74135451e+00,  1.01058261e-01, -3.23461390e+02],
         [-1.73114838e-01,  1.90259195e+00, -1.23924668e+02],
         [-3.63572321e-04,  6.43620375e-05,  1.06056044e+00]])
        
        depth_image = cv2.warpPerspective(
            depth_image, 
            depth_matrix, 
            (640, 480)
        )

        depth_image = cv2.resize(depth_image,(32,24))

        ## compute horizontal offset 
        horiz_offset = np.zeros((24,32))

        for x in range(32):
            for y in range(24):
                horiz_offset[y][x] = ((x - 16) * depth_image[y][x])/ 38; 
        ## pack data in lcm
        message = thermal_depth_t()
        message.utime = current_utime()
        message.distance_y = -horiz_offset.flatten()
        message.distance_x = depth_image.flatten()
        message.temperature = thermal_array.flatten()

        # print(horiz_offset[7][31])
        # print(depth_image[0][0])

        # visualization
        # thermal_img = cv2.normalize(thermal_array, None, 0, 225, cv2.NORM_MINMAX, cv2.CV_8U)
        # thermal_img = cv2.applyColorMap(thermal_img, cv2.COLORMAP_JET)  
        # cv2.namedWindow('Thermal', cv2.WINDOW_NORMAL)
        # cv2.imshow('Thermal', thermal_img)
        # cv2.waitKey(1)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # cv2.namedWindow('RealSense Depth', cv2.WINDOW_NORMAL)
        # cv2.imshow('RealSense Depth', depth_colormap)
        # cv2.waitKey(1)

        lc.publish("THERMAL_DEPTH", message.encode())
        print("Sent lcm...")
        time.sleep(0.25)

finally:

    # Stop streaming
    pipeline.stop()