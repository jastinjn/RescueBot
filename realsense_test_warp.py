import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import time,board,busio
import numpy as np
import adafruit_mlx90640
from scipy import ndimage
import scipy.misc

# Configure thermal camera
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000) # setup I2C
mlx = adafruit_mlx90640.MLX90640(i2c) # begin MLX90640 with I2C comm
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ # set refresh rate
mlx_shape = (24,32)
frame = np.zeros((24*32,)) # setup array for storing all 768 temperatures

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        
        try:
                mlx.getFrame(frame) # read MLX temperatures into frame var
                thermal_array = (np.reshape(frame,mlx_shape)) # reshape to 24x32
                thermal_array = cv2.normalize(thermal_array, None, 0, 225, cv2.NORM_MINMAX, cv2.CV_8U)
                thermal_array = thermal_array.astype(np.uint8)
                thermal_img = cv2.applyColorMap(thermal_array, cv2.COLORMAP_JET)
                thermal_img = cv2.flip(cv2.resize(thermal_img, (680, 480)), 1)
                #thermal_img = cv2.flip(thermal_img, 1)

                # Show images
                cv2.namedWindow('Thermal', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Thermal', thermal_img)
                cv2.waitKey(1)

        except ValueError:
                continue # if error, just read again

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # Crop depth image to match thermal image
        # horiz_offset = 180 # adjust this
        # vert_offset = 90 # adjust this
        # horiz_length = depth_image.shape[0]
        # vert_length = depth_image.shape[1]
        # cropped_depth_image = depth_image[vert_offset:10*thermal_array.shape[0]+vert_offset, horiz_offset:11*thermal_array.shape[1]+horiz_offset]
        # depth_image = cv2.resize(cropped_depth_image, (640, 480))
        color_matrix = np.array([[ 1.49923155e+00,  3.01440290e-01, -2.51073451e+02],
         [-7.00850514e-02,  1.49219969e+00, -4.03885263e+01],
         [-4.65842092e-05,  5.00506872e-04,  9.90157094e-01]])
         
        depth_matrix = np.array([[ 2.62054236e+00,  8.62819151e-02, -5.99270152e+02,],
         [ 1.83132367e-01,  2.29747483e+00, -2.91432067e+02],
         [ 9.93229728e-04, -1.86746494e-04,  7.96765889e-01]])
        
        color_image = cv2.warpPerspective(
            color_image, 
            color_matrix, 
            (640, 480)
        )
        
        depth_image = cv2.warpPerspective(
            depth_image, 
            depth_matrix, 
            (640, 480)
        )
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)
        
        cv2.namedWindow('RealSense Depth', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense Depth', depth_colormap)
        cv2.waitKey(1)
        


finally:

    # Stop streaming
    pipeline.stop()
