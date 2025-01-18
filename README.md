This codebase contains the program files for RescueBot, our team's Design Project for the class EECS 467: Autonomous Robotics taken at the University of Michigan in Winter of 2023. Most of the code is based on Botlab, the first major component of the course, which includes implementation for motion control, SLAM, path planning and the LCM messaging framework and is owned by the EECS 467 Instructional Team.

![image](https://github.com/user-attachments/assets/b8eb8938-477e-41b4-adbc-65a14f0a1a94)

Our additions to the codebase include the python script "realsense.thermal.py", which recieves data from the Intel Realsense 435i stereo camera and the MLX90640 IR thermal camera. We use the Intel Realsense SDK, Adafruit drivers and OpenCV to extract and process this data, which is published as an LCM message. Our group has made major modifications to the mapping, exploration, motion controller and GUI programs as well to support the new thermal mapping and rescue features.

Read the final report [here]("./Final%20Report.pdf") or watch the demo [here](https://www.youtube.com/watch?v=BWogS4yuJig)
