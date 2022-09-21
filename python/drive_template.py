import lcm
import numpy as np
import time
from threading import Thread, Lock, current_thread
from lcmtypes import mbot_motor_command_t, timestamp_t, mbot_encoder_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

DRIVE_LENGTH = 1
STOP_LENGTH = 0.2
ROTATE_LENGTH = 1

cur_encoder_values = None

def current_utime(): return int(time.time() * 1e6)

def encoder_message_handler(channel, data):
    global cur_encoder_values
    cur_encoder_values = mbot_encoder_t.decode(data)
    
def handle_lcm(lcm_obj):
    try:
        while True:
            lcm_obj.handle()
    except KeyboardInterrupt:
        print("lcm exit!")
        sys.exit()

def main():
    print("creating LCM ...")
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    lcm_kill_thread = Thread(target = handle_lcm, args= (lc, ), daemon = True)
    lcm_kill_thread.start()
    subscription = lc.subscribe("MBOT_ENCODERS", encoder_message_handler)
    time.sleep(0.5)
    for i in range(4):
        print('resetting encoders')
        cur_encoder_msg = mbot_encoder_t()
        lc.publish("RESET_ENCODERS", cur_encoder_msg.encode())
        time.sleep(0.25)
        print("Left ticks: " + str(cur_encoder_values.leftticks))
        print("Left delta: " + str(cur_encoder_values.left_delta))
        print("Right ticks: " + str(cur_encoder_values.rightticks))
        print("Right delta: " + str(cur_encoder_values.right_delta))

        print('Driving Straight')
        drive = mbot_motor_command_t()
        drive.utime = current_utime()
        drive.trans_v = 0.25
        drive.angular_v = 0.0

        drive_time = timestamp_t()
        drive_time.utime = drive.utime

        lc.publish("MBOT_TIMESYNC", drive_time.encode())
        lc.publish("MBOT_MOTOR_COMMAND", drive.encode())
        time.sleep(4)


        print('Stopping')
        # Stop
        stop = mbot_motor_command_t()
        stop.utime = current_utime()
        stop.trans_v = 0.0
        stop.angular_v = 0.0

        stop_time = timestamp_t()
        stop_time.utime = stop.utime
        lc.publish("MBOT_TIMESYNC", stop_time.encode())
        lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
        time.sleep(1)
        print("Left ticks: " + str(cur_encoder_values.leftticks))
        print("Left delta: " + str(cur_encoder_values.left_delta))
        print("Right ticks: " + str(cur_encoder_values.rightticks))
        print("Right delta: " + str(cur_encoder_values.right_delta))
    
        time.sleep(0.25)
        print('resetting encoders')
        cur_encoder_msg = mbot_encoder_t()
        lc.publish("RESET_ENCODERS", cur_encoder_msg.encode())
        print("Left ticks: " + str(cur_encoder_values.leftticks))
        print("Left delta: " + str(cur_encoder_values.left_delta))
        print("Right ticks: " + str(cur_encoder_values.rightticks))
        print("Right delta: " + str(cur_encoder_values.right_delta))
       
 
        time.sleep(0.25)
        # # Rotate
        print('Rotating')
        rotate = mbot_motor_command_t()
        rotate.utime = current_utime()
        rotate.trans_v = 0.0
        rotate.angular_v = np.pi / 2.0

        rotate_time = timestamp_t()
        rotate_time.utime = rotate.utime
        lc.publish("MBOT_TIMESYNC", rotate_time.encode())
        lc.publish("MBOT_MOTOR_COMMAND", rotate.encode())
        time.sleep(1)


        print('Stopping')
        # Stop
        stop = mbot_motor_command_t()
        stop.utime = current_utime()
        stop.trans_v = 0.0
        stop.angular_v = 0.0

        stop_time = timestamp_t()
        stop_time.utime = stop.utime
        lc.publish("MBOT_TIMESYNC", stop_time.encode())
        lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
        time.sleep(0.5)


        print("Left ticks: " + str(cur_encoder_values.leftticks))
        print("Left delta: " + str(cur_encoder_values.left_delta))
        print("Right ticks: " + str(cur_encoder_values.rightticks))
        print("Right delta: " + str(cur_encoder_values.right_delta))
        time.sleep(0.25)
    



