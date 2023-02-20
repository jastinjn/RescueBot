import os
import sys

import lcm
import matplotlib.pyplot as plt
import numpy as np

from lcmtypes import timestamp_t, odometry_t, pose_xyt_t, particles_t


def is_between(a, b, c):
    return a <= c <= b or b <= c <= a


sys.path.append("lcmtypes")

if len(sys.argv) < 2:
    sys.stderr.write("usage: plot_slam_pose.py <logfile>")
    sys.exit(1)

file = sys.argv[1]
log = lcm.EventLog(file, "r")

timesync_data = np.empty((0, 1), dtype=int)

odometry_data = np.empty((0, 4), dtype = float)
odometry_init = 0

slam_pose_data = np.empty((0,4), dtype = float)
slam_pose_init = 0

true_pose_data = np.empty((0,4), dtype = float)
true_pose_init = 0

particles_data = np.empty((0,2), dtype = float)
particles_init = 0

for event in log:

    if event.channel == "MBOT_TIMESYNC":
        timesync_msg = timestamp_t.decode(event.data)
        timesync_data = np.append(timesync_data, np.array([[
            (timesync_msg.utime)/1.0E6,
        ]]), axis=0)

    if event.channel == "ODOMETRY":
        odometry_msg = odometry_t.decode(event.data)
        if odometry_init == 0:
            odom_start_utime = odometry_msg.utime
            print("odom_start_utime: {}".format(odom_start_utime))
            odometry_init = 1
        odometry_data = np.append(odometry_data, np.array([[
            (odometry_msg.utime - odom_start_utime)/1.0E6,
            odometry_msg.x,
            odometry_msg.y,
            odometry_msg.theta,
        ]]), axis=0)

    if event.channel == "SLAM_POSE":
        slam_msg = pose_xyt_t.decode(event.data)
        if slam_pose_init == 0:
            slam_start_utime = slam_msg.utime
            print("slam_start_utime: {}".format(slam_start_utime))
            slam_pose_init = 1
        slam_pose_data = np.append(slam_pose_data, np.array([[
            (slam_msg.utime - slam_start_utime)/1.0E6,
            slam_msg.x,
            slam_msg.y,
            slam_msg.theta,
        ]]), axis=0)

    if event.channel == "TRUE_POSE":
        true_msg = pose_xyt_t.decode(event.data)
        if true_pose_init == 0:
            true_start_utime = true_msg.utime
            print("slam_start_utime: {}".format(true_start_utime))
            true_pose_init = 1
        true_pose_data = np.append(true_pose_data, np.array([[
            (true_msg.utime - true_start_utime)/1.0E6,
            true_msg.x,
            true_msg.y,
            true_msg.theta,
        ]]), axis=0)
    
    if event.channel == "SLAM_PARTICLES":
        part_msg = particles_t.decode(event.data)
        if particles_init == 0:
            part_start_utime = part_msg.utime
            print("particles_utime: {}".format(part_start_utime))
            particles_init = 1
        particles_data = np.append(particles_data, np.array([[
            (part_msg.utime - part_start_utime)/1.0E6,
            part_msg.num_particles,
        ]]), axis=0)

# Odometry data 
odom_time = odometry_data[:, 0]
odom_x = odometry_data[:, 1]
odom_y = odometry_data[:, 2]
odom_heading = odometry_data[:, 3]

# Slam data
slam_time = slam_pose_data[:, 0]
slam_x = slam_pose_data[:, 1]
slam_y = slam_pose_data[:, 2]
slam_delta = slam_pose_data[:, 3]

first_index = np.argmax(slam_x>0.0)

slam_time = slam_time[first_index:]
slam_x = slam_x[first_index:]
slam_y = slam_y[first_index:]
slam_delta = slam_delta[first_index:]



# particles_time = particles_data[:, 0]
# num_particles = particles_data[:, 1]
# print(particles_time.size)
# particles_time = particles_time[first_index:]
# num_particles = num_particles[first_index:]



# Ground truth
true_time = true_pose_data[:, 0]
true_x = true_pose_data[:, 1]
true_y = true_pose_data[:, 2]
true_delta = true_pose_data[:, 3]

# Calculate frequency of slam updates
update_time = slam_time[1:] - slam_time[:-1]
update_freq = 1/update_time
avg_update_time = np.sum(update_time)/update_time.size

# Calculate RMS Error from SLAM and TRUE

# time offset
offset = slam_time[np.argmax(slam_x > 0.1)] - true_time[np.argmax(true_x > 0.1)]
true_time += offset

error_position = np.zeros(slam_time.size)
error_theta = np.zeros(slam_time.size)
j = 0
for i  in range(slam_time.size):
    if j == true_time.size - 1: break
    # find first particle with matching time stamp
    while true_time[j] < slam_time[i] and j < true_time.size-1:
        j += 1
    error_theta[i] = abs(true_delta[j]-slam_delta[i])
    if error_theta[i] > np.pi : error_theta[i] = abs(error_theta[i] - 2* np.pi)
    error_position[i] = ((true_x[j]-slam_x[i])**2 + (true_y[j]-slam_y[i])**2)**0.5
    #print("true " + true_x[j].astype(str) + "slam " + slam_x[i].astype(str) + "time" + true_time[j].astype(str) + " " + slam_time[i].astype(str))

error_pos_mean = np.sum(error_position)/error_position.size
error_theta_mean = np.sum(error_theta)/error_theta.size

error_pos_rms = (np.sum(error_position**2)/error_position.size)**0.5
error_theta_rms = (np.sum(error_theta**2)/error_theta.size)**0.5

# Calculate Error from Odometry and SLAM
offset = slam_time[np.argmax(slam_x > 0.1)] - odom_time[np.argmax(odom_x > 0.1)]
odom_time += offset

error_odom = np.zeros(slam_time.size)
j = 0
for i  in range(slam_time.size):
    if j == odom_time.size - 1: break
    # find first particle with matching time stamp
    while odom_time[j] < slam_time[i] and j < odom_time.size-1:
        j += 1
    error_odom[i] = ((odom_x[j]-slam_x[i])**2 + (odom_y[j]-slam_y[i])**2)**0.5

error_odom_mean = np.sum(error_odom)/error_odom.size
error_odom_rms = (np.sum(error_odom**2)/error_odom.size)**0.5

## plot slam data for time checking
plt.plot(slam_time,slam_x,true_time, true_x, odom_time, odom_x)
plt.legend(["slam","true"])
plt.show()
plt.clf()

## Plot Error from Ground Truth
plt.plot(slam_time, error_position, slam_time, np.ones(slam_time.size) * error_pos_mean,slam_time, np.ones(slam_time.size) * error_pos_rms)
plt.legend(['Error', 'Mean', 'RMS'])
plt.xlabel("Time /s")
plt.ylabel("Error /m")
plt.title("Mean Error " + error_pos_mean.astype(str) + " RMS Error " + error_pos_rms.astype(str))
plt.show()
plt.clf()

plt.plot(slam_time, error_theta, slam_time, np.ones(slam_time.size) * error_theta_mean, slam_time, np.ones(slam_time.size) * error_theta_rms)
plt.legend(['Error', 'Mean', 'RMS'])
plt.xlabel("Time /s")
plt.ylabel("Error /rads")
plt.title("Mean Error " + error_theta_mean.astype(str) + " RMS Error " + error_theta_rms.astype(str))
plt.show()
plt.clf()

## Plot Error from Odometry Truth
plt.plot(slam_time, error_odom, slam_time, np.ones(slam_time.size) * error_odom_mean, slam_time, np.ones(slam_time.size) * error_odom_rms)
plt.legend(['Error', 'Mean', 'RMS'])
plt.xlabel("Time /s")
plt.ylabel("Error /m")
plt.title("Odom Mean Error " + error_pos_mean.astype(str) + " RMS Error " + error_pos_rms.astype(str))
plt.show()
plt.clf()

#Plot slam update frequency
plt.plot(slam_time[1:],update_freq)
plt.xlabel("Time /s")
plt.ylabel("Frequency /Hz")
plt.title("SLAM Update Frequency - Mean Time: " + avg_update_time.astype(str))
plt.show()
plt.clf()

##Plot number of particles
# plt.plot(particles_time,num_particles)
# plt.xlabel("Time /s")
# plt.ylabel("Particles")
# plt.title("Particles Number")
# plt.show()
# plt.clf()

#Plot path
plt.plot(slam_x, slam_y, true_x, true_y, odom_x, odom_y)
plt.legend(["SLAM","TRUE","ODOMETRY"])
plt.xlabel("x displacement")
plt.ylabel("y displacement")
plt.title("Pose")
plt.show()
plt.clf() 