from thermal_t import thermal_t
import lcm
import time
import sys
import os
from threading import Thread, Lock, current_thread
import numpy as np
import matplotlib.pyplot as plt

cur_therm_values = None

def encoder_message_handler(channel, data):
    global cur_therm_values
    cur_therm_values = thermal_t.decode(data)
    
def handle_lcm(lcm_obj):
    try:
        while True:
            lcm_obj.handle()
    except KeyboardInterrupt:
        print("lcm exit!")
        sys.exit()

def main():
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    lcm_kill_thread = Thread(target = handle_lcm, args= (lc, ), daemon = True)
    lcm_kill_thread.start()
    subscription = lc.subscribe("THERMAL", encoder_message_handler)

    mlx_shape = (24,32)

    # setup the figure for plotting
    plt.ion() # enables interactive plotting
    fig,ax = plt.subplots(figsize=(12,7))
    therm1 = ax.imshow(np.zeros(mlx_shape),vmin=0,vmax=60) #start plot with zeros
    cbar = fig.colorbar(therm1) # setup colorbar for temps
    cbar.set_label('Temperature [$^{\circ}$C]',fontsize=14) # colorbar label

    time.sleep(1.5)
    while(True):
        t1 = time.monotonic()
        try:
            data_array = np.array(cur_therm_values.data) # convert to np.array
            therm1.set_data(np.fliplr(data_array)) # flip left to right
            therm1.set_clim(vmin=np.min(data_array),vmax=np.max(data_array)) # set bounds
            cbar.on_mappable_changed(therm1) # update colorbar range
            plt.pause(0.001) # required
            fig.savefig('mlx90640_test_fliplr.png',dpi=300,facecolor='#FCFCFC',
                        bbox_inches='tight') # comment out to speed up
        except ValueError:
            continue # if error, just read again

if __name__== "__main__":
    main()