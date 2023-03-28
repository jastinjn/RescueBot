

import time,board,busio
import numpy as np
import adafruit_mlx90640
from thermal_t import thermal_t
import lcm

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

time.sleep(1.0)

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

mlx = adafruit_mlx90640.MLX90640(i2c)
print("MLX addr detected on I2C")
print([hex(i) for i in mlx.serial_number])

mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ

frame = [0] * 768

while True:
    # stamp = time.monotonic()
    try:
        mlx.getFrame(frame)
    except ValueError:
        # these happen, no biggie - retry
        continue
    curr_thermal = thermal_t()
    sum = 0
    for h in range(24):
        for w in range(32):
            curr_thermal.data[w][h] = frame[32 * (23-h) + w]
            sum += curr_thermal.data[w][h]

    lc.publish("THERMAL", curr_thermal.encode())
    print("Thermal published")


            