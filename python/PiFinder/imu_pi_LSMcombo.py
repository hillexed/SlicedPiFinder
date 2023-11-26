#!/usr/bin/python
# -*- coding:utf-8 -*-
"""
This module is for IMU related functions, using
the LSM6DSOX + LIS3MDL - Precision 9 DoF IMU at https://www.adafruit.com/product/4517

To use, install libraries:
sudo pip3 install adafruit-circuitpython-lis3mdl adafruit-circuitpython-lsm6ds

Also requires this library:
https://github.com/gamblor21/Gamblor21_CircuitPython_AHRS/

"""
from pprint import pprint
import time
import board
import adafruit_lis3mdl
import adafruit_lsm6ds.lsm6dsox
from PiFinder.gamblor21_ahrs import mahony # 9dof sensor fusion library

from scipy.spatial.transform import Rotation

from PiFinder import config

QUEUE_LEN = 10
MOVE_CHECK_LEN = 2


class Imu:
    def __init__(self):
        i2c = board.I2C()
        self.accelgyro = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(i2c)
        self.magnetometer = adafruit_lis3mdl.LIS3MDL(i2c)
        
        self.imu_sample_period = 1 / 30

        self.filter = mahony.Mahony(Kp=0.05, Ki=0.0, sample_freq=1.0/self.imu_sample_period)

        # used in self.remap_axes()
        self.remap_flat = False
        cfg = config.Config()
        if cfg.get_option("screen_direction") == "flat":
            self.remap_flat = True 
        self.quat_history = [(0, 0, 0, 0)] * QUEUE_LEN
        self.calibration = 0
        self.avg_quat = (0, 0, 0, 0)
        self.__moving = False

        self.last_sample_time = time.time()

        self.num_updates = 0

        # Calibration settings

        self.gyro_xavg = 0
        self.gyro_yavg = 0
        self.gyro_zavg = 0
        self.gyro_samples = 0
        self._last_accel = (0,0,0)

        self.magnetom_min = [-90.07600116924876, -26.074247296112247, -53.785442852966966]
        self.magnetom_max = [-3.8439052908506284, 62.84712072493422, 33.0751242326805]

        # First value is delta to exceed between samples
        # to start moving, second is threshold to fall below
        # to stop moving.
        self.__moving_threshold = (0.001, 0.0005)

    def remap_axes(self, triple):
        x,y,z = triple
        # call this after every sensor reading to remap the axes to what pifinder expects
        # based on imu_pi's remappings
        if self.remap_flat:
            return (y,x,-z) # is this right?
        else:
            return (z,y,x)

    def quat_to_euler(self, quat):
        if quat[0] + quat[1] + quat[2] + quat[3] == 0:
            return 0, 0, 0
        rot = Rotation.from_quat(quat)
        rot_euler = rot.as_euler("xyz", degrees=True)
        # convert from -180/180 to 0/360
        rot_euler[0] += 180
        rot_euler[1] += 180
        rot_euler[2] += 180
        return rot_euler

    def moving(self):
        """
        Compares most recent reading
        with past readings
        """
        return self.__moving

    def take_magnetometer_calibrating_measurement(self):
        # the magnetometer should really be moved in a sphere.
        # the measurements will form a circle on the xy plane, which we then adjust so the center is on (0,0)   
        # this just takes the max and min and then remaps that, so outliers can vastly affect things
        mag_values = self.remap_axes(self.magnetometer.magnetic) # raw values
        for i in range(3):
            if mag_values[i] < self.magnetom_min[i]:
                self.magnetom_min[i] = mag_values[i]
                print("Min: {}, max: {}, ranges: {}".format(self.magnetom_min, self.magnetom_max, [self.magnetom_max[i] - self.magnetom_min[i] for i in range(3)]))
            if mag_values[i] > self.magnetom_max[i]:
                self.magnetom_max[i] = mag_values[i]
                print("Min: {}, max: {}, ranges: {}".format(self.magnetom_min, self.magnetom_max, [self.magnetom_max[i] - self.magnetom_min[i] for i in range(3)]))

    def map_range(self, x, in_min, in_max, out_min, out_max):
        """
        Maps a number from one range to another.
        :return: Returns value mapped to new range
        :rtype: float
        """
        mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        if out_min <= out_max:
            return max(min(mapped, out_max), out_min)
        return min(max(mapped, out_max), out_min)

    def calibrate(self):

        if self.magnetom_min == [0,0,0]:
            mag_values = self.remap_axes(self.magnetometer.magnetic)
            self.magnetom_min = list(mag_values)

        if self.magnetom_max == [0,0,0]:
            mag_values = self.remap_axes(self.magnetometer.magnetic)
            self.magnetom_max = [mag_values[i] + 0.00001 for i in range(3)]

        if self.calibration  == 0:
            # just beginning calibration
            self.calibration = 1

        if self.calibration in (0,1):
        # calibrate gyro using accelerometer.
            current_accel = self.remap_axes(self.accelgyro.acceleration)

            # gyro needs to be calibrated while not moving.
            # use accelerometer to determine when we're not moving
            # take squared magnitude of acceleration difference
            accel_diff = sum([(current_accel[x] - self._last_accel[x])**2 for x in range(3)])
            if accel_diff < self.__moving_threshold[1]:
                # take a gyro reading to calibrate

                gx, gy, gz = self.remap_axes(self.accelgyro.gyro)

                self.gyro_xavg += gx
                self.gyro_yavg += gy
                self.gyro_zavg += gz
                self.gyro_samples += 1

            self._last_accel = current_accel
            
            if self.gyro_samples > 100:
                print(f"Gyro calibrated to {self.gyro_xavg/self.gyro_samples}, {self.gyro_yavg/self.gyro_samples}, {self.gyro_zavg/self.gyro_samples} average")
                # signal done with gyro calibration
                self.calibration = 3

        # meanwhile, rotate this a bunch to calibrate the magnetometer
        self.take_magnetometer_calibrating_measurement()
        

    def update(self):
        # check for update frequency
        if time.time() - self.last_sample_time < self.imu_sample_period:
            return

        self.last_sample_time = time.time()

        self.num_updates += 1

        if self.calibration in (0,1):
            self.calibrate()
            return True

        # compute quat
        ax,ay,az = self.remap_axes(self.accelgyro.acceleration)
        gx, gy, gz = self.remap_axes(self.accelgyro.gyro)
        mx, my, mz = self.remap_axes(self.magnetometer.magnetic)

        # apply gyro calibration
        gx -= self.gyro_xavg/self.gyro_samples
        gy -= self.gyro_yavg/self.gyro_samples
        gz -= self.gyro_zavg/self.gyro_samples

        # apply magnetometer calibration
        mx = self.map_range(mx, self.magnetom_min[0], self.magnetom_max[0], -1, 1)
        my = self.map_range(my, self.magnetom_min[1], self.magnetom_max[1], -1, 1)
        mz = self.map_range(mz, self.magnetom_min[2], self.magnetom_max[2], -1, 1)
        # keep calibrating magnetometer the more it moves
        self.take_magnetometer_calibrating_measurement()


        # update orientation
        self.filter.update(gx, gy, gz, ax, ay, az, mx, my, mz)

        quat = (self.filter.q0, self.filter.q1, self.filter.q2, self.filter.q3)


        if self.num_updates % 30 == 0:
            print(("A",ax,ay,az))
            print(("G",gx,gy,gz))
            print(("M",mx,my,mz))
            print(("Q",quat))

        if quat[0] == None:
            print("IMU: Failed to get sensor values")
            return

        _quat_diff = []
        for i in range(4):
            _quat_diff.append(abs(quat[i] - self.quat_history[-1][i]))

        self.__reading_diff = sum(_quat_diff)   

        self.avg_quat = quat
        if len(self.quat_history) == QUEUE_LEN:
            self.quat_history = self.quat_history[1:]
        self.quat_history.append(quat)

        if self.__moving:
            if self.__reading_diff < self.__moving_threshold[1]:
                self.__moving = False
        else:
            if self.__reading_diff > self.__moving_threshold[0]:
                self.__moving = True

    def get_euler(self):
        return list(self.quat_to_euler(self.avg_quat))


def imu_monitor(shared_state, console_queue):
    imu = Imu()
    imu_calibrated = False
    imu_data = {
        "moving": False,
        "move_start": None,
        "move_end": None,
        "pos": [0, 0, 0],
        "start_pos": [0, 0, 0],
        "status": 0,
    }
    while True:
        imu.update()
        imu_data["status"] = imu.calibration
        if imu.moving():
            if imu_data["moving"] == False:
                # print("IMU: move start")
                imu_data["moving"] = True
                imu_data["start_pos"] = imu_data["pos"]
                imu_data["move_start"] = time.time()
            imu_data["pos"] = imu.get_euler()
        else:
            if imu_data["moving"] == True:
                # If wer were moving and we now stopped
                # print("IMU: move end")
                imu_data["moving"] = False
                imu_data["pos"] = imu.get_euler()
                imu_data["move_end"] = time.time()

        if imu_calibrated == False: # todo?
            if imu_data["status"] == 3:
                imu_calibrated = True
                console_queue.put("IMU: NDOF Calibrated!")
                console_queue.put(f"IMU: Mag min: {imu.magnetom_min}")
                console_queue.put(f"IMU: Mag max: {imu.magnetom_max}")

        if shared_state != None and imu_calibrated:
            shared_state.set_imu(imu_data)
