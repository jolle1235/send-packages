from fcu_interface import FcuInterface
import time
import pymap3d
from math import sqrt, cos, sin
import numpy as np
import math
import sys


class FlightCtrl:

    # Flight test stages
    states = ["wait_for_arm", "fly_to_safe_height", "fly_to_circle_start", "circle_slow", "circle_fast","vertical_circle_slow","vertical_circle_fast", "raster", "single", "rtl", "max"]

    def __init__(self, port, baud):
        self.fcu_interface = FcuInterface(port, baud)

        # Start flight controller interface
        self.fcu_interface.run()
        
        # State machine control
        self.state_idx = 0
        self.state = self.states[self.state_idx]

        # Used to store arming position
        self.arm_gps_pos = [0, 0, 0]
        self.pos_init = False

        # Altitude to fly to after arming
        self.safety_height = 20     # [m]

        # Altitude of the horizontal half-circles
        self.circle_height = 50     # [m]
        self.circle_radius = 80     # [m]

        # Speeds of the horizontal half circles, slow and fast
        self.slow_circle_speed = 1.5    # [deg/s]
        self.fast_circle_speed = 4      # [deg/s]

        # Speeds of the vertical half circles, slow and fast
        self.slow_vert_circle_speed = 1.5 # [deg/s]
        self.fast_vert_circle_speed = 2   # [deg/s]

        # Acceptance radius
        self.acceptance_radius = 0.4

        # "Raster" flight, namely horizontal lines at varying altitudes, at different speeds
        self.raster_speeds = [3, 3, 3, 2, 4, 5, 3, 7, 3, 8]     # [m/s]
        self.raster_points = [(80, 0, 40), (80, 0, 50), (0, 0, 50), (0, 0, 60), (80, 0, 60), (0, 0, 60), (0, 0, 50), (80, 0, 50), (80, 0, 40), (0, 0, 40)]
        self.raster_endpoint_wait_time = 3  #[s]
        
        # Single point
        self.single_duration = 60 * 3 # [s]
        self.single_location = (0, 0, 50)

    def run(self):
        circle_angle = 0
        rate = 20
        raster_idx = 1
        raster_target = self.raster_points[0]
        single_start_time = 0
        sleep_cnt = 0

        print("Starting...")
        print("waiting for arm...")

        while True:
            target = None
            yaw = 0
            if self.pos_init:
                cur_pos_enu = pymap3d.geodetic2enu(*self.fcu_interface.cur_gps_pos, *self.arm_gps_pos)
            else:
                cur_pos_enu = [0, 0, 0]

            if self.state == "wait_for_arm":
                if self.fcu_interface.armed:
                    self.state_idx += 1
                    print("Vehicle armed, flying to safety height")

                    self.arm_gps_pos = self.fcu_interface.cur_gps_pos
                    self.pos_init = True

            elif self.state == "fly_to_safe_height":
                target_enu = [0, 0, self.safety_height]
                target = pymap3d.enu2geodetic(*target_enu, *self.arm_gps_pos)
                
                dist = self.calc_dist(cur_pos_enu, target_enu)
                if(dist < self.acceptance_radius):
                    self.state_idx += 1
                    print("Flying to start of circle")

            elif self.state == "fly_to_circle_start":
                target_enu = [self.circle_radius, 0, self.circle_height]
                target = pymap3d.enu2geodetic(*target_enu, *self.arm_gps_pos)
                dist = self.calc_dist(cur_pos_enu, target_enu)
                yaw = math.radians(90)
                if(dist < self.acceptance_radius):
                    self.state_idx += 1
                    #self.state_idx = 7  # TODO REMOVE
                    print("Starting circle_slow")

            elif self.state == "circle_slow":
                if((circle_angle + (self.slow_circle_speed / rate) < 180)):
                    target_enu = [cos(np.radians(circle_angle)) * self.circle_radius, sin(np.radians(circle_angle)) * self.circle_radius, self.circle_height]
                    yaw = math.atan2(cur_pos_enu[0]*-1,cur_pos_enu[1]*-1)
                    target = pymap3d.enu2geodetic(*target_enu, *self.arm_gps_pos)
                    # print("enu target circle slow",target_enu)
                    circle_angle += (self.slow_circle_speed / rate)
                    # print("circle_angle:",circle_angle)
                else:
                    self.state_idx += 1
                    print("Starting circle_fast")

            elif self.state == "circle_fast":
                if((circle_angle - (self.fast_circle_speed / rate)) > 0):
                    target_enu = [cos(np.radians(circle_angle))* self.circle_radius, sin(np.radians(circle_angle))* self.circle_radius, self.circle_height]
                    # print("enu target circle_fast",target_enu)
                    yaw = math.atan2(cur_pos_enu[0]*-1,cur_pos_enu[1]*-1)
                    target = pymap3d.enu2geodetic(*target_enu, *self.arm_gps_pos)
                    circle_angle -= (self.fast_circle_speed / rate)
                else:
                    circle_angle = 30
                    self.state_idx += 1
                    print("Starting vertical_circle_slow")

            elif self.state == "vertical_circle_slow":
                if((circle_angle + (self.slow_vert_circle_speed / rate) < 150)):
                    target_enu = [cos(np.radians(circle_angle)) * self.circle_radius, 0, sin(np.radians(circle_angle)) * self.circle_radius]
                    yaw = math.radians(270) # point towards direction of flight.
                    target = pymap3d.enu2geodetic(*target_enu, *self.arm_gps_pos)
                    # print("vertical_circle_slow",target_enu)
                    circle_angle += (self.slow_vert_circle_speed / rate)
                    # print("circle_angle:",circle_angle)
                else:
                    self.state_idx += 1
                    print("Starting vertical_circle_fast")

            elif self.state == "vertical_circle_fast":
                if((circle_angle - (self.fast_vert_circle_speed / rate)) > 30):
                    target_enu = [cos(np.radians(circle_angle)) * self.circle_radius, 0, sin(np.radians(circle_angle)) * self.circle_radius]
                    # print("vertical_circle_fast",target_enu)
                    yaw = math.radians(90)
                    target = pymap3d.enu2geodetic(*target_enu, *self.arm_gps_pos)
                    circle_angle -= (self.fast_vert_circle_speed / rate)
                    # print("circle_angle:",circle_angle)
                else:
                    self.state_idx += 1
                    print("Starting raster")

            elif self.state == "raster":
                if sleep_cnt > 0:
                    sleep_cnt -= 1
                    #print("Waiting")
                else:
                    target_vec = np.array(self.raster_points[raster_idx]) - np.array(self.raster_points[raster_idx - 1])
                    target_vec = self.normalize(target_vec) * (self.raster_speeds[raster_idx] / rate)
                    target_enu = raster_target + target_vec

                    target = pymap3d.enu2geodetic(*target_enu, *self.arm_gps_pos)

                    raster_target = target_enu

                    if self.calc_dist(target_enu, self.raster_points[raster_idx]) <= (self.raster_speeds[raster_idx] / rate) * 2:
                        raster_idx += 1
                        print("Advancing to raster index " + str(raster_idx))
                        raster_target = self.raster_points[raster_idx - 1]
                        
                        sleep_cnt = self.raster_endpoint_wait_time * rate
                        print("Sleeping")

                        if(raster_idx >= len(self.raster_points)):
                            self.state_idx += 1
                            print("Starting single point")
                            single_start_time = time.time()

            elif self.state == "single":
                if sleep_cnt > 0:
                    sleep_cnt -= 1
                else:
                    target = pymap3d.enu2geodetic(*self.single_location, *self.arm_gps_pos)

                    if (time.time() - single_start_time) > self.single_duration:
                        self.state_idx += 1
                        print("Done, flying back to starting point. Please land manually.")

            elif self.state == "rtl":
                target = pymap3d.enu2geodetic(0, 0, self.safety_height, *self.arm_gps_pos)
                yaw = math.radians(0)

            else:
                print("State exceeded limit, please land!")
                break

            self.state = self.states[self.state_idx]

            if target is not None:
                self.fcu_interface.send_target(*target,yaw)
            time.sleep(1.0/rate)

    # Euclidean distance calculation for enu coordinates
    def calc_dist(self,enu1, enu2):
        return sqrt((enu1[0]-enu2[0])**2 + (enu1[1] - enu2[1])**2 + (enu1[2] - enu2[2])**2)

    def normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0: 
            return v
        return v / norm

if __name__ == '__main__':
    port = "udp:127.0.0.1:14550"
    baud = None

    if len(sys.argv) >= 3:
        port = sys.argv[1]
        baud = sys.argv[2]

    fc = FlightCtrl(port, baud)
    fc.run()