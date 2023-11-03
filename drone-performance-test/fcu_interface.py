from pymavlink import mavutil
import threading
import csv
import os
import time
import queue
import datetime
from pymavlink.dialects.v20 import common as mavlink2
import math
import time

class FcuInterface:
    def __init__(self,conn_string,baud):
        # Msg queues
        self.global_pos_int_queue = queue.Queue()
        self.gps_raw_int_queue = queue.Queue()
        self.attitude_queue = queue.Queue()
        self.target_queue = queue.Queue()
        self.cur_gps_pos = [0, 0, 0]

        self.raw_gps_fieldnames = ['timestamp_ns','time_usec', 'fix_type', 'lat', 'lon', 'alt', 'eph', 'epv', 'vel', 'cog', 'satellites_visible', 'alt_ellipsoid', 'h_acc', 'v_acc', 'vel_acc', 'hdg_acc', 'yaw']
        self.gps_int_fieldnames = ['timestamp_ns','time_boot_ns', 'lat', 'lon', 'alt', 'relative_alt', 'vx', 'vy', 'vz', 'hdg']
        self.attitude_fieldnames = ['timestamp_ns','time_boot_ns', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed']
        self.target_fieldnames = ['timestamp_ns','target_latitude', 'target_longitude', 'target_altitude', 'yaw']

        if baud:
            print("Connecting to " + conn_string + " with baud " + str(baud))
            self.mavlink2_conn = mavutil.mavlink_connection(conn_string, str(baud))
        else:
            print("Conencting to " + conn_string)
            self.mavlink2_conn = mavutil.mavlink_connection(conn_string)

        print("Connection initialized")

        # Wait for the first heartbeat 
        self.mavlink2_conn.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.mavlink2_conn.target_system, self.mavlink2_conn.target_component))
        self.armed = 0
        self.disarmed = 0
        
    # Request specified mavlink message at certain rate
    def request_msg_at_rate(self, msg_id, stream_interval_us):
        # param1: message to stream
        # param2: 1000000 (Stream interval in microseconds,1 sec)
        msg = self.mavlink2_conn.mav.command_long_encode(
                self.mavlink2_conn.target_system,  # Target system ID
                self.mavlink2_conn.target_component,  # Target component ID
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
                0,  # Confirmation
                msg_id,  # param1: Message ID to be streamed
                stream_interval_us, # param2: Interval in microseconds
                0,       
                0,      
                0,       
                0,       
                0       
                )

        # Send the COMMAND_LONG     
        self.mavlink2_conn.mav.send(msg)

        # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
        response = self.mavlink2_conn.recv_match(type='COMMAND_ACK', blocking=True)
        if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Command SET_MESSAGE_INTERVAL accepted")
        else:
            print("Command SET_MESSAGE_INTERVAL failed")

    # Send set_position_target message
    def send_target(self, target_latitude, target_longitude, target_altitude, yaw, vx = 0,vy = 0,vz = 0):
                                #adding target data as tuple
        self.target_queue.put((time.time_ns(),target_latitude,target_longitude,target_altitude,yaw))
        # Create a SET_POSITION_TARGET_GLOBAL_INT message
        mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE | 
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)  # Type mask (position and velocity)
        if vx is None:
            mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        if vy is None:
            mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        if vz is None:
            mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        msg = self.mavlink2_conn.mav.set_position_target_global_int_encode(
            0,#time_boot_ms
            self.mavlink2_conn.target_system,  # Target system ID
            self.mavlink2_conn.target_component, # Component ID
            5,  # Global int frame
            mask,   # Ignore mask
            int(target_latitude * 1e7),  # Target latitude in degrees * 1e7
            int(target_longitude * 1e7),  # Target longitude in degrees * 1e7
            target_altitude,  # Target altitude in m
            vx,           # X (North) velocity in cm/s 
            vy,           # Y (East) velocity in cm/s 
            vz,           # Z (Down) velocity in cm/s 
            0,           # X (North) acceleration in m/s^2 (not used in this case)
            0,           # Y (East) acceleration in m/s^2 (not used in this case)
            0,           # Z (Down) acceleration in m/s^2 (not used in this case)
            yaw,           # Yaw angle in radians 
            0            # Yaw rate in rad/s (not used in this case)
        )
        self.mavlink2_conn.mav.send(msg)

    def log_data(self):
        while True:
            try:
                if(self.gps_raw_int_queue.qsize() > 0):
                    gps_raw_int = self.gps_raw_int_queue.get_nowait()
                    
                    row = {
                        'timestamp_ns':time.time_ns(),'time_usec': gps_raw_int.time_usec * 1e3, 'fix_type': gps_raw_int.fix_type, 'lat': gps_raw_int.lat / 1e7,
                        'lon': gps_raw_int.lon / 1e7, 'alt': gps_raw_int.alt / 1e3, 'eph': gps_raw_int.eph, 'epv': gps_raw_int.epv,
                        'vel': gps_raw_int.vel / 1e2, 'cog': gps_raw_int.cog, 'satellites_visible': gps_raw_int.satellites_visible,
                        'alt_ellipsoid': gps_raw_int.alt_ellipsoid / 1e3, 'h_acc': gps_raw_int.h_acc / 1e3, 'v_acc': gps_raw_int.v_acc / 1e3,
                        'vel_acc': gps_raw_int.vel_acc / 1e3, 'hdg_acc': gps_raw_int.hdg_acc / 1e5, 'yaw': gps_raw_int.yaw
                    }
                    self.raw_gps_writer.writerow(row)

            except Exception as e:
                print(e)

            try:
                if(self.global_pos_int_queue.qsize() > 0):
                    global_pos_int = self.global_pos_int_queue.get_nowait()
                    
                    row = {
                        'timestamp_ns':time.time_ns(),'time_boot_ns': global_pos_int.time_boot_ms * 1e6, 'lat': global_pos_int.lat / 1e7, 'lon': global_pos_int.lon / 1e7,
                        'alt': global_pos_int.alt / 1e3, 'relative_alt': global_pos_int.relative_alt / 1e3, 'vx': global_pos_int.vx / 1e2,
                        'vy': global_pos_int.vy / 1e2, 'vz': global_pos_int.vz / 1e2, 'hdg': global_pos_int.hdg
                    }
                    self.gps_int_writer.writerow(row)

            except Exception as e:
                print(e)

            try:
                if(self.attitude_queue.qsize() > 0):
                    attitude = self.attitude_queue.get_nowait()
                    
                    row = {
                        'timestamp_ns':time.time_ns(),'time_boot_ns': attitude.time_boot_ms * 1e6, 'roll': math.degrees(attitude.roll), 'pitch': math.degrees(attitude.pitch),
                        'yaw': math.degrees(attitude.yaw), 'rollspeed': math.degrees(attitude.rollspeed), 'pitchspeed': math.degrees(attitude.pitchspeed),
                        'yawspeed': math.degrees(attitude.yawspeed)
                    }
                    self.attitude_writer.writerow(row)

            except Exception as e:
                print(e)

            try:
                if(self.target_queue.qsize() > 0):
                    target_tuple = self.target_queue.get_nowait()
                    row = {
                        'timestamp_ns': target_tuple[0],'target_latitude': target_tuple[1], 'target_longitude': target_tuple[2], 'target_altitude': target_tuple[3],
                        'yaw': math.degrees(target_tuple[4])
                    }
                    self.target_writer.writerow(row)

            except Exception as e:
                print(e)
            
            if(self.gps_raw_int_queue.qsize() == 0 and self.global_pos_int_queue.qsize() == 0 and self.attitude_queue.qsize() == 0 and self.target_queue.qsize() == 0 and self.disarmed):
                print('killing the log_data thread',self.disarmed)
                print("closing csv")
                self.attitude_csv.close()
                self.gps_int_csv.close()
                self.raw_gps_csv.close()
                self.target_csv.close()
                break

    def init_csvs(self, data_path):
        now = datetime.datetime.now()
        formatted_date = now.strftime("%Y-%m-%d-%H-%M-%S")

        if not os.path.exists(data_path):
            os.mkdir(data_path)
        
        self.gps_int_csv = open(data_path + "/" + formatted_date + '_' + 'gps_int.csv', 'w')
        self.gps_int_writer = csv.DictWriter(self.gps_int_csv, fieldnames=self.gps_int_fieldnames)
        self.gps_int_writer.writeheader()

        self.raw_gps_csv = open(data_path + "/" + formatted_date + '_' + 'gps_raw_int.csv', 'w')
        self.raw_gps_writer = csv.DictWriter(self.raw_gps_csv, fieldnames=self.raw_gps_fieldnames)
        self.raw_gps_writer.writeheader()

        self.attitude_csv = open(data_path + "/" + formatted_date + '_' + 'attitude.csv', 'w')
        self.attitude_writer = csv.DictWriter(self.attitude_csv, fieldnames=self.attitude_fieldnames)
        self.attitude_writer.writeheader()

        self.target_csv = open(data_path + "/" + formatted_date + '_' + 'target.csv', 'w')
        self.target_writer = csv.DictWriter(self.target_csv, fieldnames=self.target_fieldnames)
        self.target_writer.writeheader()

    def msg_recv_func(self):
        previous_armed = False
        while True:
            msg = self.mavlink2_conn.recv_match()
            if msg is not None:
                if msg.get_type() == 'HEARTBEAT':
                    # Check armed status
                    if msg.type in [1, 2, 3, 4, 13, 14, 15, 43]:
                        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY
                        if armed != previous_armed:
                            if not armed:
                                # Drone was armed before and is now disarmed
                                print("Drone is disarmed")
                                self.disarmed = True
                            else:
                                # Drone was disarmed before and is now armed
                                print("Drone is armed")
                                self.init_csvs("data")
                                self.armed = True
                                self.disarmed = False
                                data_consumer_thread = threading.Thread(target=self.log_data)
                                data_consumer_thread.start()
                                
                            # Update the previous_armed status
                            previous_armed = armed
                if msg.get_type() == 'ATTITUDE':
                    if(self.armed):
                        self.attitude_queue.put(msg)
                        
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    self.cur_gps_pos = [msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1e3]
                    if(self.armed):
                        self.global_pos_int_queue.put(msg)

                if msg.get_type() == 'GPS_RAW_INT':
                    if(self.armed):
                        self.gps_raw_int_queue.put(msg)

    def run(self):
        self.request_msg_at_rate(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 50000)
        self.request_msg_at_rate(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 100000)
        self.request_msg_at_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000)
        msg_recv_thread = threading.Thread(target=self.msg_recv_func)
        msg_recv_thread.start()