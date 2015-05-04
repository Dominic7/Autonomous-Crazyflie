'''
Sensors
'''

import time
from threading import Thread, Timer

#Has to be launched from within the autonomous folder

import cflib
from cflib.crazyflie import Crazyflie

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig


import logging
logging.basicConfig(level=logging.ERROR)

import crazy_auto1
from pykalman import KalmanFilter
import numpy as np


class logs:

    def __init__(self, cf):

        #self.log_file_next_stuff = open("log_file_next.txt", "w+")
        #self.log_file_stab = open("log_file_stab.txt", "w+")

        #local copy of crazy_Auto
        self._cf = cf

        # Roll, Pitch, Yaw
        self.init_state = [0,0,0]
        self.state = [0,0,0]
        self.next_state = [0,0]
        # X, Y, Z, Mag
        self.init_accel_state = [0,0,0,0]
        self.accel_state = [0,0,0,0]

        # X, Y, Z 
        self.init_gyro_state = [0,0,0]
        self.gyro_state = [0,0,0]

        # ground average, current
        self.altitude = [0,0]

        #current battery voltage
        self.battery_state = 0
        
        #[roll,pitch,yaw,altitude,acc.x,acc.y,acc.z,acc.mag,gyro.x, gyro.y, gyro.z, batteryV]
        self.measurements = [0 for i in range(12)]

        self.logging_count = 0

        #Setup logs once connection to crazyflie has been established
        self._cf._cf.connected.add_callback(self._init_flight_var)

        # specifying the size of the state and the observation space
        #self.transition_matrices = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        #self.transition_matrices.resize(4,1)
        #self.kf = KalmanFilter(transition_matrices=self.transition_matrices, n_dim_obs= 4) #transition_matrices=self.transition_matrices,
        

        #self._cf.connected.add_callback(self._init_flight_var) #fill in base parameters for flight
        


        
    def _init_flight_var(self, link_uri):

        print "Connected to %s" % link_uri

        self.RPY_log = LogConfig(name="Stabilizer", period_in_ms=10)

        self.RPY_log.add_variable("stabilizer.roll", "float")
        self.RPY_log.add_variable("stabilizer.pitch", "float")
        self.RPY_log.add_variable("stabilizer.yaw", "int16_t")
        
        self.RPY_log.add_variable("baro.asl", "float") #barometer above sea level 

        self.RPY_log.add_variable("gyro.x", "float")
        self.RPY_log.add_variable("gyro.y", "float")
        self.RPY_log.add_variable("gyro.z", "float")
        self.battery_log = LogConfig(name="Battery", period_in_ms=1000)
        self.battery_log.add_variable("pm.vbat", "float")

        self.acc_log = LogConfig(name="Acc", period_in_ms = 10)
        self.acc_log.add_variable("acc.x", "float")
        self.acc_log.add_variable("acc.y", "float")
        self.acc_log.add_variable("acc.z", "float")
               
        self._cf._cf.log.add_config(self.RPY_log) #add the log to the CrazyFlie 
        self._cf._cf.log.add_config(self.battery_log)
        self._cf._cf.log.add_config(self.acc_log)

        self.RPY_log.data_received_cb.add_callback(self.update_flight_params)
        self.RPY_log.error_cb.add_callback(self.update_error)
        self.battery_log.data_received_cb.add_callback(self.update_battery)
        self.battery_log.error_cb.add_callback(self.update_error)

        self.acc_log.data_received_cb.add_callback(self.update_acc)
        self.acc_log.error_cb.add_callback(self.update_error)


        self.RPY_log.start()
        self.battery_log.start()
        self.acc_log.start()
        print "Logging Started\n"
        
        self._cf._cf.connected.add_callback(self._cf._connected)

    def update_error(self, logconf, msg):
        print "Error when logging %s: %s" % (logconf.name, msg)

    def update_battery(self, timestamp, data, logconf):
        self.battery_state = data["pm.vbat"]
    def update_acc(self, timestamp, data, logconf):
        if not self._cf.is_flying:
            self.init_accel_state[0] = data["acc.x"]
            self.init_accel_state[1] = data["acc.y"]
            self.init_accel_state[2] = data["acc.z"]
            self.init_accel_state[3] = data["acc.mag2"]
            return
        self.accel_state[0] = data["acc.x"]
        self.accel_state[1] = data["acc.y"]
        self.accel_state[2] = data["acc.z"]
        self.accel_state[3] = data["acc.mag2"]

    def update_flight_params(self, timestamp, data, logconf):
        #print data
        #print self._cf.is_flying
        if not self._cf.is_flying:
            self.init_state[0] = float(data["stabilizer.roll"])
            self.init_state[1] = float(data["stabilizer.pitch"])
            self.init_state[2] = data["stabilizer.yaw"]

            #self.init_accel_state[0] = data["acc.x"]
            #self.init_accel_state[1] = data["acc.y"]
            #self.init_accel_state[2] = data["acc.z"]
            #self.init_accel_state[3] = data["acc.mag2"]

            self.init_gyro_state[0] = data["gyro.x"]
            self.init_gyro_state[1] = data["gyro.y"]
            self.init_gyro_state[2] = data["gyro.z"]
            
            #self.battery_state =  data["pm.vbat"]
           
            #if self.logging_count is 0:
            self.altitude[0] = data["baro.asl"]
            #    self.logging_count += 1
            #else:
                
            #    self.altitude[0] = float((data["baro.asl"] + self.logging_count*self.altitude[0])/(self.logging_count+1)) #take the running average of the inital altitude for better ground reading
            #    self.logging_count += 1

            #self._cf.altitude[1] = self.altitude[0] #<-----------this shouldnt be here....
            
            #[roll,pitch,yaw,altitude,acc.x,acc.y,acc.z,acc.mag,gyro.x, gyro.y, gyro.z, batteryV]

            #print self.altitude
            return

        self.state[0] = data["stabilizer.roll"]
        self.state[1] = data["stabilizer.pitch"]
        self.state[2] = data["stabilizer.yaw"]

        #self.accel_state[0] = data["acc.x"]
        #self.accel_state[1] = data["acc.y"]
        #self.accel_state[2] = data["acc.z"]
        #self.accel_state[3] = data["acc.mag2"]

        self.gyro_state[0] = data["gyro.x"]
        self.gyro_state[1] = data["gyro.y"]
        self.gyro_state[2] = data["gyro.z"]

        #self.battery_state =  data["pm.vbat"]
        
        self.altitude[1] = data["baro.asl"]
        #print self.altitude
        self.next_state[0] = self.state[0] - self.init_state[0]
        self.next_state[1] = self.state[1] - self.init_state[0]



    def log_file_print(self, file, data):
        for i in range(len(data)):
            file.write(str(data[i]))
            file.write(',')
        file.write('\n')   
    
    

    def get_measurements(self):
        pass
        #return self.measurements
        #self.kf = self.kf.em(self.measurements, n_iter=1)
        #return self.kf.filter(self.measurements)

    def get_gyro(self, value):

        if (value == 0):
            print "X value:  ", self.gyro_state[0]

        elif(value == 1):
            print "Y value:  ", self.gyro_state[1]

        elif(value == 2):
            print "Z value:  ", self.gyro_state[2]
        else:
            return 0

        
    def get_altitude(self):
        #print "altitude:  ", self.baro_sensor[0]
        return self.altitude[0]


    def get_roll(self):
        print "roll:  ", self.state[0]

    def get_pitch(self):
        print "pitch:  ", self.state[1]

    def get_yaw(self):
        print "yaw:  ", self.state[2]


    def get_baro(self, value):

        if (value == 0):
            print "lRaw:  ", self.baro_sensor[0]

        elif(value == 1):
            print "lLong:  ", self.baro_sensor[0]

        elif(value == 2):
            print "Pressure:  ", self.baro_sensor[0]
        else:
            return 0

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print "Scanning interfaces for Crazyflies..."
    available = cflib.crtp.scan_interfaces()
    print "Crazyflies found:"
    for i in available:
        print i[0]

    if len(available) > 0:
        le = crazy_auto1.Crazy_Auto(available[0][0])
        #t = logs(le)
        while le.is_connected:
            time.sleep(1)
        # t.log_file_stab.close()
        # t.log_file_next_stuff.close()
        # print "Roll, Pitch, Yaw",t.state
        # print "Gyro",t.gyro_state
        # t.get_gyro(1)

    else:
        print "No Crazyflies found, cannot run example"
