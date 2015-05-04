
"""
Autonomous flight library for Crazyflie2
"""

import sys
sys.path.append("../lib")

import time

from threading import Thread, Timer

#Has to be launched from within the autonomous folder

import cflib
from cflib.crazyflie import Crazyflie

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig

import numpy as np

import logging
logging.basicConfig(level=logging.ERROR)

import Sensors



class Crazy_Auto:
    """ Basic calls and functions to enable autonomous flight """  
    def __init__(self, link_uri):
        """ Initialize crazyflie using passed in link"""
        
        
        #trim variables that will be updated during flight for stabilization
        self.roll_trim = 2
        self.pitch_trim = -1

        #-------------- PID stuff -------------------#

        self.avg_state = [0,0,0]
        self.last_state = [0,0,0]

        self.num_readings = 0
        self.state_sum = [0,0,0]

        self.kproportional = .1

        #----------------- END PID --------------------#

        #----------------- Flight Vars ----------------#
        self.next_roll = 0
        self.next_pitch = 0
        self.next_thrust = 0

        self.gyro_threash = 15

        self.take_off_thrust_step = 500

        self.kill_threash = 10000
        self.take_off_thrust = 38000

        self.hover_alt_step = 3
        self.thrust_step = 500

        self.hover_thrust_step = 250

        self.is_hovering = False

        self.min_thrust = 0
        self.is_flying = False

        #---------------- End Flight Vars --------------#

        #---------------- Copter State Vars ----------------#

        #self.altitude = [0,0]  #[base/initial, current]
        #self.current_altitude = 0
        
        self.curr_thrust = 10000 #10000 is the base thrust to get the props moving
        
        self.max_thrust = 60000 #max thrust we ever want to give it
        
        
        self._cf = Crazyflie() #class declaration is in ..\lib\cflib\crazyflie\__init__.py
        self.t = Sensors.logs(self)

        #the three function calls below setup threads for connection monitoring
        self._cf.disconnected.add_callback(self._disconnected) #first monitor thread checking for disconnections
        self._cf.connection_failed.add_callback(self._connection_failed) #second monitor thread for checking for back connection to crazyflie
        self._cf.connection_lost.add_callback(self._connection_lost) # third monitor thread checking for lost connection

        print "Connecting to %s" % link_uri

        self._cf.open_link(link_uri) #connects to crazyflie and downloads TOC/Params
        
        self.is_connected = True

### ---------------------------------------------------------- Main Flight States ---------------------------------------------------------------###
    def hover_mode(self):
        self.is_hovering = True
        print "hovering"
        #t = Timer(10, self._land_copter())
        #t.start()
        yawrate = 0
        sleep_step = .1

        
        while self.is_hovering:
            #print self.t.accel_state
            if self.t.gyro_state[2] >= self.gyro_threash:
                self.next_thrust += self.hover_thrust_step
            elif self.t.gyro_state[2] <= -self.gyro_threash:
                self.next_thrust -= self.hover_thrust_step*2
            print self.next_thrust
            #self.next_thrust = self.curr_thrust + (self.hover_alt - self.altitude[1])*self.thrust_step
            #print self.t.gyro_state
            self._cf.commander.send_setpoint(self.next_roll + self.roll_trim, self.next_pitch+self.pitch_trim, yawrate, self.next_thrust)
            time.sleep(sleep_step)
            
        


    
    def _take_off(self):
                
        
        #self.altitude[0] = self.t.get_altitude()
        self.hover_alt = self.t.altitude[0] + self.hover_alt_step
        
        while self.hover_alt == self.hover_alt_step or self.t.altitude[0] == 0:
            #print self.altitude
            #self.altitude[0] = self.t.get_altitude()
            self.hover_alt = self.t.altitude[0] + self.hover_alt_step
        time.sleep(5)
        self.is_flying = True
        
        yawrate = 0
        sleep_step = .1
        co = 0 
        #print self.t.altitude, self.hover_alt
        print "taking off"
        while self.t.altitude[1] <= self.hover_alt or self.curr_thrust < self.take_off_thrust:
            if self.t.altitude[1] < self.t.altitude[0]:
                self.t.altitude[0] = self.t.altitude[1]
                self.hover_alt = self.t.altitude[0] + self.hover_alt_step
            #print self.t.altitude, self.hover_alt
            print "roll: ",self.next_roll, "pitch: ",self.next_pitch, "gyro: ",self.t.gyro_state
            #print self.altitude, self.hover_alt
            self._cf.commander.send_setpoint(self.next_roll+self.roll_trim, self.next_pitch + self.pitch_trim, yawrate,self.curr_thrust)
            self.curr_thrust += self.take_off_thrust_step
            #print self.curr_thrust
            #print self.altitude[1], self.hover_alt
            time.sleep(sleep_step)
            co += 1
        
        #print self.altitude
        self.hover_alt = self.t.altitude[1]
        self.next_thrust = self.curr_thrust - self.take_off_thrust_step*2
        Thread(target=self.fly_circle(20)).start()
        #Thread(target=self._land_copter).start()

    def _land_copter(self):
        self.is_hovering = False
        print "landing"
        landing_thrust = self.curr_thrust
        while self.altitude[1]  <= self.altitude[0]:# or abs(self.accel_state[2]) > 0:
            landing_thrust -= self.thrust_step
            self._cf.commander.send_setpoint(self.next_roll, self.next_pitch, 0, landing_thrust)

        self.is_flying = False
        time.sleep(.1)
        self._cf.close_link()
        
### --------------------------------------------------------------------------- END MAIN FLIGHT STATES ----------------------------------------------------###




### ---------------------------------------------------------------------- Flight Fuctions ---------------------------------------------------------------###
    def fly_circle(self, max_angle):
        yaw = 0
        count = 0
        print "start circle"
        increment = (max_angle/2)*.1
        self.is_hovering = True
        if self.is_hovering:
            # 3 degrees per time slic
            roll_circle = max_angle
            pitch_circle = 0
            while (roll_circle > 0 and pitch_circle < max_angle):
                self._cf.commander.send_setpoint( roll_circle,+pitch_circle, yaw, self.next_thrust)
                print '1'
                roll_circle -= increment
                pitch_circle += increment
                time.sleep(.1)
             

            roll_circle = 0
            pitch_circle = max_angle
            while (roll_circle > -(max_angle) and pitch_circle > 0):
                self._cf.commander.send_setpoint(roll_circle, pitch_circle, yaw, self.next_thrust)
                print '2'
                
                roll_circle -= increment
                pitch_circle -= increment
                time.sleep(.1)

            roll_circle = -max_angle
            pitch_circle = 0
            while (roll_circle < 0 and pitch_circle > -(max_angle)):
                self._cf.commander.send_setpoint(roll_circle,pitch_circle, yaw, self.next_thrust)
                print '3'
                roll_circle += increment
                pitch_circle -= increment
                time.sleep(.1)


            roll_circle = 0
            pitch_circle = -max_angle
            while (roll_circle < (max_angle) and pitch_circle < 0):
                self._cf.commander.send_setpoint(roll_circle,pitch_circle, yaw, self.next_thrust)
                print '4'
                roll_circle += increment
                pitch_circle += increment
                time.sleep(.1)

    def _ramp_motors(self):
        print "Starting Flight"
        print "Start Roll, Pitch", self.init_state
        timer_start = False
        self.is_flying = True
        thrust_mult = 1
        thrust_step = 500
        thrust = 20000
        pitch = 0
        roll = 0
        yawrate = 0
        sleep_step = .1
        while thrust >= 20000:
            self._cf.commander.send_setpoint(self.next_roll, self.next_pitch, yawrate, thrust)
            time.sleep(sleep_step)
            if thrust >= 45000:
                if timer_start is False:
                    t = Timer(10, self._cf.close_link)
                    t.start()
                    timer_start = True
            else:
                thrust += thrust_step * thrust_mult
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()
   
    def kill_check(self): #
        if self.t.accel_state[3] >= self.kill_threash:
            self._cf.commander.send_setpoint(0,0,0,0);
            print "QUADCOPTER KILLED --- Acceleration Mag: ", self.t.accel_state
            self.is_hovering = False
            self.is_flying = False
            self._cf.close_link()
        else:
            Timer(.5, self.kill_check).start()

###------------------------------------------------------------------------------ END Flight Functions -------------------------------------------------------###


    def update_error(self, logconf, msg):
        print "Error when logging %s: %s" % (logconf.name, msg)

    def update_vals(self):
        #m = self.t.get_measurements() #  m = [roll,pitch,yaw,altitude,acc.x,acc.y,acc.z,acc.mag,gyro.x, gyro.y, gyro.z, batteryV]
        
        #print m
        pre_roll = self.t.next_state[0]#float(m[0])
        pre_pitch = self.t.next_state[1]#float(m[1])
        self.state_PID( pre_roll, pre_pitch)

        Timer(.1, self.update_vals).start()
       
    def state_PID(self, p_r, p_p):
        error = [p_r,p_p, float(self.next_thrust)]
        
        kp = self.kproportional#[self.avg_state[i] - error[i] for i in range(len(error))]
        
        #self.state_sum = [self.state_sum[i] + error[i] for i in range(len(error))]
        
        p = [error[i]*kp for i in range(len(error))]
        
        #ii = [self.state_sum[i]*kp for i in range(len(self.state_sum))]
        
        #d = [float((error[i]-self.last_state[i])/.1) for i in range(len(error))]
        
        #self.pid_file.write(str(p[0])+','+str(p[1])+','+str(p[2]))
        
        #self.pid_file.write(str(ii)+' ')    
        #self.pid_file.write(str(d))
        #self.pid_file.write('\n')
        #if (self.t.gyro_state
        self.next_roll = p[0] #+ ii[0] + d[0]
        self.next_pitch = p[1]# + ii[1] + d[1]
        #self.next_thrust = p[2]# + ii[2] + d[2]


### ------------------------------------------------------- Callbacks ----------------------------------------------------------------------###
    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self.update_vals).start()
        #Thread(target=self.kill_check).start()
        print "Waiting for logs to initalize..."
        Thread(target=self._take_off).start()


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print "Connection to %s lost: %s" % (link_uri, msg)
        self.is_connected = False

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""

        print "Disconnected from %s" % link_uri
        self.is_connected = False
### ------------------------------------------------------ END CALLBACKS -------------------------------------------------------------------###












