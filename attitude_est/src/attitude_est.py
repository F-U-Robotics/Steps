#!/usr/bin/python

"""
This node will return the pitch and roll angles of the robot.
It assumes a MPU6050 node  has been installed
"""
import rospy
from geometry_msgs.msg import Vector3 #massage to export the attitude estimation
from sensor_msgs.msg import Imu #masssage to inport IMU data 
import time
import math



class Attitude():
    def __init__(self):
        
        #Pitch and roll estimations
        self.Roll       = 0.0
        self.Pitch      = 0.0
        self.Acc_Roll   = 0.0
        self.Acc_Pitch  = 0.0
        self.Gyro_Roll   = 0.0
        self.Gyro_Pitch  = 0.0
        self.Gyro_Yaw    = 0.0
        #Gyro bias
        self.Gyro_X_b   = 0.0
        self.Gyro_Y_b   = 0.0
        self.Gyro_Z_b   = 0.0
        #Gyro values
        self.Gyro_X     = 0.0
        self.Gyro_Y     = 0.0
        self.Gyro_Z     = 0.0
        #Accel values
        self.Acc_X      = 0.0
        self.Acc_Y      = 0.0
        self.Acc_Z      = 0.0
       
        
        # Calibrating the gyro sensors
       
   



class Attitude_Est():
    def __init__(self):
        self.alpha=0.9
        rospy.loginfo("Setting Up the Node...")

        rospy.init_node('att_Est')

        self.Attitude = Attitude()
        rospy.loginfo("> Attitude class  initialized")

        self._Att_msg  = Vector3()

        #--- Create the Attitude  publisher
        self.ros_pub_att    = rospy.Publisher("/attitude", Vector3, queue_size=1)
        rospy.loginfo("> Publisher corrrectly initialized")

        #--- Create the Subscriber to IMU 
        self.ros_sub_IMU          = rospy.Subscriber("/imu", Imu, self.get_IMU_Data)
        rospy.loginfo("> Subscriber corrrectly initialized")

        #--- Get the last time e got a commands
        self._last_time_IMU_rcv     = time.time()
        self._timeout_s             = 5

        rospy.loginfo("Initialization complete")

        #rospy.loginfo("Clibrating the IMU")
        #N=200
        #for i in range(N):
        #    self.get_IMU_Data(message)
        #    self.Gyro_X_b +=self.Gyro_X
        #    self.Gyro_Y_b +=self.Gyro_Y
        #    self.Gyro_Z_b +=self.Gyro_Z
        #self.Gyro_X_b /=N
        #self.Gyro_Y_b /=N
        #self.Gyro_Z_b /=N
        

    def get_IMU_Data(self, message):
        """
        Get a message from IMU, assuming a maximum input of 1
        """
        #-- Save the time
        self._last_time_IMU_rcv = time.time()
        #print(self.Attitude.__dict__)
        #time.sleep(10)

        #-- Convert vel into servo values
        self.Attitude.Acc_X=message.linear_acceleration.x
        self.Attitude.Acc_Y=message.linear_acceleration.y
        self.Attitude.Acc_Z=message.linear_acceleration.z
        self.Attitude.Gyro_X=message.angular_velocity.x
        self.Attitude.Gyro_Y=message.angular_velocity.y
        self.Attitude.Gyro_Z=message.angular_velocity.z
        rospy.loginfo("Got a reading  a_x = %2.1f  a_y = %2.1f"%(message.linear_acceleration.x, message.linear_acceleration.y))
        self.Comp_Att_Est()

        
    def Comp_Att_Est(self):
        
        dt=time.time()-self._last_time_IMU_rcv

        #Acceleration based estimation
        self.Attitude.Acc_Pitch = math.atan2(self.Attitude.Acc_X, self.Attitude.Acc_Z)
        self.Attitude.Acc_Roll = math.atan2(self.Attitude.Acc_Y, self.Attitude.Acc_Z)

        #Gyro based estimation
        self.Attitude.Gyro_Roll  +=math.radians( (self.Attitude.Gyro_Y-self.Attitude.Gyro_Y_b) * dt)
        self.Attitude.Gyro_Pitch -=math.radians( (self.Attitude.Gyro_X-self.Attitude.Gyro_X_b) * dt)
        self.Attitude.Gyro_Yaw -=math.radians( (self.Attitude.Gyro_Z-self.Attitude.Gyro_Z_b) * dt)

        #Complimentary estimation
        self.Attitude.Roll=(self.alpha)*(self.Attitude.Roll + (self.Attitude.Gyro_Y-self.Attitude.Gyro_Y_b) * dt) + (1-self.alpha)*(self.Attitude.Acc_Roll)
        self.Attitude.Pitch=(self.alpha)*(self.Attitude.Pitch - (self.Attitude.Gyro_X-self.Attitude.Gyro_X_b) * dt) + (1-self.alpha)*(self.Attitude.Acc_Pitch)


        self.send_Att_msg()

   

    def send_Att_msg(self):
        
            

        self._Att_msg.x=self.Attitude.Roll
        self._Att_msg.y=self.Attitude.Pitch
        self._Att_msg.z=self.Attitude.Gyro_Yaw
        self.ros_pub_att.publish(self._Att_msg)

    @property
    def is_Estimator_connected(self):
        print time.time() - self._last_time_IMU_rcv
        return(time.time() - self._last_time_IMU_rcv < self._timeout_s)

    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            #print self._last_time_IMU_rcv, self.is_Estimator_connected
            #if not self.is_controller_connected:
                #self.set_actuators_idle()

            rate.sleep()

if __name__ == "__main__":
    att_Est     =  Attitude_Est ()
    att_Est.run()
