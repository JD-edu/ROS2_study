#!/usr/bin/env python3
#from JDamr_lib import JDamr
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, LaserScan
import serial
import threading 
import time 
from rclpy.node import Node


'''
In this script, we study follwings:
1. How to write basic ROS node for robot car
2. Getting encoder data from Arduino and puslish encoder value  
'''

'''
Class for vehicle control 
'''
class JDamr(object):
    def __init__(self, com="/dev/ttyACM0"):
        self.ser = serial.Serial(com, 115200)
        self.HEAD = 0xf5
       
        self.CMD_GET_ENCODER = 0x03

        self.encoder1 = 0
        self.encoder2 = 0

        if self.ser.isOpen():
            print("JDamr serial port opened!")
        else:
            print("Can't open JDamr serial port!")

        time.sleep(1)

    '''
    Protocol 
    - Packets have following bytes.
      - Header byte 
      - length byte
      - command byte
      - payload bytes 
      - checksum byte 
    '''

    # Starting receiving thread 
    def receive_data(self):     
        self.ser.flushInput()
        while True:
            head = bytearray(self.ser.read())[0]
            if head == self.HEAD:
                length = bytearray(self.ser.read())[0]  
                payload = [] 
                for i in range(length-1):
                    value = bytearray(self.ser.read())[0]
                    payload.append(value)
                self.parse_cmd(payload)

    # Python thread for receiving packet 
    def receive_thread(self):
        try:
            taks_name = "serial_thread"
            rx_task = threading.Thread(target=self.receive_data, name=taks_name)
            rx_task.setDaemon(True)
            rx_task.start()
            print("Start serial receive thread ")
            time.sleep(.05)
        except:
            pass

    # parsing command from incoming packet 
    def parse_cmd(self, payload):
        if self.CMD_GET_ENCODER == payload[0]:
            print(payload)
            encode1_str = payload[1:5]
            encode2_str = payload[5:9]
            encode3_str = payload[9:13]
            encode4_str = payload[13:17]
            self.encode1 = int.from_bytes(encode1_str, byteorder="big")
            print(self.encode1)
            self.encode2 = int.from_bytes(encode2_str, byteorder="big")
            print(self.encode2)


# ROS node class 
class jdamr_driver(Node): 
    def __init__(self):
        super().__init__('jdamr_driver')
        self.jdamr = JDamr()
        time.sleep(1)
        #self.jdamr.receive_thread()

 
    #def reset_amr(self):
    #    pass 

def main(args=None):
    rclpy.init(args=args)

    jdamr_1 = jdamr_driver()

    rclpy.spin(jdamr_1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    jdamr_1.destroy_node()
    rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()


    '''
    rospy.init_node("jdamr_driver_node", anonymous=False)
    rate = rospy.Rate(10) # 10hz
    driver = jdamr_driver()
    rospy.spin()
    '''