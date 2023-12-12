# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import serial
from rclpy.node import Node

from std_msgs.msg import String
from linear_sensor_msgs.msg import Strain
import tkinter.messagebox
import time
import threading


## Resistance Meter Package
class Usb_rs:

    def __init__(self, gui=False):
        self.ser = serial
        self.gui = gui
    
    #Open port
    def open(self, port, speed):
        ret = False

        try:
            self.ser = serial.Serial(port, speed,timeout=0)
            ret = True
        except Exception as e:
            if self.gui == True:
                tkinter.messagebox.showerror("Open Error", e)
            else:
                print("Open error")
                print(e)
        
        return ret

    #Close port
    def close(self):
        ret = False

        try:   
            self.ser.close()
            ret = True
        except Exception as e:
            if self.gui == True:
                tkinter.messagebox.showerror("Close Error", e)
            else:
                print("Close error")
                print(e)
        
        return ret
    #Send command
    def sendMsg(self, strMsg):
        ret = False

        try:
            strMsg = strMsg + '\r\n'                #Add a terminator, CR+LF, to transmitted command
            self.ser.write(bytes(strMsg, 'utf-8'))  #Convert to byte type and send
            ret = True
        except Exception as e:
            if self.gui == True:
                tkinter.messagebox.showerror("Send Error", e)
            else:
                print("Send Error")
                print(e)

        return ret
    
    #Receive
    def receiveMsg(self, timeout):

        msgBuf = bytes(range(0))                    #Received Data

        try:
            start = time.time()                     #Record time for timeout
            while True:
                if self.ser.inWaiting() > 0:        #Is exist the data in the receive buffer?
                    rcv = self.ser.read(1)          #Receive 1 byte
                    if rcv == b"\n":                #End the loop when LF is received
                        msgBuf = msgBuf.decode('utf-8')
                        break
                    elif rcv == b"\r":              #Ignore the terminator CR
                        pass
                    else:
                        msgBuf = msgBuf + rcv
                
                #Timeout processing
                if  time.time() - start > timeout:
                    msgBuf = "Timeout Error"
                    break
        except Exception as e:
            if self.gui == True:
                tkinter.messagebox.showerror("Receive Error", e)
            else:
                print("Receive Error")
                print(e)
            msgBuf = "Error"

        return msgBuf
    
    #Transmit and receive commands
    def SendQueryMsg(self, strMsg, timeout):
        ret = Usb_rs.sendMsg(self, strMsg)
        if ret:
            msgBuf_str = Usb_rs.receiveMsg(self, timeout)   #Receive response when command transmission is succeeded
        else:
            msgBuf_str = "Error"

        return msgBuf_str

#Timeout(1sec)
Timeout_default = 1

## Resistance Reading Function
def read_R():
    #Instantiation of the usb_rs communication class
    serial1 = Usb_rs()

    #Connect
    # print("Port?")
    port = "/dev/ttyACM0"  
    # print("Speed?")
    speed = 9600
    if not serial1.open(port,speed):
        return
    
    #Send and receive commands

        # print("Please enter the command (Exit with no input)")
    command = "MEAS:RES?"
        #Exit if no input
    # if command == "":
    #     break
        #If the command contains "?"
    if "?" in command :
        msgBuf = serial1.SendQueryMsg(command, Timeout_default)
        #print(msgBuf) 
        #Send only
    else:
        serial1.sendMsg(command)
        
    serial1.close()
    return msgBuf



ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)


class linearSensorPublisher(Node):

    def __init__(self):
        super().__init__('linear_publisher')
        self.publisher_ = self.create_publisher(Strain, 'sensor_linear', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        print('Publisher Init Done!')
  

    def timer_callback(self):
        msg = Strain()
        line = ser.readline()   # read a byte
        if line:
            if (self.i <= 10):
                self.i += 1
            else:
                string = line.decode()  # convert the byte string to a unicode string
                # print(string)
                strain = float(string.replace('\r\n', ''))
                msg.strain = strain
                msg.resistance = data_R + strain + 1.0


        self.publisher_.publish(msg)
        # print(f'Strain: {msg.strain}, Resistance: {msg.resistance} \n')
        


def main(args=None):
    x = threading.Thread(target=thread_function) 
    x.start()
    print("thread started")
    rclpy.init(args=args)

    linear_publisher = linearSensorPublisher()

    rclpy.spin(linear_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    linear_publisher.destroy_node()
    rclpy.shutdown()

## Resistance value reading thread
def thread_function(): 
    global data_R
    while 1:
        # data_R = read_R() # data read from resistance meter
        data_R = 0 # test code without resistance meter
        # print(data_R)


if __name__ == '__main__':
    main()
