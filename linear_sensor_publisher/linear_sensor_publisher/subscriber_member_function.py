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
import os
import csv
from rclpy.node import Node

from std_msgs.msg import String
from linear_sensor_msgs.msg import Strain

def write_value(strain, R, timestamp):
    # csv name and file path (NEED CHANGE FOR DIFFERENT TRAILS)
    filename = 'testCAAR.csv'
    path = '/home/wenpeng/Documents/ros2_ws/src/Linear_Sensing_ROS2/data'
    file_path = os.path.join(path, filename)
    data = [strain, R, timestamp]
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow(data)
        print('value done')

class linearSensorSubscriber(Node):

    def __init__(self):
        super().__init__('linear_subscriber')
        self.subscription = self.create_subscription(
            Strain,
            'sensor_linear',
            self.listener_callback,
            10)
        self.flag = False
        self.t0 = 0
        self.subscription  # prevent unused variable warning
        
        
        

    def listener_callback(self, msg):
        if self.flag == False:
            self.t0 = msg.timestamp
            self.flag = True
            # print('flag true')
        strain = msg.strain
        resistance = msg.resistance
        tstamp = msg.timestamp
        # print(self.t0)
        timenow = round(tstamp-self.t0,2)
        print(f'Strain: {strain}, Resistance: {resistance}, time: {timenow} \n')
        write_value(strain, resistance, timenow)
        


def main(args=None):
    rclpy.init(args=args)

    linear_subscriber = linearSensorSubscriber()

    rclpy.spin(linear_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    linear_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
