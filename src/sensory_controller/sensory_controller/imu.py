#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import qwiic_icm20948
from time import *

class IMUNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        self.IMU = qwiic_icm20948.QwiicIcm20948()
        if not self.IMU.connected:
            self.get_logger().error("The Qwiic ICM20948 device isn't connected to the system. Please check your connection")
            return

        self.IMU.begin()

        self.get_logger().info("IMU node initialized successfully")
    
    def gyroscopeRead(self):
        if self.IMU.dataReady():
            self.IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
            print(\
                #'{: 06d}'.format(self.IMU.axRaw)\
                #, '\t', '{: 06d}'.format(self.IMU.ayRaw)\
                #, '\t', '{: 06d}'.format(self.IMU.azRaw)\
                 '\n gX: {: 4.1f}'.format(self.IMU.gxRaw)\
                , '\n gY: {: 4.1f}'.format(self.IMU.gyRaw)\
                , '\n gX: {: 4.1f}'.format(self.IMU.gzRaw)\
                #, '\t', '{: 06d}'.format(self.IMU.mxRaw)\
                #, '\t', '{: 06d}'.format(self.IMU.myRaw)\
                #, '\t', '{: 06d}'.format(self.IMU.mzRaw)\
            )
            sleep(0.5)
        else:
            print("Waiting for data")
            sleep(0.5)


def main(args = None):
    rclpy.init(args = args)
    
    imu = IMUNode()
     
    try:
       #rclpy.spin(imu)
       while(True):
            imu.gyroscopeRead()
    except KeyboardInterrupt:
        pass
    
    imu.get_logger().info("Shutting down..")
    imu.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()