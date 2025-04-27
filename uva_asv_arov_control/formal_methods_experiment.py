import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import numpy as np
import time

from bluerov_interfaces.srv import PositionTargetLocalNED
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Forms_Experiment_Controller(Node):
    def __init__(self):
        super().__init__('forms_expriment_controller')
        self.cli = self.create_client(PositionTargetLocalNED, 'set_position_target_local_ned')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set position service not available, waiting again...')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 0.0]

        # self.home_position = [0.0, 0.0, 0.0]
        # self.home_yaw = 0.0

        self.odom_sub = self.create_subscription(Odometry, 'bluerov/pose_from_dvl', self.odom_callback, 10)
        self.odom_sub

        self.home_position = self.position
        self.home_yaw = Rotation.from_quat(self.orientation).as_euler('z')

        self.run_experiment()

    def send_position_target_local_ned(self):
        request = PositionTargetLocalNED.Request()

        request.x = self.home_position[0]
        request.y = self.home_position[1]
        request.z = self.home_position[2]

        request.vx = 0
        request.vy = 0
        request.vz = 0

        request.ax = 0
        request.ay = 0
        request.az = 0

        request.yaw = self.home_yaw
        request.yaw_rate = 0

        return self.cli.call_async(request)
    
    def odom_callback(self, msg: Odometry):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def single_trial(self, control: Twist):
        '''
        Function to run a single trial for a given command.

        Args:
            control (Twist): The control to send to the vehicle in m/s and rad/s
        '''

        # Send command to return to home position and wait there for 0.5 s to settle
        response = False
        while not response:
            self.get_logger().info('Sending home command')
            future = self.send_position_target_local_ned()
            rclpy.spin_until_future_complete(self, future) # TODO Might not need this inside the node
            response = future.result()

        timer = 0.0
        while timer < 0.5:
            self.get_logger().info(f'Waiting at home: {timer:.2f}/0.50')
            if abs(np.linalg.norm(self.position - self.home_position)) < 0.1 and\
            abs(np.linalg.norm(Rotation.from_quat(self.orientation).as_euler('z') - self.home_yaw)) < 0.1:
                timer += 0.05
                time.sleep(0.05)
            else:
                timer = 0.0

        # Send the control to test for 0.5 s and then stop and wait 1 s
        self.get_logger().info('Sending control')
        self.cmd_vel_pub.publish(control)
        time.sleep(0.5)
        
        stop_command = Twist()
        stop_command.linear = [0.0, 0.0, 0.0]
        stop_command.angular = [0.0, 0.0, 0.0]

        self.get_logger().info('Sending stop control')
        self.cmd_vel_pub.publish(stop_command)    
        time.sleep(1)

    def run_experiment(self):
        control_x = [-1, 0, 1]
        control_y = [0, 1]
        control_yaw = [0, 1]

        control = Twist()

        for x in control_x:
            for y in control_y:
                for yaw in control_yaw:
                    control.linear = [0.1 * x, 0.1 * y, 0.0]
                    control.angular.z = 0.075 * yaw
                    
                    for i in range(10):
                        self.get_logger().info(f'Running trial {i} with controls X: {x}, Y: {y}, Yaw: {yaw}')
                        self.single_trial(control)


def main():
    rclpy.init()

    forms_experiment_contoller = Forms_Experiment_Controller()

    rclpy.spin(forms_experiment_contoller)

    forms_experiment_contoller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()