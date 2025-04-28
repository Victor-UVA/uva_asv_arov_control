import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import numpy as np
import time

from bluerov_interfaces.srv import PositionTargetLocalNED, SetMAVMode, SetArmDisarm
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Forms_Experiment_Controller(Node):
    def __init__(self):
        super().__init__('forms_expriment_controller')
        self.pos_cli = self.create_client(PositionTargetLocalNED, 'set_position_target_local_ned')
        self.mode_cli = self.create_client(SetMAVMode, 'set_mav_mode')
        self.arm_cli = self.create_client(SetArmDisarm, 'set_arm_disarm')
        while not self.pos_cli.wait_for_service(timeout_sec=1.0) and not self.mode_cli.wait_for_service(timeout_sec=1.0)\
              and not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('BlueROV services not available, waiting again...')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 0.0]

        self.home_position = [0.0, 0.0, 0.0]
        self.home_yaw = 0.0

        self.odom_sub = self.create_subscription(Odometry, 'bluerov/pose_from_dvl', self.odom_callback, 10)
        self.odom_sub

    def send_position_target_local_ned(self):
        request = PositionTargetLocalNED.Request()

        request.x = float(self.home_position[0])
        request.y = float(self.home_position[1])
        request.z = float(self.home_position[2])

        request.vx = 0.0
        request.vy = 0.0
        request.vz = 0.0

        request.ax = 0.0
        request.ay = 0.0
        request.az = 0.0

        request.yaw = float(self.home_yaw)
        request.yaw_rate = 0.0

        return self.pos_cli.call_async(request)
    
    def send_set_mode(self, mode):
        request = SetMAVMode.Request()

        request.mode = mode
        return self.mode_cli.call_async(request)
    
    def send_arm_disarm(self, set_arm: bool):
        request = SetArmDisarm.Request()

        request.set_arm = set_arm

        return self.arm_cli.call_async(request)
    
    def odom_callback(self, msg: Odometry):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z

        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w

    def single_trial(self, control: Twist):
        '''
        Function to run a single trial for a given command.

        Args:
            control (Twist): The control to send to the vehicle in m/s and rad/s
        '''

        # Send command to return to home position and wait there for 0.5 s to settle
        return_to_home = False
        while not return_to_home:
            self.get_logger().info('Sending home command')

            arm_future = self.send_arm_disarm(True)
            rclpy.spin_until_future_complete(self, arm_future)
            arm_response = arm_future.result()

            pos_future = self.send_position_target_local_ned()
            rclpy.spin_until_future_complete(self, pos_future)
            pos_response = pos_future.result()

            mode_future = self.send_set_mode('guided')
            rclpy.spin_until_future_complete(self, mode_future)
            mode_response = mode_future.result()

            return_to_home = mode_response and pos_response and arm_response

        timer = 0.0
        while timer < 0.5:
            rclpy.spin_once(self)
            self.get_logger().info(f'Waiting at home: {timer:.2f}/0.50')
            self.get_logger().info(f'{Rotation.from_quat(self.orientation).as_euler('xyz')[2] - self.home_yaw}')
            if abs(np.linalg.norm(np.array(self.position) - np.array(self.home_position))) < 0.1 and\
            abs(np.linalg.norm(Rotation.from_quat(self.orientation).as_euler('xyz')[2] - self.home_yaw)) < 0.01:
                timer += 0.05
                time.sleep(0.05)
            else:
                timer = 0.0
                time.sleep(0.05)

        # Send the control to test for 0.5 s and then stop and wait 1 s
        mode_future = self.send_set_mode('depth_hold')
        rclpy.spin_until_future_complete(self, mode_future)
        mode_response = mode_future.result()

        self.get_logger().info('Sending controls')
        self.cmd_vel_pub.publish(control)
        time.sleep(0.5)
        
        stop_command = Twist()
        stop_command.linear.x, stop_command.linear.y, stop_command.linear.z  = 0.0, 0.0, 0.0
        stop_command.angular.x, stop_command.angular.y, stop_command.angular.z = 0.0, 0.0, 0.0

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
                    control.linear.x = 0.1 * x
                    control.linear.y = 0.1 * y
                    control.angular.z = 0.075 * yaw
                    
                    for i in range(2):
                        self.get_logger().info(f'Running trial {i} with controls X: {x}, Y: {y}, Yaw: {yaw}')
                        self.single_trial(control)


def main():
    rclpy.init()

    forms_experiment_contoller = Forms_Experiment_Controller()

    rclpy.spin_once(forms_experiment_contoller)
    forms_experiment_contoller.home_position = forms_experiment_contoller.position
    forms_experiment_contoller.home_yaw = Rotation.from_quat(forms_experiment_contoller.orientation).as_euler('xyz')[2]

    forms_experiment_contoller.run_experiment()

    forms_experiment_contoller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()