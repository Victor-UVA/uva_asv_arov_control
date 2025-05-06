import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import numpy as np
import time

from bluerov_interfaces.srv import PositionTargetLocalNED, SetMAVMode, SetArmDisarm
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class vu_zcontrollerSimple(Node):
    def __init__(self):
        super().__init__('vu_z_control')
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


        # MODIFICATIONS VU

        # The goal of this second odom callback is to populate my z-pose as soon as messages start to arrive.
        self.odom_sub2 = self.create_subscription(Odometry, 'bluerov/pose_from_dvl', self.odom_callback2, 10)
        self.odom_sub2

        self.testStart = 0 # bool if test is started or not

        self.reachedGoal = 0 # bool for if we reached goal or not, 1 is has reached goal, 0 is still trying
        self.deadzone = 0.005 # minimum magnitude of cmd_vel before cutting off to zero
        self.Kp = 0.4
        self.Ki = 0.01
        self.HzRate = 10.0 #the rate should be approx 10, so 0.1s between z-pose updates
        self.saturation_vel = 0.2

        self.initialZPose = 0.0
        self.currentZPose = 0.0
        self.commandedGoal = 0.0
        self.errorOld = 0.0
        self.error = 0.0
        self.goal_depth = 1.0

        self.cummulativeError = 0.0 #summation of z_error since command given
        self.timeElasped = 0 # Maybe this can be discrete timesteps passed instead of actual time
        
        # self.i = 0
        # timer_period2 = 0.1 # seconds
        # self.timer2 = self.create_timer(timer_period2, self.startTest) #delays test start until after initialization
        self.startTest()
        
        timer_period3 = 0.1 # seconds
        self.timer3 = self.create_timer(timer_period3, self.checkTest)

        timer_period4 = 0.001 # seconds
        self.timer4 = self.create_timer(timer_period4, self.calculateControl)
        self.get_logger().info('Init finished')

        
    # MODIFICATIONS VU
    def startTest(self): #startTest is the method, testStart is the bool :)

        # This command initializes on start, then every 0.1s (or T2 interval) will add 1 to the counter. Start test after 30 intervals
        # I do this to make sure that all variables have been populated with probable non-zero things

        arm_future = self.send_arm_disarm(True)
        rclpy.spin_until_future_complete(self, arm_future)
        arm_response = arm_future.result() 

        mode_future = self.send_set_mode('manual')
        rclpy.spin_until_future_complete(self, mode_future)
        mode_response = mode_future.result() #

        self.testStart = 1 #start the test when it hits 30, exactly one time per run at around 3s.
        if self.initialZPose == 0.0: #if no initial data saved, then save this data. Only happens once per test
            self.initialZPose = self.currentZPose
        if self.commandedGoal == 0.0: #if no goal data saved, then save this data. Only happens once per test
            self.commandedGoal = self.initialZPose + self.goal_depth # GOAL IS 0.5 meters deeper, so add since z is strictly positive

        self.get_logger().info('Finished test start')

    def checkTest(self):
        if self.testStart == 1:
            if abs(self.error) < 0.02 and abs(self.errorOld) < 0.02: #checks 2 data points
                self.reachedGoal = 1 #goal reached, causes control to send 0 m/s message
                self.testStart = 0 #end the test, means stop calculating new error distance
                self.get_logger().info('Goal reached')
                
    

    def odom_callback2(self, msg: Odometry):
        # The goal of this is to populate my z-pose as soon as messages start arriving, hopefully at 10Hz
        self.currentZPose = msg.pose.pose.position.z

        # this method will also start calculating the error between the commanded goal and current if test is running

        # if self.testStart == 1: # only starts if test is running
        self.errorOld = self.error
        self.error = self.commandedGoal - self.currentZPose # WHICH WAY??? idk :)
        # assume that both values are strictly positive because depth must start after 0m from the surface.
        # assume that depth is positive with z going down, with velocity z+ going down
        # so if goal > current, that means goal is lower down than current, so positive z-velocity is needed, and big-small = positive
        # so if goal < current, that means goal is higher up than current, so negative z-velocity is needed, and small-big = negative
        
        # start integrating
        self.cummulativeError = self.cummulativeError + self.error*0.1 #adds error 10 times a second, so we do 1/10th of magnitude
        self.timeElasped = self.timeElasped + 1


    def calculateControl(self):
        # Make a dummy twist to populate.
        tempControl = Twist()
        tempControl.linear.x, tempControl.linear.y, tempControl.linear.z  = 0.0, 0.0, 0.0 # THIS is ZERO, for now
        tempControl.angular.x, tempControl.angular.y, tempControl.angular.z = 0.0, 0.0, 0.0 # THIS is ZERO, for now

        # if self.reachedGoal != 1: # It should only start sending controls after test is started, which is at least 3s later
        tempControl.linear.z = min(self.saturation_vel, max(-self.saturation_vel, self.Kp*self.error + self.Ki*self.cummulativeError))  #PI control
            
        # if tempControl.linear.z >= self.saturation_vel: # Cap the maximum twist velocity at 0.2 m/s
        #     tempControl.linear.z = self.saturation_vel
        # if tempControl.linear.z <= -self.saturation_vel:
        #     tempControl.linear.z = -self.saturation_vel
        if abs(tempControl.linear.z) <= self.deadzone: #if the magnitude is <=0.02 m/s, then deadzone -> set to 0m/s
            tempControl.linear.z = 0.0

        tempControl.linear.z = -tempControl.linear.z
        self.get_logger().info(f'Control: {tempControl.linear.z:.2f}, Depth: {self.currentZPose:.3f}, Error: {self.error}')
            
        # self.get_logger().info(f'{self.currentZPose}, {self.commandedGoal}, {tempControl.linear.z}')

        self.cmd_vel_pub.publish(tempControl) # publish at the end




    

    


    # MODIFICATIONS END VU

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

            arm_future = self.send_arm_disarm(True) #
            rclpy.spin_until_future_complete(self, arm_future)
            arm_response = arm_future.result() #

            pos_future = self.send_position_target_local_ned()
            rclpy.spin_until_future_complete(self, pos_future)
            pos_response = pos_future.result()

            mode_future = self.send_set_mode('guided')
            rclpy.spin_until_future_complete(self, mode_future)
            mode_response = mode_future.result()

            return_to_home = mode_response and pos_response and arm_response

        time.sleep(1)

        # timer = 0.0
        # while timer < 0.5:
        #     rclpy.spin_once(self)
        #     self.get_logger().info(f'Waiting at home: {timer:.2f}/0.50')
        #     self.get_logger().info(f'Rot Err: {Rotation.from_quat(self.orientation).as_euler('xyz')[2] - self.home_yaw}')
        #     self.get_logger().info(f'Pos Err: {np.linalg.norm(np.array(self.position) - np.array(self.home_position))}')
        #     self.get_logger().info(f'Pos: {self.position}, {self.home_position}')
        #     if abs(np.linalg.norm(np.array(self.position[:1]) - np.array(self.home_position[:1]))) < 0.5 and\
        #     abs(np.linalg.norm(Rotation.from_quat(self.orientation).as_euler('xyz')[2] - self.home_yaw)) < 0.1:
        #         timer += 0.1
        #         time.sleep(0.1)
        #     else:
        #         timer = 0.0
        #         time.sleep(0.1)



        # MODIFICATIONS:

        # Send the control to test for 0.5 s and then stop and wait 1 s
        mode_future = self.send_set_mode('alt_hold') #
        rclpy.spin_until_future_complete(self, mode_future)
        mode_response = mode_future.result() #


        self.get_logger().info('Sending controls')
        timer = 0
        while timer < 2:
            self.cmd_vel_pub.publish(control)
            time.sleep(0.1)
            timer += 0.1
        
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
                    control.linear.x = 0.2 * x
                    control.linear.y = 0.2 * y
                    control.angular.z = 0.2 * yaw
                    
                    for i in range(2):
                        self.get_logger().info(f'Running trial {i} with controls X: {x}, Y: {y}, Yaw: {yaw}')
                        self.single_trial(control)

    


def main():
    rclpy.init()

    forms_experiment_contoller = vu_zcontrollerSimple()

    # rclpy.spin_once(forms_experiment_contoller)
    # if forms_experiment_contoller.home_position == [0.0, 0.0, 0.0]:
    #     forms_experiment_contoller.home_position[0] = forms_experiment_contoller.position[0]
    #     forms_experiment_contoller.home_position[1] = forms_experiment_contoller.position[1]
    #     forms_experiment_contoller.home_position[2] = forms_experiment_contoller.position[2]
    #     forms_experiment_contoller.home_yaw = Rotation.from_quat(forms_experiment_contoller.orientation).as_euler('xyz')[2]

    #forms_experiment_contoller.run_experiment()

    rclpy.spin(forms_experiment_contoller)

    forms_experiment_contoller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()