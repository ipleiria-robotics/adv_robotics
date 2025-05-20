#!/usr/bin/env python3

# Copyright (c) 2023, Hugo Costelha
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
# * Neither the name of the Player Project nor the names of its contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE 4 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

'''@package docstring
Teleoperation application for the 2nd Advanced Robotics project.
Note that the "\r" in the sring ends is due to curses not playing well with the
print function.
'''

# Library packages needed
import time
import curses

# ROS API
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, UInt8MultiArray
from sensor_msgs.msg import BatteryState
from action_msgs.msg import GoalStatus
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, \
    ReliabilityPolicy

# Our modules
from markers_msgs.msg import Markers
from ar_py_utils.utils import clipValue
import ar_utils.srv
import ar_utils.msg
import ar_utils.action
from ar_py_utils.utils import printxy

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 0.5  # [m/s]
MAX_ANG_VEL = 1.14  # 90º/s (in rad/s)
# Forklift (static) information
FORKLIFT_DOWN = 0.0  # Down position
FORKLIFT_UP = 0.07  # Up position


class Teleop(Node):
    def __init__(self, stdscr) -> None:
        '''
        Initialize class instance.
        '''
        super().__init__('teleop')

        # Handle screen and input
        self.stdscr = stdscr

        # The forklift will be controlled using an action
        self.forklift_up = False
        self.forklift_down = False
        self.clientMoveForklift = ActionClient(
            self, ar_utils.action.MoveForklift, 'ActionMoveForklift')
        # Wait for the action server to be up and running
        self.clientMoveForklift.wait_for_server()

        # Parts information
        self.parts_status = []

        # Battery level and recharge action (assume 100% for starters)
        self.battery_level = 1.0
        self.clientRecharge = ActionClient(
            self, ar_utils.action.Recharge, 'ActionRecharge')
        # Wait for the action server to be up and running
        self.clientRecharge.wait_for_server()

        # Setup publisher for speed control
        self.vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 1)

        # QOS to receive info for some topics that are published less
        # frequently, particularly if they havebeen published before
        # subscription.
        qos_transient_local = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)

        # Setup subscriber for the forklift status
        self.create_subscription(
            ar_utils.msg.ForkliftState,
            'forklift/state',
            self.forkliftCallback,
            qos_profile=qos_transient_local)

        # Setup subscriber for the parts sensor
        self.create_subscription(
            UInt8MultiArray,
            'parts_sensor',
            self.partsSensorCallback,
            qos_profile=qos_transient_local)

        # Setup subscriber for parts detection (simulated with markers)
        self.create_subscription(
            Markers,
            'markers',
            self.markersCallback, 1)

        # Setup subscriber for the battery level
        self.create_subscription(
            BatteryState,
            'battery/state',
            self.batteryLevelCallback, 1)

        # Battery charging service
        self.charge_battery = self.create_client(
            ar_utils.srv.StartCharging,
            'battery/charge')
        # Wait for the service to be available
        while self.charge_battery.wait_for_service(2.0) is False:
            self.get_logger().info(
                'Still waiting for the battery service at ' +
                'battery/charge...')

    def batteryLevelCallback(self, msg: BatteryState):
        ''' Process battery level related messages'''
        # If the battery as dropped below 5%
        if (msg.percentage < 0.05) and (self.battery_level >= 0.05):
            self.stdscr.addstr(
                11, 0, 'Battery is getting extremely low ' +
                f'({self.battery_level*100:0.1f}%)...\n\r')
        elif (msg.percentage < 0.20) and (self.battery_level >= 0.20):
            self.stdscr.addstr(
                11, 0, 'Battery is getting low ' +
                f'({self.battery_level*100:0.1f}%)...\n\r')
        else:
            self.stdscr.addstr(
                11, 0, f'Battery level: {self.battery_level*100:0.1f}%\n\r')

        # Store the current value
        self.battery_level = msg.percentage

    def forkliftCallback(self, msg: ar_utils.msg.ForkliftState):
        ''' Process forklift related messages '''

        # Show forklift position
        self.stdscr.addstr(
            9, 0,
            f'Forklift position = {msg.position:.3f} [m]\n\r')

        # If the forklift is moving, then it is neither down or up
        if msg.moving:
            self.forklift_up = False
            self.forklift_down = False

    def partsSensorCallback(self, msg):
        ''' Process the parts sensor data '''

        self.parts_status = [x for x in msg.data]

        # Show parts status
        self.stdscr.addstr(8, 0, f'Parts status: {self.parts_status}\n\r')

    def markersCallback(self, msg: Markers):
        ''' Process received markers data from parts/locations'''
        # Display found parts information
        if msg.num_markers > 0:
            self.stdscr.addstr(13, 0,
                               f'Found {msg.num_markers} parts/locations.')
            for i in range(msg.num_markers):
                self.stdscr.addstr(
                    13+i, 0,
                    'Part seen at : (range, bearing) = (' +
                    f'{msg.range[i]:.2f}, {msg.bearing[i]:.2f})\n\r')

    def handle_keyboard(self):
        '''
        Process inputs from the keyboard.
        '''
        # Terminal settings
        # Check https://docs.python.org/3.6/howto/curses.html for more info
        # self.stdscr.clear()  # Clear screen

        #
        # Create robot related objects
        #
        # Linear and angular velocities for the robot (initially stopped)
        lin_vel = 0.0
        ang_vel = 0.0
        l_scale = 1.0
        a_scale = 1.0
        # Velocity increase for each keystroke
        delta_lin_vel = 0.1
        delta_ang_vel = 5.0/180.0*3.14
        vel_cmd = Twist()
        forklift_pos_cmd = Float32(data=0.0)

        # Output usage information
        self.stdscr.nodelay(True)  # Do not block when getting a key
        self.stdscr.addstr(
            'Reading from keyboard\n\r' +
            'Use UP, LEFT, DOWN, RIGHT and space to move front, left, back, ' +
            'right and stop, respectively.\n\r' +
            'Use e and d to move the forklift up and down, respectively\n\r' +
            'Use + to start charging, respectively\n\r' +
            'Press q to quit.\n\r' +
            '---------------------------\n\r')
        self.stdscr.addstr('\nPRESS ANY KEY TO START...\n\r')
        while rclpy.ok():
            nChar = self.stdscr.getch()
            if nChar is not curses.ERR:
                break
            time.sleep(0.5)

        # Infinite loop
        while rclpy.ok():
            # Get char, and act accordingly
            nChar = self.stdscr.getch()
            key_pressed = True

            if nChar == curses.ERR:
                # Decelerate automatically if no key was pressed
                lin_vel *= 0.9
                ang_vel *= 0.9
                key_pressed = False
            elif nChar == ord('q'):
                break
            elif nChar == curses.KEY_UP:
                # Increase linear velocity
                lin_vel += delta_lin_vel
                key_pressed = True
            elif nChar == curses.KEY_DOWN:
                # Decrease linear velocity
                lin_vel -= delta_lin_vel
                key_pressed = True
            elif nChar == curses.KEY_LEFT:
                # Increase angular velocity
                ang_vel += delta_ang_vel
                key_pressed = True
            elif nChar == curses.KEY_RIGHT:
                # Decrease angular velocity
                ang_vel -= delta_ang_vel
                key_pressed = True
            elif nChar == ord(' '):
                # Stop robot
                lin_vel = 0
                ang_vel = 0
            elif nChar == ord('e'):
                # Send forklit up
                goal = ar_utils.action.MoveForklift.Goal(position=FORKLIFT_UP)
                result_response = self.clientMoveForklift.send_goal(goal)
                if result_response.status != GoalStatus.STATUS_SUCCEEDED:
                    self.stdscr.addstr(
                        13, 0, 'Action MoveForklift failed')
                else:
                    self.forklift_up = True
                    self.forklift_down = False
                    self.stdscr.addstr(
                        13, 0,
                        'Action MoveForklift succeeded with result' +
                        f'{result_response.result.success}, forklift is up.')
            elif nChar == ord('d'):
                # Send forklit down
                goal = ar_utils.action.MoveForklift.Goal(
                    position=FORKLIFT_DOWN)
                result_response = self.clientMoveForklift.send_goal(goal)
                if result_response.status != GoalStatus.STATUS_SUCCEEDED:
                    self.stdscr.addstr(
                        13, 0,
                        'Action MoveForklift failed with result:' +
                        f'{result_response.result.success}')
                else:
                    self.forklift_up = False
                    self.forklift_down = True
                    self.stdscr.addstr(
                        13, 0,
                        'Action MoveForklift succeeded with result' +
                        f'{result_response.result.success}, forklift is down.')
            elif nChar == ord('+'):
                # Request charging
                goal = ar_utils.action.Recharge.Goal(target_battery_level=1.0)
                result_response = self.clientRecharge.send_goal(goal)
                if result_response.status != GoalStatus.STATUS_SUCCEEDED:
                    self.stdscr.addstr(13, 0, 'Action Recharge failed!')
                else:
                    self.stdscr.addstr(
                        13, 0,
                        'Action Recharge succeeded with result:' +
                        result_response.result.battery_level.__str__())

            # Limit maximum velocities
            lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
            ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

            # Show desired velocity
            # print(f'Robot desired velocity = {lin_vel:.2f} [m/s], ' +
            #       f'{ang_vel*180.0/pi:.2f} [°/s]\n')

            # Send velocity commands
            vel_cmd.angular.z = a_scale*ang_vel
            vel_cmd.linear.x = l_scale*lin_vel
            self.vel_pub.publish(vel_cmd)

            # Limit the amount of messages when no key is pressed
            if key_pressed is False:
                time.sleep(0.1)

        vel_cmd.angular.z = 0.0
        vel_cmd.linear.x = 0.0
        self.vel_pub.publish(vel_cmd)

        rclpy.shutdown()


def execute(stdscr):
    '''
    This is the actual function that starts everything
    '''
    # Initiate python ROS Python control
    rclpy.init()

    teleop_node = Teleop(stdscr=stdscr)

    # We will execute each node in its own thread. This is important to make
    # sure that the TF listener is continuously updated.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(teleop_node)
    # Include our handle_keyboard function
    executor.create_task(teleop_node.handle_keyboard)

    # Run both nodes until shutdown
    executor.spin()
    executor.shutdown()


def main(args=None):
    '''
    Main function
    Controls the robot using the keyboard keys and outputs relevant information
    '''
    # print(
    #     'Reading from keyboard\n\r' +
    #     'Use UP, LEFT, DOWN, RIGHT and space to move front, left, back, ' +
    #     'right and stop, respectively.\n\r' +
    #     'Use e and d to move the forklift up and down, respectively\n\r' +
    #     'Use + to start charging, respectively\n\r' +
    #     'Press q to quit.\n\r' +
    #     '---------------------------\n\r')
    # input('PRESS ENTER TO START...\n\r')

    try:
        curses.wrapper(execute)
    except KeyboardInterrupt:
        pass


'''
This is what is actually called when we run this python script. It then calls
the main function defined above, but using the curses wrapper in order to
control the terminal
'''
if __name__ == '__main__':
    main()
