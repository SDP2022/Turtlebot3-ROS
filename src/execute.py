#!/usr/bin/env python

from re import S, T
from painted.srv import *
import rospy
import json
import math
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

NAME = 'execute_node'


class execute_node():
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.execute_service = rospy.Service(
            'execute_service', ExecuteCommand, self.execute_command_callback)
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.wait_for_service('pen_service')
        rospy.wait_for_service('control_service')
        self.log_info("Starting %s service" % (NAME))
        # position = {'x': 0.5, 'y' : 0.5}
        # quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        # self.goto(position, quaternion)
        # self.load_json_demo2()
        self.demo2_with_manual_slam()

    def execute_command_callback(self, req):
        # self.load_json_demo2()
        self.log_info('starting demo2')
        self.demo2_with_manual_slam()
        return True

    def demo2_with_manual_slam(self):
        self.goto(-1, -0.5)
        # self.pen_command(True)
        self.control_command(2, 0)
        # self.pen_command(False)
        self.goto(-1, -0.4)
        # self.pen_command(True)
        self.control_command(2, 0)
        # self.pen_command(False)
        
    def load_json_demo2(self):
        f_lines = open("./test.json")
        lines_data = json.load(f_lines)
        starting_pos = []  # all starting position
        drawing_instruction = []
        # drawing has 1 startpos and moves
        for drawing in lines_data["drawingpath"]:
            for set_type in drawing:  # set_type is either startpos or drawing plan
                if set_type == "drawing_plan":
                    drawing_instruction.append(
                        self.moveset_iter(drawing, set_type))
                elif set_type == "starting_pos":
                    print("Starting position is " + str(drawing[set_type]))
                    start_pos = str(drawing[set_type])
                    print(start_pos)
                    start_pos = start_pos.split(',')
                    starting_pos.append((float(start_pos[0]), float(start_pos[1])))
        self.log_info(starting_pos)
        self.log_info(drawing_instruction)
        self.log_info('Json loaded, start drawing')
        for i in range(len(starting_pos)):
            next_shape_starting_pos = starting_pos[i]
            self.log_info('Shape: Goto %s ' % (str(next_shape_starting_pos)))
            # self.goto(next_shape_starting_pos[0], next_shape_starting_pos[1])
            self.log_info('Shape: Pen down')
            self.pen_command(True)
            for instruction in drawing_instruction[i]:
                if instruction[0] != -1:
                    self.log_info('Move %s' % (instruction[0]))
                    self.control_command(instruction[0], 0)
                else:
                    self.log_info('Shape: Pen up')
                    self.pen_command(False)
                    self.log_info('Rotate %s' % (instruction[1]))
                    self.control_command(0, instruction[1])
                    self.log_info('Shape: Pen down')
                    self.pen_command(True)
            self.log_info('Shape: Pen up')
            self.pen_command(False)
        self.log_info('Drawing completed')

    # all_moves is drawing, cat_name is set_type
    def moveset_iter(self, all_moves, cat_name):
        A_moves = []
        for move in all_moves[cat_name]:  # move contains one distance and one angle
            curr_dist = 0
            curr_angle = 0
            for move_type in move:  # movetype is either angle or distance
                if move_type == "move":
                    curr_dist = move[move_type]
                    print("We must move " + str(curr_dist))
                elif move_type == "rotate":
                    curr_angle = move[move_type]
                    print("We must rotate " + str(curr_angle))
            curr_move = tuple([curr_dist, curr_angle])
            if len(A_moves) == 0:
                A_moves = [curr_move]
            else:
                A_moves.append(curr_move)
        return A_moves

    def control_command(self, displacement, rotation):
        try:
            control_command = rospy.ServiceProxy('control_service', ControlCommand)
            print("Requesting displacement=%s rotation=%s"%(displacement, rotation))
            resp1 = control_command(displacement, rotation)
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def pen_command(self, pen_status):
        try:
            pen_command = rospy.ServiceProxy('pen_service', PenCommand)
            print("Requesting pen_status=%s"%(pen_status))
            resp1 = pen_command(pen_status)
            d = rospy.Duration(2, 0)
            rospy.sleep(d)
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def goto(self, x, y):
        pos = {'x': x, 'y' : y}
        quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        self.log_info('Requesting goto pos={0} quat={1}'.format(pos, quat))
        # Send a goal
        # self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)
        self.log_info('Goal send')

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))
        self.log_info('Waiting result')

        state = self.move_base.get_state()
        result = False

        print(state)

        if success and state == GoalStatus.SUCCEEDED:
            self.log_info('Goal success')
            # We made it!
            result = True
        else:
            self.log_info('Goal Failed')
            self.move_base.cancel_goal()
        self.log_info(result)
        # self.goal_sent = False
        return result

    def shutdown(self):
        # stop turtlebot, reset
        self.log_info("Stopping %s node" % (NAME))

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting %s service" % (NAME))
    execute_node()
    rospy.spin()
