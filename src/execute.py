#!/usr/bin/env python

from re import T
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
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        self.log_info("Starting %s service" % (NAME))
        # position = {'x': 0.5, 'y' : 0.5}
        # quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        # self.goto(position, quaternion)
        self.load_json()

    def execute_command_callback(self, req):
        f_lines = open("~/catkin_ws/src/painted/test.json")
        start_pos_test = [0,0]
        lines_data = json.load(f_lines)
        all_moves = [] #move of all drawings
        A_moves = [] #move of one drawings
        last_pos = start_pos_test
        #calc_path_to(last_pos, next_pos, 0)
        for drawing in lines_data["drawingpath"]: #drawing has 1 startpos and moves
            for set_type in drawing: #set_type is either startpos or drawing plan
                print(set_type)
                if set_type == "drawing_plan":
                        A_moves = self.moveset_iter(drawing, set_type)
                if set_type == "starting_pos":
                        print("Starting position is " + str(drawing[set_type]))
                        next_pos = drawing[set_type]
                        connection_path = self.calc_path_to(last_pos, next_pos, 0)
                if set_type == "starting_pos" and len(all_moves) == 0:
                        all_moves = [connection_path]
                elif set_type != "starting_pos":    
                        all_moves.append(A_moves)
                elif set_type == "starting_pos" and len(all_moves) > 1:
                        all_moves.append(connection_path)
            last_pos = self.calc_end_pos(A_moves, next_pos)
        
        return True

    def load_json(self):
        f_lines = open("./test.json")
        start_pos_test = [0,0]
        lines_data = json.load(f_lines)
        all_moves = [] #move of all drawings
        A_moves = [] #move of one drawings
        starting_pos = [] #all starting position
        drawing_instruction = []
        last_pos = start_pos_test
        #calc_path_to(last_pos, next_pos, 0)
        for drawing in lines_data["drawingpath"]: #drawing has 1 startpos and moves
            for set_type in drawing: #set_type is either startpos or drawing plan
                print(set_type)
                if set_type == "drawing_plan":
                        A_moves = self.moveset_iter(drawing, set_type)
                        drawing_instruction.append(self.moveset_iter(drawing, set_type))
                if set_type == "starting_pos":
                        print("Starting position is " + str(drawing[set_type]))
                        next_pos = drawing[set_type]
                        connection_path = self.calc_path_to(last_pos, next_pos, 0)
                        starting_pos.append(drawing[set_type])
                if set_type == "starting_pos" and len(all_moves) == 0:
                        all_moves = [connection_path]
                elif set_type != "starting_pos":    
                        all_moves.append(A_moves)
                elif set_type == "starting_pos" and len(all_moves) > 1:
                        all_moves.append(connection_path)
            last_pos = self.calc_end_pos(A_moves, next_pos)
        self.log_info(starting_pos)
        self.log_info(drawing_instruction)

    def moveset_iter(self, all_moves, cat_name): #all_moves is drawing, cat_name is set_type
        A_moves = []
        for move in all_moves[cat_name]: #move contains one distance and one angle
            curr_dist = 0
            curr_angle = 0
            for move_type in move: #movetype is either angle or distance
                if move_type == "move":
                    curr_dist = move[move_type]
                    print("We must move " + str(curr_dist))
                elif move_type == "rotate":
                    curr_angle = move[move_type]
                    print("We must rotate " + str(curr_angle))
            curr_move = tuple([curr_dist,curr_angle])
            if len(A_moves) == 0:
                A_moves = [curr_move]
            else:
                A_moves.append(curr_move)  
        return A_moves

    def calc_end_pos(self, A_moves, start_pos): #takes moves and a start pos to calculate where the last position is (continuous)
        curr_pos = start_pos
        for move in A_moves:
            sin_angle = math.sin(math.radians(move[1]))
            cos_angle = math.cos(math.radians(move[1]))
            curr_pos = [float(curr_pos[0]) + float(sin_angle*move[0]) , float(curr_pos[0]) + float(cos_angle*move[0])]
        return curr_pos


    def calc_end_angle(self, A_moves): #finds total direction change over one drawing attempt TODO
        return
            

    def calc_path_to(self, prev_end, start_pos, start_angle): 
        x_dist = int(start_pos[0]) - int(prev_end[0])
        y_dist = int(start_pos[1]) - int(prev_end[1]) #forces a round down
        angle_rad = math.atan(y_dist/x_dist)
        angle_degrees = (180*angle_rad) / math.pi
        dist = math.sqrt(x_dist^2 + y_dist^2)

        return tuple([dist,angle_degrees])

    def goto(self, pos, quat):
        self.log_info('staring goto pos={0} quat={1}'.format(pos, quat))
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)
        self.log_info('goal send')

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 
        self.log_info('waiting result')

        state = self.move_base.get_state()
        result = False

        print(state)

        if success and state == GoalStatus.SUCCEEDED:
            self.log_info('success')
            # We made it!
            result = True
        else:
            self.log_info('failed')
            self.move_base.cancel_goal()
        self.log_info(result)
        self.goal_sent = False
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
