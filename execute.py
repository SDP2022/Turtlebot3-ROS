import json
import math

def moveset_iter(all_moves, cat_name): #all_moves is drawing, cat_name is set_type
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

def calc_end_pos(A_moves, start_pos): #takes moves and a start pos to calculate where the last position is (continuous)
        curr_pos = start_pos
        curr_x = float(curr_pos[0])
        curr_y = float(curr_pos[1])
        total_angle = 0
        
        for move in A_moves:
                total_angle = total_angle + move[1]
                
                curr_dist = move[0]
                print("move: " , move)
                sin_angle = math.sin(math.radians(total_angle))
                cos_angle = math.cos(math.radians(total_angle))
                curr_x = curr_x + sin_angle*curr_dist 
                curr_y = curr_y + cos_angle*curr_dist 
                
                print("pos: ", curr_x, curr_y)
                
        curr_pos = (curr_x, curr_y)   #rounded 
        return curr_pos


##def calc_curr_pos(move, prev_pos):
##        curr_dist = move[0]
##        sin_angle = math.sin(move[1])
##        cos_angle = math.cos(move[1])
##        curr_x = float(prev_pos[0]) + sin_angle*curr_dist #rounded
##        curr_y = float(prev_pos[1]) + cos_angle*curr_dist #rounded
##        curr_pos = (curr_x, curr_y)
##        print(curr_pos)
##        return curr_pos
        


def calc_end_angle(A_moves, start_angle): #finds total direction change over one drawing attempt TODO
        
        curr_angle = start_angle

        for move in A_moves:
                curr_angle =+ move[1]
                if curr_angle > 180:
                        curr_angle = curr_angle - 360
                elif curr_angle < -180:
                        curr_angle = curr_angle + 360
                
        return curr_angle

def adjust_final(last_pos, angle_from_north):
        north_move = tuple([0,-1*angle_from_north])
        return north_move
        

def calc_path_to(prev_end, start_pos, start_angle): 
        x_dist = float(start_pos[0]) - float(prev_end[0])
        y_dist = float(start_pos[1]) - float(prev_end[1])
        if x_dist == 0:
                angle_rad = 0
        else:
                angle_rad = math.atan(y_dist/x_dist)
        angle_degrees = (180*angle_rad) / math.pi
        dist = math.sqrt((x_dist*x_dist + y_dist*y_dist))
        return tuple([dist,angle_degrees])


f_lines = open("test.json")

start_pos_test = [0,0]

lines_data = json.load(f_lines)

all_moves = [] #move of all drawings
A_moves = [] #move of one drawings

last_pos = start_pos_test
last_angle = 0

#calc_path_to(last_pos, next_pos, 0)

for drawing in lines_data["drawingpath"]: #drawing has 1 startpos and moves
        for set_type in drawing: #set_type is either startpos or drawing plan
                print(set_type)
                if set_type == "drawing_plan":
                        A_moves = moveset_iter(drawing, set_type)
                        
                if set_type == "starting_pos":
                        print("Current position is " + str(last_pos))
                        print("Next starting position is " + str(drawing[set_type]))
                        next_pos = drawing["starting_pos"].split(",")
                        connection_path = calc_path_to(last_pos, next_pos, last_angle)
                last_pos = tuple([0,0])
                last_angle = 0

                
                if set_type == "starting_pos":
                    if len(all_moves) == 0:
                        all_moves = [connection_path]
                    else:
                        all_moves.append(connection_path)

                    last_pos = calc_end_pos([connection_path], last_pos)
                    #last_angle = calc_end_angle(connection_path, last_angle)
                    last_angle = connection_path[1]
                    robot_adjust = adjust_final(last_pos,last_angle)
                    all_moves.append(robot_adjust)
                        
#makes the robot face north before drawing 

                elif set_type != "starting_pos":
                        for item in A_moves:
                                all_moves.append(item)
                    #all_moves.append(A_moves)
                    
                        last_pos = calc_end_pos(A_moves, next_pos)
                        last_angle = calc_end_angle(A_moves, last_angle)
        

print(all_moves)



#TODO insert calc_end_angle


        
