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

        for move in A_moves:
                sin_angle = math.sin(math.radians(move[1]))
                cos_angle = math.cos(math.radians(move[1]))
                
                curr_pos = [float(curr_pos[0]) + float(sin_angle*move[0]) , float(curr_pos[0]) + float(cos_angle*move[0])]

        return curr_pos


def calc_end_angle(A_moves): #finds total direction change over one drawing attempt TODO
        return
        

def calc_path_to(prev_end, start_pos, start_angle): 
        x_dist = int(start_pos[0]) - int(prev_end[0])
        y_dist = int(start_pos[1]) - int(prev_end[1]) #forces a round down
        angle_rad = math.atan(y_dist/x_dist)
        angle_degrees = (180*angle_rad) / math.pi
        dist = math.sqrt(x_dist^2 + y_dist^2)

        return tuple([dist,angle_degrees])


f_lines = open("test.json")

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
                        A_moves = moveset_iter(drawing, set_type)
                if set_type == "starting_pos":
                        print("Starting position is " + str(drawing[set_type]))
                        next_pos = drawing[set_type]
                        connection_path = calc_path_to(last_pos, next_pos, 0)


                if set_type == "starting_pos" and len(all_moves) == 0:
                        all_moves = [connection_path]
                elif set_type != "starting_pos":    
                        all_moves.append(A_moves)
                elif set_type == "starting_pos" and len(all_moves) > 1:
                        all_moves.append(connection_path)
                        
        last_pos = calc_end_pos(A_moves, next_pos)

        

print(all_moves)



#TODO complete calc_end_angle


        
