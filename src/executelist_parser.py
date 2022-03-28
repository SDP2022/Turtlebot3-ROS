import json
import math
#from point_transformation.py import relative_point_geo_to_real_pen

def calc_angle(start_x, start_y, end_x, end_y): #takes moves and a start pos to calculate where the last position is (continuous)
    x_dist = abs(start_x - end_x)
    y_dist = abs(start_y - end_y)
    if y_dist != 0:
        angle = math.atan(x_dist / y_dist)
    elif start_y > end_y:
        angle = -math.pi / 2
    elif end_y <= start_y:
        angle = math.pi / 2
    else:
        angle = 0
    
    return angle

def calc_dist(start_x, start_y, end_x, end_y):
    x_dist = abs(start_x - end_x)
    y_dist = abs(start_y - end_y)
    sl_dist = math.sqrt(x_dist*x_dist + y_dist*y_dist)
    
    return sl_dist

#copied

geojson_x = -3.200362642
geojson_y = 55.937597084

pen_offset_x = -0.0102
pen_offset_y = -0.0088

division_x = 62345.65
division_y = 111139
scale = 100

irl_x = -0.095 
irl_y = -0.075

def relative_point_geo_to_real_pen(x_coord, y_coord):
	change_in_x = (x_coord - geojson_x) * division_x
	change_in_y = (y_coord - geojson_y) * division_y

	## get the relative x, y movement

	x_movement = change_in_x / scale
	y_movement = change_in_y / scale

	##return point in robot world.
	return (irl_x + x_movement - pen_offset_x), (irl_y + y_movement - pen_offset_y)

#end_of_copy



def ExecuteListParser(lines_data):
    move_triple = []
    curr_move = [0,0,0]


    for line in lines_data:
        curr_move = [0,0,0]
        
        x_0 = relative_point_geo_to_real_pen(line[0][0], line[0][1])[0]
        y_0 = relative_point_geo_to_real_pen(line[0][0], line[0][1])[1]
        x_1 = relative_point_geo_to_real_pen(line[1][0], line[1][1])[0]
        y_1 = relative_point_geo_to_real_pen(line[1][0], line[1][1])[1]
        
        #x_0 = line[0][0]
        #y_0 = line[0][1]
        #x_1 = line[1][0]
        #y_1 = line[1][1]
        
        curr_move[0] = [x_0,y_0]
        curr_move[1] = calc_angle(x_0,y_0,x_1,y_1)
        curr_move[2] = calc_dist(x_0,y_0,x_1,y_1)
        
        if len(move_triple) == 0:
            move_triple = curr_move
        else:
            move_triple.append(curr_move)
        print(curr_move)
    print(move_triple)

    return move_triple


#f_lines = open("test3.json")

#lines_data = json.load(f_lines)

#ExecuteListParser(lines_data)
