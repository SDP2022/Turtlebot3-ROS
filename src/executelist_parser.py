import json
import math

class ExecuteListParser(object):
    def __init__(self, geojson_x, geojson_y):
        self.geojson_x = geojson_x
        self.geojson_y = geojson_y
        self.pen_offset_x = -0.0102
        self.pen_offset_y = -0.0088
        self.division_x = 62345.65
        self.division_y = 111139
        self.scale = 100
        self.irl_x = -0.095 
        self.irl_y = -0.075

    def calc_angle(self,start_x, start_y, end_x, end_y): #takes moves and a start pos to calculate where the last position is (continuous)
        x_dist = abs(start_x - end_x)
        y_dist = abs(start_y - end_y)
        angle = math.tan(x_dist / y_dist)
        
        return angle

    def calc_dist(self,start_x, start_y, end_x, end_y):
        x_dist = abs(start_x - end_x)
        y_dist = abs(start_y - end_y)
        sl_dist = math.sqrt(x_dist*x_dist + y_dist*y_dist)
        
        return sl_dist

    def relative_point_geo_to_real_pen(self,x_coord, y_coord):
        change_in_x = (x_coord - self.geojson_x) * self.division_x
        change_in_y = (y_coord - self.geojson_y) * self.division_y

        ## get the relative x, y movement

        x_movement = change_in_x / self.scale
        y_movement = change_in_y / self.scale

        ##return point in robot world.
        return (self.irl_x + x_movement - self.pen_offset_x), (self.irl_y + y_movement - self.pen_offset_y)

    def to_execute_list(self, lines_data):
        move_triple = []
        curr_move = [0,0,0]
        for line in lines_data:
            x_0 = self.relative_point_geo_to_real_pen(line[0][0], line[0][1])[0]
            y_0 = self.relative_point_geo_to_real_pen(line[0][0], line[0][1])[1]
            x_1 = self.relative_point_geo_to_real_pen(line[1][0], line[1][1])[0]
            y_1 = self.relative_point_geo_to_real_pen(line[1][0], line[1][1])[1]
            
            curr_move[0] = [x_0,y_0]
            curr_move[1] = self.calc_angle(x_0,y_0,x_1,y_1)
            curr_move[2] = self.calc_dist(x_0,y_0,x_1,y_1)
            
        return curr_move