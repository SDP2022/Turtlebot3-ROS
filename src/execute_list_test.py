import json
import math
from executelist_parser import ExecuteListParser


test_empty = []
test_empty_output = []

# 1,1 / (-3.198622664952684, 55.93855642320585)
# 2,2 / (-3.1970187038102464, 55.939456197362546)
# 2,1 / (-3.198622664952684, 55.939456197362546)
# 1,2 / (-3.198622664952684, 55.939456197362546)
# 1,3 / (-3.198622664952684, 55.94035597151923)

# 90 / 1.5707963267948966
# 180 / 3.141592653589793  
# 270 / -1.5707963267948966

test_0_dist = [[[-3.198622664952684, 55.93855642320585] , [-3.198622664952684, 55.9385564232058]]]
test_0_dist_output = [[[1,1], 0 , 0]]
test_1_dist = [[[-3.198622664952684, 55.93855642320585] , [-3.198622664952684, 55.939456197362546]]]
test_1_dist_output = [[[1,1], 0 , 1]]
test_2_dist =[[[-3.198622664952684, 55.93855642320585],[-3.198622664952684, 55.94035597151923]]]
test_2_dist_output = [[[1,1], 0, 2]]


test_90 = [[[-3.198622664952684, 55.93855642320585] , [-3.1970187038102464, 55.93855642320585]]]
test_90_output = [[[1,1], 1.5707963267948966 , 1]]

test_180 = [[[-3.198622664952684, 55.93855642320585] , [-3.198622664952684, 55.93765664904917]]]
test_180_output = [[[1,1], 3.141592653589793 , 1]]

test_270 = [[[-3.198622664952684, 55.93855642320585] , [-3.2002266260951213, 55.93855642320585]]]
test_270_output = [[[1,1], -1.5707963267948966 , 1]]

test_45 = [[[-3.198622664952684, 55.93855642320585] , [-3.1970187038102464, 55.939456197362546]]]
test_45_output = [[[1,1], 0.7853981633974483 , 1.4142135623730951]]

#goes to from 1,1 -> 1,0 -> 0,0 -> 1,1
test_1 =  [ [ [-3.198622664952684, 55.93855642320585], [-3.198622664952684, 55.93765664904917] ] , [ [-3.198622664952684, 55.93765664904917] , [-3.2002266260951213, 55.93765664904917] ] , [ [-3.2002266260951213, 55.93765664904917] , [-3.198622664952684, 55.93855642320585]] ]
test_1_output = [[[1,1], 3.141592653589793 , 1], [[1,0], -1.5707963267948966  , 1], [[0,0], 0.7853981633974483 , 1.4142135623730951] ]

#goes from 1,0 -> 1,2 jumps to 1.5,2 -> 0.5,2
test_2 = [[[-3.198622664952684, 55.93765664904917], [-3.198622664952684, 55.939456197362546]], [[-3.1978206843814654, 55.939456197362546], [-3.199424645523903, 55.939456197362546]]]

test_2_output = [[[1,0] , 0 , 2] , [[1.5,2 ] , -1.5707963267948966 , 1 ]]

#goes from 1,0.5 -> 1, 1.5 then 1.5,0,5 -> 1.5, 1.5 then 2,0.5 -> 2,1.5
test_3 = [[[-3.198622664952684, 55.93810653612751] , [-3.198622664952684, 55.9390063102842]], [[-3.1978206843814654, 55.93810653612751] , [-3.1978206843814654, 55.9390063102842]], [[-3.1970187038102464, 55.93810653612751] , [-3.1970187038102464, 55.9390063102842]]]
test_3_output = [[[1,0.5], 0 , 1], [[1.5,0.5], 0 , 1], [[2,0.5], 0 , 1]]


def close_enough(real_output, expected_output): #tests to 5 d. p.
    check = True

 

    if len(real_output) == 0 or len(real_output) == 0:
        return True

    for i in range(len(real_output)):

        curr_point_out = real_output[i]
        curr_point_exp = expected_output[i]
       
    
        if abs( curr_point_out[0][0] - curr_point_exp[0][0]) > 0.00001 :
            check = False

        if abs( curr_point_out[0][1] - curr_point_exp[0][1]) > 0.00001 :
            check = False

        if abs( curr_point_out[1] - curr_point_exp[1]) > 0.00001 :
            check = False

        if abs( curr_point_out[2] - curr_point_exp[2]) > 0.00001 :
            check = False

    return check


#start

if close_enough(ExecuteListParser(test_empty), test_empty_output):
    print("Test empty success")
else:
    print("Test empty fail")
    

if close_enough(ExecuteListParser(test_0_dist), test_0_dist_output):
    print("Test 0 dist and zero degrees success")
else:
    print("Test 0 dist and zero degrees fail")

if close_enough(ExecuteListParser(test_1_dist), test_1_dist_output):
    print("Test 1 dist success")
else:
    print("Test 1 dist fail")

if close_enough(ExecuteListParser(test_2_dist), test_2_dist_output):
    print("Test 2 dist success")
else:
    print("Test 2 dist fail")


if close_enough(ExecuteListParser(test_90), test_90_output):
    print("Test 90 degrees success")
else:
    print("Test 90 degrees fail")

if close_enough(ExecuteListParser(test_180), test_180_output):
    print("Test 180 degrees success")
else:
    print("Test 180 degrees fail")

if close_enough(ExecuteListParser(test_270), test_270_output):
    print("Test 270 degrees success")
else:
    print("Test 270 degrees fail")

if close_enough(ExecuteListParser(test_45), test_45_output):
    print("Test 45 degrees success")
else:
    print("Test 45 degrees fail")


if close_enough(ExecuteListParser(test_1), test_1_output):
    print("Test 1 success")
else:
    print("Test 1 fail")
    
if close_enough(ExecuteListParser(test_2), test_2_output):
    print("Test 2 success")
else:
    print("Test 2 fail")


if close_enough(ExecuteListParser(test_3), test_3_output):
    print("Test 3 success")
else:
    print("Test 3 fail")
