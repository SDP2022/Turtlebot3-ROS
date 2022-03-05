from motors import Motors 
from time import time, sleep 

mc = Motors() 
 
motor_id = 2        # motor port
speed = 100         # forward = positive, backwards = negative 
run_time = 2        # number of seconds to run motors 

 
# Move motor with the given ID at your set speed 
mc.move_motor(motor_id,speed)      
start_time = time() 

sleep(0.5)     # avoid errors 
 
mc.stop_motors() 