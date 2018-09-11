import numpy as np
import time
import random
import math

def is_stuck(Rover):
    stuck_time_threshold = 5
    if math.fabs(Rover.vel) < 0.2:
        if Rover.stuck_start_time is None:
            Rover.stuck_start_time = time.time()
        else:
            stuck_time = time.time() - Rover.stuck_start_time
            print(stuck_time)
            return stuck_time > stuck_time_threshold
    else:
        Rover.stuck_start_time = None
    return False

def unstuck(Rover):
    #stop(Rover)
    Rover.brake = 0
    Rover.steer = np.random.choice([-15,15])
    #random = np.random.random_sample()
    stuck_time = time.time() - Rover.stuck_start_time
    if stuck_time < 5:
        Rover.throttle = Rover.throttle_set*2
        time.sleep(1)
    else:
        Rover.throttle = -Rover.throttle_set
        time.sleep(2)

def stop(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Only keep the angles pointing toward the left
    # the rover will move along the left wall
    Rover.nav_angles = np.sort(Rover.nav_angles)[-int(len(Rover.nav_angles)*0.5):]

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            if is_stuck(Rover):
                print('stuck')
                Rover.mode = 'stuck'
            else:

                if len(Rover.rock_angles) > 1 and not Rover.picking_up:

                    if Rover.near_sample: # stop
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set*2 
                    elif Rover.vel < Rover.max_vel/4: 
                        # not there yet and moving slow, accelerate
                        Rover.throttle = Rover.throttle_set/4
                        Rover.brake = 0
                    else: 
                        # too fast, slow down
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                    

                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15) 
                # Check the extent of navigable terrain
                elif len(Rover.nav_angles) >= Rover.stop_forward:

                    # If mode is forward, navigable terrain looks good 
                    # and velocity is below max, then throttle
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    # If there's a lack of navigable terrain pixels then go to 'stop' mode
                elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    stop(Rover)
                    Rover.mode = 'stop'   
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                stop(Rover)
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                
                if len(Rover.rock_angles) > 1 and not Rover.picking_up:
                    print("Approaching sample")
                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)            
                    Rover.throttle = 0 #Rover.throttle_set
                    Rover.brake = 0                    
                # Now we're stopped and we have vision data to see if there's a path forward
                elif len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
        elif Rover.mode == 'stuck':
            if Rover.vel > 0.2:
                stop(Rover)
            else:
                unstuck(Rover)
                Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

