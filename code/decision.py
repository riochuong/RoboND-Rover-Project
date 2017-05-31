import numpy as np
from perception import to_polar_coords

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function


def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating
    # autonomously!

    def is_rock_sample_visible(rock_channel):
        print("rock shape ", rock_channel.shape)
        return (np.sum(rock_channel) > (3 * 255))

    def rock_pix(rock_channel):
        total_x = 0
        x_count = 0
        total_y = 0
        y_count = 0
        for x in range(0, rock_channel.shape[0]):
            for y in range(0, rock_channel.shape[1]):
                if rock_channel[x][y]:
                    total_x = x
                    x_count += 1
                    total_y = y
                    y_count += 1
        if (x_count and y_count):
            return total_x, total_y
            #return (int(total_x/x_count), int(total_y/y_count))
        else:
            return (-1, -1)

    def rescue(Rover, limit):
        if (Rover.last_pos != None):
            print("Rover last pos ", Rover.last_pos)
            dist = np.sum(np.square(np.absolute(np.subtract(Rover.last_pos,Rover.pos))))
                    # if device stuck back up
            print("dist ", dist)
            if (dist <= 0.001):
                Rover.stuck_count += 1
                print("Angles ",len(Rover.nav_angles))
                print("Rover stuck count ",Rover.stuck_count)
                Rover.last_pos = Rover.pos
                
                if Rover.stuck_count >= limit:
                    print("it's stuck")
                    Rover.brake = Rover.brake_set
                    Rover.throttle = 0
                    Rover.steer = 0
                    Rover.prev_mode = Rover.mode
                    Rover.mode = 'stuck'
                    return Rover           
            else:
                Rover.stuck_count = 0
                Rover.throttle = 0.2

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        print ("Mode: ", Rover.mode)
        # Check for Rover.mode status
        if Rover.mode == 'stuck':
            # for sure it's stop now 
            print("stuck mode")
            Rover.steer = 0
            dist = np.sum(np.square(np.absolute(np.subtract(Rover.last_pos,Rover.pos))))
            if (dist < 0.01):
                print("still stuck")
                Rover.stuck_count += 1
                if Rover.stuck_count < 150:
                    Rover.brake = 0
                    Rover.steer = 15
                    Rover.yaw = 60
                    Rover.throttle = -0.5
                elif Rover.stuck_count < 250:
                    Rover.yaw = -60
                    Rover.brake = 0
                    Rover.steer = -15
                # if still stuck maybe we can switch to stop state
                else:
                    Rover.brake = Rover.brake_set
                    Rover.stuck_count = 0
                    Rover.steer = 0
                    Rover.mode = 'stop'
            else:
                print("escapce")
                Rover.stuck_count = 0
                Rover.throttle = 0
                Rover.steer = 0
                Rover.mode = 'forward'  


        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            print("nav dist ", Rover.nav_angles.shape)
            if is_rock_sample_visible(Rover.vision_image[:,:,1]) \
                and Rover.search_fo_new_rock :
                print("found rock Sample. Stop")
                Rover.brake = Rover.brake_set
                #Rover.steer = -5
                Rover.mode = 'locate_rock'
                Rover.last_pos = Rover.pos
                Rover.last_rock_sum = is_rock_sample_visible(Rover.vision_image[:,:,1])
                Rover.throttle = 0
            
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                rescue(Rover, 100);
                # check if it's stuck 
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                Rover.search_fo_new_rock = True
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.last_pos = Rover.pos
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                    # set steer dir
                    rand = np.random.random_integers(0,1)
                    if rand:
                        Rover.steer_dir = 0
                    else:
                        Rover.steer_dir = 1

        # try to locate the rock
        elif Rover.mode == 'locate_rock':
            print("navigate rock angle")
            rescue(Rover, 400)
            if Rover.near_sample and Rover.vel == 0:
                print("start pick up")
                Rover.send_pickup = True
                Rover.mode = 'forward'
                Rover.last_rock_sum = 0
            elif Rover.near_sample:
                print("break to pickup object ")
                Rover.brake = Rover.brake_set
                Rover.throttle = 0

            else:
                # get rock angle  
                new_rock_sum = is_rock_sample_visible(Rover.vision_image[:,:,1])
                Rover.throttle = 0
                Rover.brake = 0
                if (new_rock_sum < Rover.last_rock_sum):
                    Rover.steer *= -1
                # if we lost the rock brake
                if (new_rock_sum == 0):
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                # now spin to find the rock again 
                if (Rover.last_rock_sum == 0):
                    Rover.brake = 0
                    Rover.steer = -10
                # if we found the rock pick it up
                if (new_rock_sum):
                    Rover.brake = 0
                    Rover.throttle = 0.1
                # update visibility of the rock   
                Rover.last_pos = Rover.pos 
                Rover.last_rock_sum = new_rock_sum
                
    
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            rescue(Rover, 50);
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.last_pos = Rover.pos
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward  
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.stuck_count += 1
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # randomly steer to avoid get in to same location 
                    #if Rover.steer_dir:
                    Rover.steer = -15 # Could be more clever here about which way to turn
                    Rover.last_pos = Rover.pos

                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    Rover.stuck_count = 0
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    Rover.last_pos = Rover.pos
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    Rover.last_pos = Rover.pos    
    return Rover

