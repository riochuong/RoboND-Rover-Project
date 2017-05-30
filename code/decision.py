import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    def is_rock_sample_visible(rock_channel):
        return (np.sum(rock_channel))
    
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if is_rock_sample_visible(Rover.vision_image[:,:,1]) \
                and Rover.search_fo_new_rock :
                print("found rock Sample. Stop")
                Rover.brake = Rover.brake_set
                Rover.steer = -5
                Rover.mode = 'locate_rock'
                Rover.last_pos = Rover.pos
                Rover.last_rock_sum = is_rock_sample_visible(Rover.vision_image[:,:,1])
            
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
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
            if len(Rover.samples_found) == (Rover.my_rock_count + 1):
                print("Found samples ", Rover.samples_found)
                Rover.mode = 'forward'
                Rover.search_fo_new_rock = False
                Rover.my_rock_count += 1
            else:
                Rover.steer = -5
                Rover.brake = Rover.brake_set
                new_rock_sum = is_rock_sample_visible(Rover.vision_image[:,:,1])
                if new_rock_sum >= Rover.last_rock_sum + 5:
                    # come closer to rock
                    Rover.steer = 0
                Rover.mode = 'come_to_rock'
                Rover.last_rock_sum = new_rock_sum

        # move clost to rock 
        elif Rover.mode == 'come_to_rock':
            if len(Rover.samples_found) == (Rover.my_rock_count + 1):
                print("Found samples ", Rover.samples_found)
                Rover.mode = 'forward'
                Rover.search_fo_new_rock = False
                Rover.my_rock_count += 1
            else:
                Rover.throttle = 0.05
                Rover.brake = 0
                new_rock_sum = is_rock_sample_visible(Rover.vision_image[:,:,1])
                if (new_rock_sum >= Rover.last_rock_sum + 10):
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                Rover.mode = 'locate_rock'
                Rover.last_rock_sum = new_rock_sum


        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # randomly steer to avoid get in to same location 
                    if Rover.steer_dir:
                        Rover.steer = -15 # Could be more clever here about which way to turn
                    else:
                        Rover.steer = 15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover

