import numpy as np
from perception import rover_coords
from perception import to_polar_coords

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    CRAWL_RIGHT = -15
    ESCAPE_LEFT = 15
    STUCK_DIST_THRES = 0.01
    STUCK_THRES_0 = 10
    STUCK_THRES_1 = 20
    STUCK_THRES_2 = 55
    STUCK_THRES_3 = 120
    ROCK_VISIBLE_THRESHOLD = (10*255)
    NAVIGATE_ROCK_TRIAL = 500

    def is_rock_sample_visible(rock_channel):
        rock_sum = np.sum(rock_channel)
        print("rock sum ", rock_sum)
        return rock_sum

    def take_small_negative_angle(angles):
        #angle = max(angles)
        neg_angle = [neg for neg in angles if neg < 0]
        pos_angle = [pos for pos in angles if pos >= 0]
        if len(neg_angle) > (len(pos_angle)/5):
            print("neg angle ", np.mean(neg_angle))
            return np.clip(np.min(neg_angle)*180/np.pi,-10,0)
        else:
            return np.clip(np.mean(pos_angle) * 180/np.pi,0,15)

    # check if we are stuck
    def is_stuck (Rover, thres_0, thres_1, thres_2, thres_3):
        if Rover.last_pos:
            dist = np.sum(np.square(np.subtract(Rover.pos, Rover.last_pos)))
            if (dist < STUCK_DIST_THRES):
                Rover.stuck_count += 1
            else:
                Rover.stuck_count = 0
            print("dist ", dist)
            print("stuck count ", Rover.stuck_count)
            # try to steer in the open area
            if Rover.stuck_count >= thres_0 \
                    and Rover.stuck_count < thres_1: 
                Rover.throttle = 0
                if len(Rover.nav_angles):
                    Rover.steer = np.clip(max(Rover.nav_angles * 180/np.pi) * 100, -15,15)
                else:
                    Rover.steer = 15
                return True
            # start moving 
            elif Rover.stuck_count >= thres_1 and Rover.stuck_count < thres_2:
                Rover.throttle = 1.0
                return True
            # try to back up here 
            elif Rover.stuck_count > thres_2 and Rover.stuck_count <= thres_3:
                Rover.throttle = -0.5
                Rover.steer *= (-1)
                return True
            # try again 
            elif Rover.stuck_count > thres_3 :
                Rover.stuck_count = thres_0
                return True
           
            return False



    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        print("Rover mode ", Rover.mode)
        # Check for Rover.mode status
        # looking for the rock
        if Rover.mode == 'navigate_rock':
            Rover.nav_rock_trial += 1
            new_rock_sum = is_rock_sample_visible(Rover.vision_image[:,:,1])
            if (is_stuck(Rover, 150, 200, 300, 400)):
                    print("stuck try to escape")
            else:
                Rover.stuck_count += 1
                
                Rover.brake = 0
                # send pick up
                if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
                    print("picking up")
                    Rover.send_pickup = True
                    Rover.mode = 'rotate_back_to_yaw'
                    Rover.brake = 0
                    Rover.last_rock_sum = 0
                elif Rover.near_sample:
                    print("near sample ")
                    Rover.brake = Rover.brake_set
                elif new_rock_sum == 0 :
                    print("scan again")
                    if (Rover.vel != 0):
                        Rover.brake = Rover.brake_set
                    else:
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = 15
                elif new_rock_sum >= (Rover.last_rock_sum - 4*255):
                    # come closer
                    print("try to come closer ")
                    Rover.brake = 0
                    Rover.throttle = 0.3
                    Rover.steer = np.clip(np.min(Rover.rock_angles) * 180 / np.pi,-15,15)
                # we need to find where the rock is 
                elif new_rock_sum < Rover.last_rock_sum:
                    print("scan backward")
                    Rover.brake = 0
                    Rover.throttle = 0
                    Rover.steer *= -1
            #  
            if (Rover.nav_rock_trial >= NAVIGATE_ROCK_TRIAL):
                Rover.nav_rock_trial = 0
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.mode = 'rotate_back_to_yaw'

            Rover.last_rock_sum = new_rock_sum

        # find back the current direction
        elif Rover.mode == 'rotate_back_to_yaw':
            Rover.yaw_spin_count += 1
            if (is_stuck(Rover, 50, 100, 150, 200)):
                print ("stuck in rotate back to yaw")
            else:
                print("rotate back to prev yaw")
                if Rover.yaw_spin_count < 50:
                    Rover.brake = 0
                    Rover.throttle = 0
                    Rover.steer = -15
                elif Rover.yaw_spin_count < 100:
                    Rover.brake = 0
                    Rover.throttle = 0
                    Rover.steer = 15
                elif Rover.yaw_spin_count < 150:
                    Rover.brake = 0
                    Rover.steer = 0
                    Rover.throttle = -0.5
                else:
                    Rover.mode = 'forward'

                print('prev yaw ', Rover.prev_yaw, 'current yaw', Rover.yaw, 'diff')
                if (Rover.prev_yaw):
                    yaw_diff = np.abs(Rover.yaw - Rover.prev_yaw)
                    if (yaw_diff <= 10):
                        Rover.yaw_spin_count = 0
                        Rover.mode = 'forward'


        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward: 
                #if we found rock switch to navigate rock mode
                new_rock_sum = is_rock_sample_visible(Rover.vision_image[:,:,1])
                if new_rock_sum > ROCK_VISIBLE_THRESHOLD:
                    print("rock is visible")
                    Rover.throttle = 0.2
                    Rover.steer = 0
                    Rover.stuck_count = 0
                    if not (new_rock_sum >= (Rover.last_rock_sum - 3*255)):
                        Rover.prev_yaw = Rover.yaw
                        Rover.brake = Rover.brake_set
                    elif (Rover.vel == 0) or (Rover.samples_found) :
                        #Rover.brake = Rover.brake_set
                        Rover.mode = 'navigate_rock'
                    else: # steer to the rock
                        Rover.steer = np.clip(np.min(Rover.rock_angles) * 180 / np.pi,-15,15)

                    Rover.last_rock_sum = new_rock_sum
                    return Rover


                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    if Rover.samples_found >= 2:
                        Rover.throttle = 0.5
                    elif Rover.samples_found >= 1:
                        Rover.throttle = 0.3
                    else:
                        Rover.throttle = 0.2
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                if (is_stuck(Rover, STUCK_THRES_0,STUCK_THRES_1, STUCK_THRES_2, STUCK_THRES_3)):
                    print("stuck try to escape")
                    #Rover.steer = ESCAPE_LEFT
                else:
                    nav_angles = take_small_negative_angle(Rover.nav_angles)
                    print("nav angles ", nav_angles )
                    Rover.steer = nav_angles
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

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
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
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
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
         Rover.send_pickup = True
    
    Rover.last_pos = Rover.pos

    return Rover
