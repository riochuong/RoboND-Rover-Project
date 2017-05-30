import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    #print ("np.pi ",np.pi)
    #print("yaw ",yaw  )
    #print("yaw type", type(yaw))
    yaw_rad = float(yaw) * np.pi / 180
    #print ("yaw_rad", yaw_rad)
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

# invert color for getting the obstacle
def invert_color (color_pix):
    color_invert = np.zeros_like(color_pix)
    color_invert[color_pix == 0] = 1 
    return color_invert

"""
    filter out rock yellow 
"""
def select_rock_color(img, r_thres, g_thres, b_thres):
    rock_color = np.zeros_like(img[:,:,0])
    thresh_required = (img[:,:,0] > r_thres[0]) \
                & (img[:,:,0] < r_thres[1]) \
                & (img[:,:,1] > g_thres[0]) \
                & (img[:,:,1] < g_thres[1]) \
                & (img[:,:,2] > b_thres[0]) \
                & (img[:,:,2] < b_thres[1]) 
    rock_color[thresh_required] = 1
    #print(thresh_required)
    return rock_color


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    img = Rover.img
    dst_size = 5
    bottom_offset = 6
    source =  np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dest = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    img_trans = perspect_transform(img, source, dest)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable_thresh = color_thresh(img_trans)
    obstacle_thresh = invert_color(navigable_thresh)
    rock_thresh = select_rock_color(img, (130,190),(100,160),(0,60))
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacle_thresh * 255
    Rover.vision_image[:,:,1] = rock_thresh * 255
    Rover.vision_image[:,:,2] = navigable_thresh * 255

    # 5) Convert map image pixel values to rover-centric coords
    navigable_xpix, navigable_ypix = rover_coords(navigable_thresh)
    obstacle_xpix, obstacle_ypix = rover_coords(obstacle_thresh)
    rock_xpix, rock_ypix = rover_coords(rock_thresh)
    
    # get rover x pos
    rover_x_pos = float(Rover.pos[0])
    rover_y_pos = float(Rover.pos[1])
    rover_yaw = float(Rover.yaw)
    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    navigable_x_world, navigable_y_world = pix_to_world(navigable_xpix, 
                                                        navigable_ypix, 
                                                        rover_x_pos, 
                                                        rover_y_pos, 
                                                        rover_yaw,
                                   Rover.worldmap.shape[0],scale)
    
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_xpix, 
                                                        obstacle_ypix, 
                                                        rover_x_pos, 
                                                        rover_y_pos, 
                                                        rover_yaw,
                                   Rover.worldmap.shape[0],scale)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, 
                                                        rock_ypix, 
                                                        rover_x_pos, 
                                                        rover_y_pos, 
                                                        rover_yaw,
                                   Rover.worldmap.shape[0],scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    dist, angles = to_polar_coords(navigable_xpix, navigable_ypix)
    Rover.nav_dists = dist
    Rover.nav_angles = angles
       
    return Rover