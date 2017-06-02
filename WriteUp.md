### Writeup / README

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

In order to add obstacle and rock, I made two new function:
`invert_color` :  to quickly invert the color from the navigable terrain to produce the image channel.
`select_rock_color` : this function take in the threshold for Red, Green and Blue channel which can be used to filter out pixel that comprise the yellow color. Also, I come up with a decent threshold for each channel after playing around with the data. 
Note: Image can be viewed from the Jupyter Notebook. I have a seperate section for doing processing on the data I recorded.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
_In order to complete this function, I first declare `source` and `destination` values used for perspective transform. After that, I follow the steps below to use all the availalble helpers functions to trasnform real image to world mapping with:
1. Apply Perspective Transform
2. Apply color threshold to create 3 seperate binary images for navigable terrain, obstacle , and rock.
3. User `rover_coords` helper to translate all the binary channel to rover coordinates so it can be used for rover views.
4. Lastly, we convert these pixel coordinates from rover view to realworld view so we can map it to the world map.


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
`perception_step` - following the similar logic of the `process_image` from the notebook and adding two changes:
1. Check for pitch and roll angle to verify they are smaller than 1.2 to maintain the worldmap fidelity.
2. Need to call `to_polar_coords` on navigable terrain pixel and rock image pixel in order to obtain the needed angles to steer the Rover.

`decision step` - in this function, I was trying to make the Rover running as close to right wall as possible so I can enhance the map coverage. I also addded feature for checking if the Rover get stuck and variou methods to unstuck the Rover at different levels. Also, I added a function for checking if the rock is within the visibility of the rover. If it is then we will switch the rover the specific mode which helps to locate and pick-up the rock. After picking up the rock, the Rover aso tries to rotate back to the previous yaw angle before start picking up rock in order to continue explore the map with minimum interruption. 


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

I was running the simulator on my Macbook air with resolution: 640x480 and FASTEST graphic option. FPS is between 11-15. 
The rover does a decent job at mapping with average around 60 % and fidelity flucuating between 61% - 65% within around 300-500 seconds. 
Note: sample recording can be located in the `autonomous_data` folder
There are a lot improvements can be madde for this Rover:
1. Sometimes the rover stuck with tough obstacles to get away. We can enhance the navgiation methos to avoid these areas at all.
2. Making the Rover remember where it came from so we can have better navigation directio rather than keeps crawling along the right wall.
3. Enhance the rock navigation features to locate the correct rock angle and move to it rather than spinning around to relocate when lost sight of the rock.






![alt text][image3]