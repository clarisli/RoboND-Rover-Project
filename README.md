## Project: Search and Sample Return

![alt text][image13]

**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on perception and decision function until the rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 
[image4]: ./images/threshed.png
[image5]: ./images/warped.png
[image6]: ./images/warp_threshold_map.png
[image7]: ./images/threshed2.png
[image8]: ./images/image_space.png
[image9]: ./images/world.png
[image10]: ./images/rover_world.png
[image11]: ./images/rover_world2.png
[image12]: ./images/rotation_matrix.png
[image13]: ./images/result.png

### Setup
#### Python
* Download the simulator: [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](	https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip), or [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip).
* Setup python environment: [RoboND-Python-Starterkit](https://github.com/ryan-keenan/RoboND-Python-Starterkit)
* Start the simulator and run `drive_rover.py` scrpt

```
source activate RoboND
python drive_rover.py
```


---

### Notebook Analysis

#### 1. Color Thresholding

![alt text][image4]

I modified the `color_thresh()` function to accept both lower and upper thresholds: 

```
between_thresh = (img[:,:,0] >= lower_thresh[0]) \
                & (img[:,:,1] >= lower_thresh[1]) \
                & (img[:,:,2] >= lower_thresh[2]) \
                & (img[:,:,0] < upper_thresh[0]) \
                & (img[:,:,1] < upper_thresh[1]) \
                & (img[:,:,2] < upper_thresh[2])
```

I applied it to the warped image to identify following:

* Navigable terrain: everything above `(160, 160, 160)`
* Obstacles: everything below `(160, 160, 160)`
* Rock: between `(110, 110, 0)` and `(255, 255, 50)`

I did this in cell 6 of `Rover_Project_Test_Notebook.ipynb`.


#### 2. Map the Environment

Goal: identify navigable terrain, obstacles and rock samples from the rover's camera images. 

![alt text][image6]

##### Step 1: Perspective Transform

![alt text][image5]

The goal of this step was to obtain a top-down view of the world from the rover camera's field of view. 

I mapped each grid in the source(camera) to a 10x10 pixel square in the destination(top-down map), each grid cell represents 1 square meter. I first defined 4 source points(the 4 corners of a grid cell in the camera image above), then defined 4 destination points - in the same order as the source - to place the warped 10x10px square at the center bottom of the destination. 


```
dst_size = 5 
bottom_offset = 6
source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
```

I used OpenCV functions `cv2.getPerspectiveTransform()` and `cv2.warpPerspective()` to implement perspective transform. I did this in the function `perspect_transform()` at cell 5 of  `Rover_Project_Test_Notebook.ipynb`.

##### Step 2: Color Thresholding

I used `color_thresh()` to identify navigable terrain, obstacles, and rock in the warped image.

##### Step 3: Convert to rover-centric coordinates

The camera is at `(x, y) = (160, 160)` in the top-down thresholded images. The goal here was to convert it to a coordinate system that is fixed with respect to the robot where the camera is at `(x, y) = (0, 0)`.

The thresholded image has its origin `(0, 0)` at the upper left:
![alt text][image7]

I first used `numpy.nonzero` to get the pixel positions in image space. Notice that now the origin `(0 ,0)` is at the lower left:
![alt text][image8]

In addition to converting it to the rover-centric coordinates system, I also swapped x and y axes so that they are consistent with the world coordinate system:
![alt text][image9]

I did this in the function `rover_coords()` in cell 7, and converted the thresholded images in `process_image()`.

##### Step 4: Convert to world coordinates

![alt text][image10]

The goal was to map those points in the rover-centric coordinates to the world coordinates. It's a two steps process:

1. **Rotate** the rover-centric coordinates so that x and y axes are parallel to the world space.
2. **Translate** the rotated positions by the x and y position values given by the rover's location in the world.

![alt text][image11]

I used this rotation matrix in the rotate step:

![alt text][image12]

I did the rotation in the function `rotate_pix()` in cell 7 of the notebook.

Then, in the translation step, I added the x and y components of the rover's position to the rotated x and y values calculated above. A scale factor of 10 was used in this calculation, because in the rover space each pixel represents 0.1 x 0.1 m, and in the world map each pixel represents 1 x 1 m. I did the translation in the function `translate_pix()` in cell 7 of the notebook.

Finally, I combined rotation and translation in the function `pix_to_world()` in cell 7 of the notebook.

##### Step 4: Show on worldmap
After converting the navigable terrain, obstacles, and rock pixels to the world space. I then added each to different RGB channels. I assigned obstacle to red, navigable to blue, and rock to green.

I did this in the `process_image()` function in the notebook. 

##### Result

I applied `process_image()` on test data and created a video [here](./output/test_mapping.mp4).


### Autonomous Navigation and Mapping

#### 1. Perception and Decision

##### Perception Step

Very similar to `process_image()` in the notebook, but with following differences:

* Show the thresholded navigable terrain, obstacles, and rock on the left side of screen. I assigned them to the rover's vision image in lines 108 to 110 of `perception.py`.
* I clipped out navigable terrain and obstacles that are out a certain range, because they tend to be more inaccurate. In this way I increased the fidelity. Line 113 to 123 in `perception.py`.
* Only update the worldmap when the rover is flat, i.e., roll and pitch are near zero, because the perspective transform is only valid under this condition. Lines 141 to 142 in `perception.py`.

I did this in the function `perception_step()` in `perception.py`.

##### Decision Step

To increase the % mapped and find more rocks(the rocks always appear near the walls), I made the rover a "wall crawler" that always keeping a wall on its left. With the navigable angles pointing to the right removed, the rover moved along the left wall all the way around the worldmap. 

I did this in line 45 of `decision.py`.

If the rover stopped moving for a while, it might be stucked. I checke whether the rover's stucked in the function called `is_stuck()`, in lines 6 to 17 of `decision.py`. 

If it gets stucked, the rover would switch to the 'stuck' mode, and try to make turns or move backward to get out. I did this in the function `unstuck()` in lines 19 to 28 of `decision.py`.

If there's a rock, the rover would move toward it and adjust its speed according to the distance, and then it tries to stop and pick the rock up. I did this in lines 56 to 71 in `decision.py`.

I did this in the function `decision_step()` in `decision.py`.

In addition, I've also increased the maximum velocity(max_vel) and acceleration(throttle_set) and brake setting(brake_set) to minimize total time.

#### 2. Results

![alt text][image13]

The rover mapped 98.67% of the environment with 72.0% fidelity against the ground truth, located 6 rocks and collected 5 within 566.8s.

I might improve the the project in following areas:

* The rover could get stucked in turning large circles around open-field. If the rover keeps turning in the same direction, it might be looping and I can break it by making the rover to turn in the opposite direction.
* Avoid re-visiting the explored region.
* The rover could get stucked if there's obstacles between the rock and itself.
* Return the rover to its home position after collected all rocks.

Simulator settings used: screen resolution 640 x 480, graphics quality Good, FPS 2

