# particle_filter_localization_project

## Implementation Plan
* Joseph Henry, Charles Suarez

* We will randomly place 1000 particles on the map, varying location and orientation randomly. Our code will only place particles inside the house since that is where the robot will definitely be located. We would test our initialization by running it and looking at the visualization to see if our particles are located where we expect them to be, comparing them to the photos on the project page.

* We will move the particles according to the odemetry we recieve and introduce noise on both the velocity and orientation, the noise will be generated randomly within parameters we experiment with and find ideal. We can test the model by generating our initial filter, moving our robot and seeing how the particles have shifted position. If they have broadly but not exactly matched the robot's motion it means it works.

* We will use the in class formula for calculating the difference between the laser scan data and our knowledge of the particles surroundings since we know exactly where the particle is and we have access to the map. We would test this by manually placing particles so we know the surroundings beforehand and checking if our generated surroundings data match. We would also use this data to compare against a known position and laser scan of the robot to see if we get the expected values.

* We would calculate the sum of our particles' weight and then use this sum to divide all our weights and generate their normalized probability. To resample we will generate a number of particles equal to the probability * number of particles. (a probabilty of 0.4 particle will have 40 copies in the next round if we have 100 particles) This can be checked by hand with previously known values to see if the code is doing what we expect it to. For resampling we could study the visualization to see if we get the expected clustering. 

* We would take the average of all the particle locations and orientation to estimate our best guess as to the robot's pose. This could be very inaccurate at first but should quickly collapse into a more probable state as we update further. We could test this by creating a particle filter that surrounds a location that we know the filter should coalesce to. 

* For each variable that needs to have noise encorporated, we could develop a normal distribution of values to draw from such that the mean is the recorded value and the standard deviation can be found with experimentation over time. This would ensure that variables receive noise in a logical way that allows the most probable values get a larger share. We could test with a small filter and observe how they move in the visualization. We know to expect some variation but still some adherence to mean value generally. 

## Timetable
* initialize_particle_cloud by Sunday 4/17
* update_particles_with_motion_model and update_particle_weights_with_measurement_model by Wednesday 4/21
* resample_particles and update_estimated_robot_pose by Saturday 4/24
* Rest of time is bug fixes

## Writeup
### Objectives Description
The goal of this project is to achieve localization via a particle filter. Localization is the process by which a robot determines its approximate location on a map. It usually will move around and correlate input from its sensory data with features on the map, much like a hiker would look at their surroundings and try to correlate them with features on a paper map. 

### High-Level Description
Localization is achieved with a particle filter. The robot holds a pool of potential guesses (particles) as to where it might be. Whenever the robot moves, the robot moves each particle and then weights them based on how closely the particle's surroundings would match the robot's own sensory data. The robot then resamples the particle field with a preference for higher-weighted particles. Noise is added after resampling to ensure that slightly new guesses are always generated. In this way, the robot gradually coalesces all of its particles down to one general location on the map, which presumably is the approximate location of the robot in real space.

### Steps of Particle Filter Localization
#### Initialization of Particle Cloud
This is accomplished in `ParticleFilter.initialize_particle_cloud()`. First, the code uses the map data to determine which x,y coordinates are allowed. It loops through the full range of x,y possibilities and selects only those x,y combinations that have a data value of 0 (indicating a 0 percent chance these spaces are occupied). The code then selects 3000 of these coordinates uniformly to initialize the particle field. For each chosen coordinate, a new `Particle` object is created with its `pose` field initialized to the coordinates.

#### Movement Model
This is accomplished in `ParticleFilter.update_particles_with_motion_model()`. To move each particle, the robot calculates how far it has traveled since the last update using the Euclidean distance formula on the `ParticleFilter.odom_pose` and `ParticleFilter.odom_pose_last_motion_update` members. The robot then uses its current yaw value (orientation) and its x,y coordinates to check if it has moved forwards or backwards based on the Unit Circle. If the robot moved backwards, it multiplies the distance moved by -1. The code then loops through each particle and updates each particle's x,y coordinates as well as its yaw using some basic trigonometry. 

#### Measurement model
This is accomplished in `ParticleFilter.update_particle_weights_with_measurement_model()`. This function loops through each particle in the `particle_cloud` and runs the likelihood field for range finders algorithm. For simplicity, we used values of 0 for `xk,sense` and `yk,sense`, and we ommited `z_hit`, `z_random`, and `z_max`. In our zero-centered Gaussian function, we used a sigma of 0.3. Lastly, we used only 36 directions (increments of 10 degrees) instead of the full 360. 

#### Resampling
This is accomplished in `ParticleFilter.resample_particles()`. The code uses `random.choices()` to perform a weighted random selection of all particles. However, this function only returns references to objects rather than the objects themselves. To overcome this, the code then loops through the chosen particles and creates a deep copy for each reference to a particle in the chosen particle array. When a new particle is initialized, its x, y and yaw are slightly noised (see below). This set of new particles then becomes the new `particle_cloud`.

#### Incorporation of Noise
This is accomplished in `ParticleFilter.resample_particles()`. After a particle has been chosen in the resampled `particle_cloud`, its x and y coordinates are noised by setting them equal to a random value pulled from a normal distribution where the mean is the original x or y coordinate. We do the same for the yaw. We use a sigma of 0.2 for the x,y coordinates and 0.1 (~5 degrees) for the yaw.

#### Updating Estimated Robot Pose
This is accomplished in `ParticleFilter.update_estimated_robot_pose()`. This function simply looks at the x,y coordinates and yaw of every particle. It calculates the average of each, and then updates the `robot_estimate` pose with those aggregated values.

#### Optimization of Parameters
In our experience, we found that more cardinal directions in the range finder and fewer particles gave us better results. Thus we used 36 different directions (increments of 10 degrees) and only 3000 individual particles. We found that a sigma of 0.3 worked well for our zero-centered Gaussian function. Finally, when noising the particles we found that a sigma of 0.2 worked well for the x,y coordinates and 0.1 (~5 degrees) worked well for the yaw.

### Challenges
The hardest part for us was figuring out how to remove incorrect competing clusters of particles. The particles would often coalesce to spots on the map that were similar to the robot's true location, but were not in fact where the robot actually was. To solve this, we tightened up our sigma a bit for the zero-centered Gaussian function (we were previously trying values closer to 0.7). Another difficulty was balancing the quality (essentially the number of cardinal directions) vs quantity of particles. We overcame this with simple trial and error and found that 36 directions and 3000 particles worked well.

### Future Work
If we had more time, we would like to use values for `z_hit`, `z_random`, and `z_max` in our range finder function. It would be nice to have some kind of mechanism in place to overcome measuring error. Additionally, we would like to figure out how to tighten up the spread of our particle cloud. Currently, the middle of our cloud is usually spot-on, but the cloud is quite large. I believe this could be tightened by decreasing the sigma for our noise values, but we did not get a chance to test this. 

### Takeaways
* *Test comprehensively and thoroughly* - It's a good rule of thumb to never move onto a new stage of the project until you are certain your current code works as expected. There are always at least some ways you can test, even if that just means printing out values for manual inspection. The more time you spend on this, the smoother your transition will be.
* *Make sure you know exactly how your code works* - We had a large roadblock when resampling that revolved around how Python handles object references. We assumed that `random.choices` would return copies of any object that was selected multiple times, but it in fact only returned shallow copies. As a result, when we went to update the weight, we would continually update the same particle. This would have been overcome if we knew from the start what our chosen function returned.
* *Don't forget about radians* - Most of the functions in this assignment take in angles in radians rather than degrees. If you don't keep track of this, your filter has zero hope of working correctly. 
* *Nothing will work until everything does* - Once you've initialized the particles and are moving them correctly, the next visible benchmark for your particle filter will basically be its completion. We spent a significant amount of time fiddling with the various values and parameters before we started to get consistent localization results. I think this might apply to many projects in robotics, since most solutions will have many moving parts.
