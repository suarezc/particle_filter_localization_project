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
