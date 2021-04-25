# Particle Filter Localization Project

## Implementation Plan

**Team Members**: Sydney Jenkins and Rachel Wang

* `initialize_particle_cloud`: We will initialize the particle cloud by randomly selecting 10,000 particle positions (each represented by a Pose message from geometry_msgs, specifically the x and y values of the Point and the yaw of the Orientation) with uniform distribution over the spatial constraints provided by the map. It appears from the project documentation that by publishing a particle cloud, we will be able to visually see the particles represented by red dots on the map, so we will test this component by initializing and publishing the particle cloud and confirming visually that the particles appear to be uniformly distributed over the map.

* `update_particles_with_motion_model`: We will update the position of the particles by changing the x, y, and yaw values of each particle by the amount that the x, y, and yaw value changed between the `odom_pose_last_motion_update` and the current `odom_pose`. To test this component, we will write a unit test for the `update_particles_with_motion_model()` function that manually sets some mock `odom_pose_last_motion_update` and `odom_pose` values and checks that the particles are updated with the expected values.

* `update_particle_weights_with_measurement_model`: We will compute the importance weights of each particle by using the likelihood fields for range finders measurement model that we will complete in Class Meeting 06. As we have not yet covered this material in class, we will determine how we will test this component after the lecture on measurement models for range finder.

* `normalize_particles` and `resample_particles`: We will normalize the particles’ importance weights by dividing each weight by the total sum of all weights. We will resample the particles by implementing the given `draw_random_sample()` function using either `numpy.random.random_sample` or `numpy.random.choice`. `random_sample` is recommended by the project starter code, however it samples from a continuous uniform distribution over an interval [a, b) where b > a, so we would need to implement a way to take the weights into account. `choice` takes in two arrays, one containing the elements that the sample is taken from and the other with the probability associated with each element, so we would not have to implement the weights as probabilities. We will write a unit test to check that the weights of the normalized particles sum to 1 and we will check the resampling visually similar to the `initialize_particle_cloud` step.

* `update_estimated_robot_pose`: We will update the estimated pose of the robot by taking the average value of the re-sampled particles. We will write a unit test to ensure that the `robot_estimate` is indeed updated to reflect the average of a given mock particle cloud.

* Incorporating Noise: We will add noise from a Gaussian in the measurement model step.

**Timeline**
* Friday 4/16: Finish `initialize_particle cloud`
* Monday 4/19: Finish `update_particles_with_motion_model` and `update_particle_weights_with_measurement_model`
* Thursday 4/22: Finish `normalize_particles` and `resample_particles`
* Saturday 4/24: Finish `update_estimated_robot_pose`
* Sunday 4/25: Finish adding noise to model
* Monday 4/26: Project Due 11am CT

## Writeup

![particle filter](./particle_filter.gif)

### Objectives Description
The goal of this project is to implement a particle filter algorithm to solve the problem of robot localization on the given map. Using the Monte Carlo Localization algorithm, the robot will determine where on the map it believes itself to be.

### High-Level Description
We solved the problem of robot localization using several key steps: initialization of a particle cloud representing different possible positions of the robot, a movement model for updating particles based on the robot's movement, a measurement model for computing weights for each particle, resampling for updating the particle cloud based on the calculated weights, noise for counter-acting the effects of the Gazebo noise, a method for updating the robot's predicted location based on the current particle cloud, and optimization of parameters to improve the accuracy and efficiency of the algorithm. The final algorithm converges on a value for the robot location by repeatedly updating the particle cloud as it moves through its environment and re-calculating the associated weights. This allows it to identify which particles correspond to locations that are more likely to represent the robot's true location, and allows us to re-sample the cloud to obtain a more accurate particle cloud. The prediction location, as determined by the mean of the particles' locations, therefore converges on the true value with each iteration.

### Main Steps of the Particle Filter Localization
**1. Initialization of particle cloud**

  Code Location: `initialize_particle_cloud()` method within the `ParticleFilter` class

  Code Description: We keep track of how many particles have already been generated, then use `np.random.uniform` to generate as many particles as are still needed to reach 10,000 total particles. For each newly generated particle, we add the particle to the particle set if it is within open map space and discard it otherwise. We repeat this process until 10,000 good (i.e. within open map space) particles have been generated.

**2. Movement model**

  Code Location: `update_particles_with_motion_model()` method within `ParticleFilter`

  Code Description: First we retrieve the x, y, and yaw values of the robot from before and after the movement. Then we calculate the differences to get Δx, Δy, and Δyaw. Then for each particle in the cloud, we create a new Pose object with x, y, and yaw values changed by the same amount that the robot moved, i.e. by the Δ values. This function also includes normalization (see below).

**3. Measurement model**

  Code Location: `update_particle_weights_with_measurement_model()` method within `ParticleFilter`

  Code Description: For this part I wrote an additional function `likelihood_field_range_finder_model()`. For each of the eight cardinal/ordinal directions (N, S, E, W, NW, NE, SW, SE), we get the scan data in that direction. If the scan is within range, then we set `x_z_k_t` and `y_z_k_t` according to the likelihood field range finger algorithm from class. We calculate the distance to the obstacle closest to (`x_z_k_t`, `y_z_k_t`) using the `get_closest_obstacle_distance()` function, which was given in the `LikelihoodField` class in the Class Meeting 06 starter code. If the distance is valid, then we compute the probability (likelihood) of observation using the `compute_prob_zero_centered_gaussian()`, which was also given in the Class Meeting 06 starter code.

**4. Resampling**

  Code Location: within `resample_particles` and `draw_random_sample` methods within `ParticleFilter`

  Code Description: The `resample_particles` function calls the `draw_random_sample` function using the current particle cloud as input. It then assigns the output to be the new particle cloud. The `draw_random_sample` function resamples the particles within a particle cloud, returning a new set of n particles from a given cloud of n particles. It uses each particle's weight as its probability for being resampled. It then uses `np.random.choice` to generate a random sample of the particle cloud, using the given probabilities.

**5. Incorporation of noise**

  Code Location: within `update_particles_with_motion_model()` method within `ParticleFilter`

  Code Description: We add some noise (from `np.random.normal`) to the x, y, and yaw values for each particle in the particle cloud while its movement is being updated.

**6. Updating estimated robot pose**

  Code Location: within `update_particles_with_motion_model` method within `ParticleFilter`

  Code Description: We update each particle's location information using data about the robot's motion. We first calculate the change in each coordinate using the robot's odometer data. We then apply the change in each coordinate i to the ith coordinate for each particle by iterating over the particles in the particle cloud. We add noise to each variable by sampling a random number from a Gaussian distribution. We then create a new particle with the updated coordinates and a weight of 1.0 and add it to the particle cloud, replacing the old particle.

**7. Optimization of parameters**

  Code Location: this step was accomplished by re-running the code using different variables, and so does not directly correspond to a given section of the code. We optimized the number of particles, the standard deviation for the noise distribution, and the standard deviation for calculating weights.

  Code Description: We optimized each parameter by running the code with several different values of the parameter. We used 5,000 particles because we found that this struck a good balance between accuracy and efficiency, as having significantly fewer particles made our algorithm less accurate and having significantly more particles introduced a larger lag time. Additionally, we used a standard deviation of 0.2 for the noise distribution because we found that this appeared to be on the same scale as the Gazebo noise. If it was smaller, the robot would not be able to compensate for all of the noise, and if it was much larger, it would be over-compensating. Finally, we used a standard deviation of 0.1 to obtain the weights, as this was suggested in the class exercise and we found that it led to good overall performance.

### Challenges

One challenge we faced was that debugging was very difficult. We found that the terminal would give the same error message of `bad callback: <bound method ParticleFilter.robot_scan_received of <__main__.ParticleFilter object at 0x7fd1795bfaf0>>` for a wide variety of bugs, with little or no indication of where in the code the error was stemming from. We overcame this by running and testing the code in smaller portions (including writing tests for single functions and isolating different parts of the core logic within the `robot_scan_received` function). Another challenge we faced was handling the particle pose position coordinate grid and the map occupancy grid. We found converting points between the two (particularly with the row-major ordering) conceptually confusing. We overcame this challenge by drawing out diagrams to help us visualize how the map and grid systems worked.

### Future Work

If we had more time to improve our particle filter localization project, we could implement different variations of the code to determine the best approach. For instance, we could try different noise distributions other than a Gaussian, such as a uniform distribution. Additionally, we could resample particles using some function of their weight, rather than directly using their weights for the sampling probability. We might want to emphasize particles with larger weights, and so use an exponential function. We might also want to ignore particles with negligible weights, and apply a cut-off for which particles are included in the resampling. This might help the algorithm to converge on the robot's location quicker. Finally, we could explore different ways of calculating the robot's location from the particle locations, as taking the mean of each coordinate direction may not be the optimal technique. We could, for instance, use a clustering algorithm to identify different hotspots and then compute the center of the largest cluster. This would be especially helpful for getting accurate locations before the algorithm has converged on a value.

### Takeaways

- One key takeaway from this project is that Zoom screen sharing is a very good tool for working on robot programming in pairs. We found it fairly effective to be able to see the same Gazebo and RViz windows to talk through solutions even while working remotely.
- An additional takeaway was that making a plan ahead of time for who would complete which tasks by which dates was helpful for communication purposes. This allowed us to hold each other responsible and to stay on track to finish the assignment on time.
