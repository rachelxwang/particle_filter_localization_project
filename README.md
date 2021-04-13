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
TBD
