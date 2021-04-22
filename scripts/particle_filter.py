#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random

from likelihood_field import LikelihoodField


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(particles_list):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """

    probabilities = [p.w for p in particles_list]
    n = len(particles_list)
    samples = np.random.choice(particles_list,n,replace=True,p=probabilities)
    return samples.tolist()

# copied from Class Meeting 6 starter code
def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """

    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


# helper function to get index in occupancy grid from a Particle position
def get_occupancy_grid_index(x, y):
    x_ogrid = round((x+10)*20)
    y_ogrid = round((y+10)*20)
    return y_ogrid*384 + x_ogrid


# helper function to make a pose from given x, y, and yaw values
def make_pose(x, y, yaw):
    p = Pose()
    p.position = Point()
    p.position.x = x
    p.position.y = y
    p.position.z = 0
    q = quaternion_from_euler(0.0, 0.0, yaw)
    p.orientation = Quaternion()
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and likelihood field
        self.map = OccupancyGrid()
        self.likelihood_field = LikelihoodField()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        rospy.sleep(1)
        self.initialize_particle_cloud()


        self.initialized = True



    def get_map(self, data):

        self.map = data


    def initialize_particle_cloud(self):

        num_particles = 10000 # total number of particles to generate
        cur_particles = 0 # current number of generated particles
        initial_particle_set = []

        # generate particles until total number is reached;
        # looping is necessary because some randomly generated particles
        # will be off the map or on top of an object/wall and need to be
        # discarded
        while cur_particles < num_particles:
            needed_particles = num_particles - cur_particles

            xs = np.random.uniform(-10, 9, needed_particles)
            ys = np.random.uniform(-10, 9, needed_particles)
            thetas = np.random.randint(0, 360, needed_particles)

            new_particles = []
            # only keep particles that are within open map space
            for i in range(needed_particles):
                if (self.map.data[get_occupancy_grid_index(xs[i], ys[i])] == 0):
                    new_particles.append([xs[i], ys[i], math.radians(thetas[i])])

            initial_particle_set = initial_particle_set + new_particles
            cur_particles = len(initial_particle_set)

        self.particle_cloud = []
        # create Particle objects from generated particle values
        # from Class Meeting 6 starter code
        for i in range(len(initial_particle_set)):
            p = make_pose(initial_particle_set[i][0], initial_particle_set[i][1], initial_particle_set[i][2])
            # initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)

            # append the particle to the particle cloud
            self.particle_cloud.append(new_particle)

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        sum_weights = np.sum([p.w for p in self.particle_cloud])
        for p in self.particle_cloud:
            p.w = p.w/sum_weights


    def publish_particle_cloud(self):
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):
        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        self.particle_cloud = draw_random_sample(self.particle_cloud)


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated)
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):

        # based on the particles within the particle cloud, update the robot pose estimate
        x = np.mean([p.pose.position.x for p in self.particle_cloud])
        y = np.mean([p.pose.position.y for p in self.particle_cloud])
        yaw = np.mean([get_yaw_from_pose(p.pose) for p in self.particle_cloud])
        self.robot_estimate = make_pose(x, y, yaw)

        return

    def likelihood_field_range_finder_model(self, z_t, x_t):

        # by recommendation from Yves, we are scaling the Class Meeting 6
        # four direction implementation to include 8 cardinal directions
        cardinal_directions_idxs = [0, 45, 90, 135, 180, 225, 270, 315]

        q = 1

        # implementation of likelihood field range finger algorithm from
        # Class Meeting 6 exercise; likelihood_field.py from class exercise
        for i in cardinal_directions_idxs:

            z_k_t = z_t.ranges[i]

            if z_k_t < 3.5:
                x_z_k_t = x_t.pose.position.x + z_k_t * math.cos(get_yaw_from_pose(x_t.pose) + math.radians(i))
                y_z_k_t = x_t.pose.position.y + z_k_t * math.sin(get_yaw_from_pose(x_t.pose) + math.radians(i))

                # returns NaN when out of map range
                dist = self.likelihood_field.get_closest_obstacle_distance(x_z_k_t, y_z_k_t)

                if not math.isnan(dist):
                    prob = compute_prob_zero_centered_gaussian(dist, 0.1)
                    q = q * prob

        return q


    def update_particle_weights_with_measurement_model(self, data):

        for p in self.particle_cloud:
            p.w = self.likelihood_field_range_finder_model(data, p)

        return



    def update_particles_with_motion_model(self):

        # retrieve current and old x, y, yaw values
        cur_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        cur_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        cur_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        # calculate how the robot has moved
        delta_x = cur_x - old_x
        delta_y = cur_y - old_y
        delta_yaw = cur_yaw - old_yaw

        # update each particle
        for i in range(len(self.particle_cloud)):
            p_og = self.particle_cloud[i].pose # original particle pose
            yaw_og = get_yaw_from_pose(p_og)

            # create new pose with updated x, y, yaw values
            p = make_pose(p_og.position.x + delta_x, p_og.position.y + delta_y, yaw_og + delta_yaw)

            # create new particle with weight 1.0, weight to be changed in
            # the measurement model step
            new_particle = Particle(p, 1.0)
            self.particle_cloud[i] = new_particle

        return



if __name__=="__main__":


    pf = ParticleFilter()

    rospy.spin()
