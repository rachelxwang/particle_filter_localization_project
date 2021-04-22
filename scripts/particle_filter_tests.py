#!/usr/bin/env python3

import rospy
from particle_filter import Particle, ParticleFilter, get_yaw_from_pose, make_pose
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import numpy as np
import math

class ParticleFilterTest:

    def __init__(self):
        print("Begin testing...")
        self.mockParticleFilter = ParticleFilter()

    def test_update_particles_with_motion_model(self):
        # create mock values for old and current robot pose; turtlebot begins
        # at (-3, 1, 0) by default; need to move only a small amount to avoid
        # triggering the real particle filter localization; movement of 0.1
        # left, 0.1 up, and yaw 0.25 radians
        mockHeader = Header()
        originPoseStamped = PoseStamped(mockHeader, make_pose(-3.0, 1.0, 0.0))
        movedPoseStamped = PoseStamped(mockHeader, make_pose(-3.1, 1.1, 0.25))
        self.mockParticleFilter.odom_pose_last_motion_update = originPoseStamped
        self.mockParticleFilter.odom_pose = movedPoseStamped

        # mock first particle to (0, 0) with yaw 90 degrees so that we know
        # what value to expect after motion update; keep original weight
        weight = self.mockParticleFilter.particle_cloud[0].w
        self.mockParticleFilter.particle_cloud[0] = Particle(make_pose(0.0, 0.0, math.pi/2.0), weight)

        # call update_particles_with_motion_model() function to be tested
        self.mockParticleFilter.update_particles_with_motion_model()

        # retrieve updated first particle and assert on expected values
        test_particle = self.mockParticleFilter.particle_cloud[0]
        assert math.isclose(-0.1, test_particle.pose.position.x)
        assert math.isclose(0.1, test_particle.pose.position.y)
        assert math.isclose((math.pi/2.0)+0.25, get_yaw_from_pose(test_particle.pose))

        print("Test passed: update_particles_with_motion_model")


if __name__=="__main__":

    pft = ParticleFilterTest()
    pft.test_update_particles_with_motion_model()
