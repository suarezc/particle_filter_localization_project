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
from sklearn.neighbors import NearestNeighbors

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random, choice, choices

PARTICLE_FIELD_SIZE = 1000#0

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample():
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # TODO
    return


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w

    
    def __str__(self):
        theta = euler_from_quaternion([
            self.pose.orientation.x, 
            self.pose.orientation.y, 
            self.pose.orientation.z, 
            self.pose.orientation.w])[2]
        return ("Particle: [" + str(self.pose.position.x) + ", " + str(self.pose.position.y) + ", " + str(theta) + "]")



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

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = PARTICLE_FIELD_SIZE

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

        # The coordinates of each grid cell in the map  
        X = np.zeros((self.map.info.width*self.map.info.height, 2))

        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                X[curr, 0] = float(i)
                X[curr, 1] = float(j)
                curr += 1


        # The coordinates of each occupied grid cell in the map
        occupied = np.zeros((total_occupied, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    occupied[curr, 0] = float(i)
                    occupied[curr, 1] = float(j)
                    curr += 1
        # use super fast scikit learn nearest neighbor algorithm
        nbrs = NearestNeighbors(n_neighbors=1,
                                algorithm="ball_tree").fit(occupied)
        distances, indices = nbrs.kneighbors(X)

        self.closest_occ = np.zeros((self.map.info.width, self.map.info.height))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                self.closest_occ[i, j] = \
                    distances[curr][0]*self.map.info.resolution
                curr += 1
        self.occupied = occupied


        # intialize the particle cloud
        rospy.sleep(1)
        self.initialize_particle_cloud()


        self.initialized = True



    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        # Loop through each x,y pair
        free_coordinates = []
        curr = 0
        for x in range(self.map.info.width):
            for y in range(self.map.info.height):

                # Occupancy grids are stored in row major order
                ind = x + y * self.map.info.width
                if self.map.data[ind] == 0:
                    orientation = ((np.random.randint(360) / 180) * np.pi)
                    x_adjusted = (x * self.map.info.resolution) + self.map.info.origin.position.x
                    y_adjusted = (y * self.map.info.resolution) + self.map.info.origin.position.y
                    free_coordinates.append([float(x_adjusted), float(y_adjusted), orientation])
                    curr += 1
        
        

        self.particle_cloud = []



        # Draw randomly from above list for each particle
        for i in range(0, PARTICLE_FIELD_SIZE):
            initial_particle_set = choice(free_coordinates)
        #     initial_particle_sets = [
        #     [0.0, 0.0, 0.0],
        #     [-6.6, -3.5, np.pi],
        #     [5.8, -5.0, (np.pi / 2.0)],
        #     [-2.2, 4.5, (np.pi / 2.0)],
        #     [-3, 1, 0.0]
        # ]
            #initial_particle_set = initial_particle_sets[i]
            p = Pose()
            p.position = Point()
            p.position.x = initial_particle_set[0]
            p.position.y = initial_particle_set[1]
            p.position.z = 0
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, initial_particle_set[2])
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)

            # append the particle to the particle cloud
            self.particle_cloud.append(new_particle)

        self.normalize_particles()
        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        
        # First, calculate sum
        weight_sum = 0.0
        for p in self.particle_cloud:
            weight_sum += p.w

        # Now, renormalize with n
        for p in self.particle_cloud:
            p.w = p.w / weight_sum
        
        return



    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)
        print(len(self.particle_cloud))





    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        new_particles = []
        particle_weights = []

        for p in self.particle_cloud:
            particle_weights.append(p.w)
            #print(p.w)

        new_particles = choices(population = self.particle_cloud, k = PARTICLE_FIELD_SIZE, weights = particle_weights)
        self.particle_cloud = new_particles

        return



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

                #dibe
                self.update_particles_with_motion_model()
                #done
                self.update_particle_weights_with_measurement_model(data)

                #done
                self.normalize_particles()


                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose
                print("\n\n")



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # TODO
        return

    #From Class06 likelihood_field.py
    def get_closest_obstacle_distance(self, x, y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in
            the map.  If the (x,y) coordinate is out of the map boundaries, nan
            will be returned. """
        x_coord = (x - self.map.info.origin.position.x)/self.map.info.resolution
        y_coord = (y - self.map.info.origin.position.y)/self.map.info.resolution
        if type(x) is np.ndarray:
            x_coord = x_coord.astype(np.int)
            y_coord = y_coord.astype(np.int)
        else:
            x_coord = int(x_coord)
            y_coord = int(y_coord)

        is_valid = (x_coord >= 0) & (y_coord >= 0) & (x_coord < self.map.info.width) & (y_coord < self.map.info.height)
        if type(x) is np.ndarray:
            distances = np.float('nan')*np.ones(x_coord.shape)
            distances[is_valid] = self.closest_occ[x_coord[is_valid], y_coord[is_valid]]
            return distances
        else:
            return self.closest_occ[x_coord, y_coord] if is_valid else float('nan')
    
    def compute_prob_zero_centered_gaussian(self, dist, sd):
        """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
        c = 1.0 / (sd * math.sqrt(2 * math.pi))
        prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
        return prob
    
    def update_particle_weights_with_measurement_model(self, data):

        for p in self.particle_cloud:
            x = p.pose.position.x
            y = p.pose.position.y
            theta = get_yaw_from_pose(p.pose)

            # print(str(p))
            # print("theta: ", theta)
            q = 1
            for angle in range(0, 360, 10):
                zkt = data.ranges[angle]
                if zkt == float("inf"):
                    continue
                # print(angle)
                # print("zkt: ", zkt)
                xzkt = x + zkt * math.cos(theta + (angle * math.pi/180))
                yzkt = y + zkt * math.sin(theta + (angle * math.pi/180))
                # print("xzkt: " + str(xzkt) + " yzkt " + str(yzkt))
                dist = self.get_closest_obstacle_distance(xzkt, yzkt)
                if math.isnan(dist):
                    dist = 3.5
                # print("dist: " , dist)
                # print("prob: " ,self.compute_prob_zero_centered_gaussian(dist, 0.1))
                q = q * self.compute_prob_zero_centered_gaussian(dist, 0.1)
                # print("q is " + str(q))
            if math.isnan(q):
                q = 0.0
            p.w = q
            # if q > 0:
            #     print(str(p))
            #     print("total:" + str(q) + "\n")


        

    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y

        distance_moved = math.sqrt(((curr_x - old_x) ** 2) + ((curr_y - old_y) ** 2))

        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        x_pos = False
        if curr_x - old_x > 0:
            x_pos = True

        y_pos = False
        if curr_y - old_y > 0:
            y_pos = True

        moving_backwards = False
        working_yaw = curr_yaw % (2 * np.pi)
        if working_yaw <= np.pi / 2:
            moving_backwards = not x_pos or not y_pos
        elif working_yaw <= np.pi:
            moving_backwards = x_pos or not y_pos
        elif working_yaw <= (3 * np.pi) / 2:
            moving_backwards = x_pos or y_pos
        elif working_yaw <= 2 * np.pi:
            moving_backwards = not x_pos or y_pos
        else:
            print("ERROR: yaw is outside of range")
            print("original yaw:", curr_yaw, "working_yaw:", working_yaw, end="\n\n")
        
        if moving_backwards:
            distance_moved *= -1


        distance_turned = (old_yaw - curr_yaw) % (2 * np.pi)

        for p in self.particle_cloud:
            particle_yaw = get_yaw_from_pose(p.pose)

            p.pose.position.x += math.cos(particle_yaw) * distance_moved
            p.pose.position.y += math.sin(particle_yaw) * distance_moved

            new_yaw = (particle_yaw - distance_turned) % (2 * np.pi)
            q = quaternion_from_euler(0.0, 0.0, new_yaw)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]

        





if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









