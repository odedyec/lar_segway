#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2
import numpy as np
import tf


class MapGenerator:
    def __init__(self):
        self.mapPublisher = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        self.ini_map = False
        self.ogMsg = OccupancyGrid()
        self.ogMsg.info.width = 400
        self.ogMsg.info.height = 400
        self.ogMsg.info.resolution = 0.05
        self.ogMsg.info.origin.position.x = -10
        self.ogMsg.info.origin.position.y = -10
        self.ogMsg.info.origin.position.z = 0
        self.ogMsg.info.origin.orientation.z = 0
        self.ogMsg.header.frame_id = "map"
        self.n = 1 / self.ogMsg.info.resolution
        self.middle = self.ogMsg.info.width / 2 - 1  # distance start in the middle of the map (should be from seg)
        self.listener = tf.TransformListener()
        self.grid = np.zeros((self.ogMsg.info.width, self.ogMsg.info.height))
        # self.grid_final = self.grid

    def callback_pc(self, pc2msg=PointCloud2()):
        # self.grid = self.grid * 0.995
        # if (rospy.Time.now() - pc2msg.header.stamp).to_sec() > 1:
        #     return
        pcmsg = PointCloud()
        pcmsg.header = pc2msg.header
        print "im here"
        for point in sensor_msgs.point_cloud2.read_points(pc2msg, skip_nans=True):
            if (point[2]) > 0.1 and abs(point[0]) > 0.1 and abs(point[1]) > 0.1:
                pt = Point32()  # define Point32()
                pt.x = point[0]
                pt.y = point[1]
                pt.z = point[2]
                pcmsg.points.append(pt)
                print "z point: {}".format(pt.z)

        # self.listener.waitForTransform("/multisense/left_camera_optical_frame", "/odom", pcmsg.header.stamp,
        #                                rospy.Duration(0.1))
        pc2 = self.listener.transformPointCloud("odom", pcmsg)
        for pt in pc2.points:
            if 0.1 < pt.z < 1.0 and -self.ogMsg.info.width / 20 < pt.y < self.ogMsg.info.width / 20 \
                    and -self.ogMsg.info.width / 20 < pt.x < self.ogMsg.info.width / 20:  # ignore floor and ceiling
                radius = 3
                self.circle(radius, pt)

        if self.ini_map:  # for reset the map
            self.grid = np.zeros((self.ogMsg.info.width, self.ogMsg.info.height))

        # for i in range(0, self.ogMsg.info.width):  # put the old obstacles in the new map
        #     for j in range(0, self.ogMsg.info.width):
        #         if self.grid[i, j] > 50:
        #             self.grid_final[i, j] = 1
        #         elif self.grid[i, j] < -50:
        #             self.grid_final[i, j] = -1
        #         else:
        #             self.grid_final[i, j] = 0

        # self.grid_final = self.grid
        self.ogMsg.data = np.reshape(self.grid, (self.ogMsg.info.width * self.ogMsg.info.height, 1))
        self.mapPublisher.publish(self.ogMsg)

    def circle(self, radius, point):
        x = int(point.x * self.n + self.middle)
        y = int(point.y * self.n + self.middle)
        self.grid[y, x] = 1
        radius_square = radius ** 2
        n = len(self.grid)
        if x - radius > 0:
            left = x - radius
        else:
            left = 0
        if x + radius + 1 < n:
            right = x + radius + 1
        else:
            right = n
        if y + radius + 1 < n:
            up = y + radius + 1
        else:
            up = n
        if y - radius > 0:
            bottom = y - radius
        else:
            bottom = 0
        for i in range(left, right):
            for j in range(bottom, up):
                if (x - i) ** 2 + (y - j) ** 2 <= radius_square and self.grid[j, i] != 1:
                    self.grid[j, i] = -1

    def callback_odom(self, odom_msg=Odometry()):  # expand map
        odom_x = -odom_msg.pose.pose.position.x  # (up will be possitive place of the system)
        odom_y = odom_msg.pose.pose.position.y  # (right possitive place of the system)
        grid2 = self.grid

        exp_map_trig = 50
        #print(abs(odom_y / self.ogMsg.info.resolution))
        #print(abs(odom_y / self.ogMsg.info.resolution))

        if (abs(odom_x / self.ogMsg.info.resolution) > ((self.ogMsg.info.width / 2 - 1) - exp_map_trig)
                or (abs(odom_y / self.ogMsg.info.resolution) > ((self.ogMsg.info.width / 2 - 1) - exp_map_trig))):
            self.ogMsg.info.origin.position.x = self.ogMsg.info.origin.position.x - 10
            self.ogMsg.info.origin.position.y = self.ogMsg.info.origin.position.y - 10
            self.ogMsg.info.width = self.ogMsg.info.width + 400
            self.ogMsg.info.height = self.ogMsg.info.height + 400

        # self.grid = np.zeros((self.ogMsg.info.width, self.ogMsg.info.height))  # create the new map size
        # self.grid_final = np.zeros((self.ogMsg.info.width, self.ogMsg.info.height))

        #print self.ogMsg.info.width
        for i in range(0, self.ogMsg.info.width - 400):  # put the old obstacles in the new map
            for j in range(0, self.ogMsg.info.width - 400):
                if grid2[i, j] != 0:
                    self.grid[i + 200, j + 200] = grid2[i, j]

    def callback_map_ini(self, initial=Bool()):
        self.ini_map = initial


if __name__ == '__main__':
    rospy.init_node('map_generator')
    mg = MapGenerator()
    rospy.Subscriber("/multisense/lidar_points2", PointCloud2, mg.callback_pc, queue_size=1)
    rospy.Subscriber("/segway_rmp_node/odom", Odometry, mg.callback_odom)
    rospy.Subscriber("/map_reset", Bool, mg.callback_map_ini)
    rospy.spin()
