#!/usr/bin/python

import rospy
import numpy as np
import std_msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Polygon, PolygonStamped, Point32
from jsk_recognition_msgs.msg import PolygonArray


# class PixPoint(object):
#     def __init__(self, node, x, y):
#         self.node_index = node
#         self.x_loc = float(x)
#         self.y_loc = float(y)
#         self.dict = {}


class PointTriangulation(object):
    def __init__(self, pixel_array, connection_array):
        self.point_publisher = rospy.Publisher('~nodes_topic', Marker, queue_size=100)
        self.connect_publisher = rospy.Publisher('~triangulation_topic', PolygonArray, queue_size=100)
        self.rate = rospy.Rate(5)
        self.pixel_arr = pixel_array
        self.connect_array = connection_array
        self.poly_array = PolygonArray()
        self.node_marker = Marker()
        self.pub_nodes()
        self.pub_triangulation()

    def pub_nodes(self):
        self.node_marker.header.frame_id = "/world"
        self.node_marker.header.stamp = rospy.Time.now()
        self.node_marker.ns = "nodes"
        self.node_marker.id = 0
        self.node_marker.type = Marker.POINTS
        self.node_marker.action = Marker.ADD
        self.node_marker.pose.orientation.w = 1.0
        self.node_marker.scale.x = 100.0
        self.node_marker.scale.y = 100.0
        self.node_marker.scale.z = 0.0
        self.node_marker.color.g = 0.0
        self.node_marker.color.r = 1.0
        self.node_marker.color.b = 0.0
        self.node_marker.color.a = 1.0
        self.node_marker.lifetime = rospy.Duration()

        # To add points from pixel_array to marker object
        pixel_point_hold = []
        for i in range(len(self.pixel_arr)):
            p = Point()
            p.x = self.pixel_arr[i+1][0]  # x location for i+1 key
            p.y = self.pixel_arr[i+1][1]  # y location for i+1 key
            p.z = 0
            pixel_point_hold.append(p)

        self.node_marker.points = pixel_point_hold

    def pub_triangulation(self):
        self.poly_array.header.frame_id = "/world"
        self.poly_array.header.stamp = rospy.Time.now()
        self.poly_array.polygons = []
        self.poly_array.labels = []
        self.poly_array.likelihood = []
        delauney_tri = self.triangulate(self.poly_array.header, self.connect_array[0])
        self.poly_array.polygons.append(delauney_tri)
        self.poly_array.labels.append(1)
        self.poly_array.likelihood.append(np.random.ranf())
        # print(self.poly_array.polygons)

        # for i, triangle in enumerate(self.connect_array):
        #     delauney_tri = self.triangulate(self.poly_array.header, triangle)
        #     self.poly_array.polygons.append(delauney_tri)
        #     self.poly_array.labels.append(i)
        #     self.poly_array.likelihood.append(np.random.ranf())

    def triangulate(self, header, pixel_points):
        p = PolygonStamped()
        p.header = header
        node1_coord = self.pixel_arr[pixel_points[0]]
        node2_coord = self.pixel_arr[pixel_points[1]]
        node3_coord = self.pixel_arr[pixel_points[2]]

        # p.polygon.points = [Point(x=node1_coord[0], y=node1_coord[1]),
        #                     Point(x=node2_coord[0], y=node2_coord[1]),
        #                     Point(x=node3_coord[0], y=node3_coord[1])]
        p.polygon.points = [Point(x=1000, y=1),
                            Point(x=400, y=4),
                            Point(x=1, y=3)]
        return p

    def start(self):
        while not rospy.is_shutdown():
            self.point_publisher.publish(self.node_marker)
            self.connect_publisher.publish(self.poly_array)
            # print(self.node_marker.points)
            self.rate.sleep()


def load_pixels(filename):
    # pixel_holder = []
    # with open(filename, "r") as file:
    #     for i, line in enumerate(file):
    #         line = line.strip()
    #         curr_line = line.split(',')
    #         p = PixPoint(i+1, curr_line[0], curr_line[1])
    #         pixel_holder.append(p)
    pixel_hold = dict()
    with open(filename, "r") as file:
        for i, line in enumerate(file):
            line = line.strip()
            curr_line = line.split(',')
            pixel_hold[i+1] = [float(curr_line[0]), float(curr_line[1])]

    return pixel_hold


def load_connections(filename):
    connection_holder = []
    with open(filename, "r") as file:
        for line in file:
            line = line.strip()
            curr_line = line.split(',')
            node_list = [int(curr_line[0]), int(curr_line[1]), int(curr_line[2])]
            connection_holder.append(node_list)

    return connection_holder


if __name__ == "__main__":
    points_file = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/DT_Points.csv"
    connect_file = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/DT_Connectivity.csv"
    pnt_holder = load_pixels(points_file)
    connect_holder = load_connections(connect_file)

    # print(len(holder))
    # for i in connect_holder:
    #     print(i)
    # for ix in range(len(pnt_holder)):
    #     print(ix+1, pnt_holder[ix+1])
    rospy.init_node("pixel_nodes")
    node_obj = PointTriangulation(pnt_holder, connect_holder)
    try:
        node_obj.start()
    except rospy.ROSInterruptException:
        pass
