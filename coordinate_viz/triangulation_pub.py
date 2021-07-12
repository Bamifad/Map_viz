#!/usr/bin/python

import rospy
import numpy as np
import std_msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class PointTriangulation(object):
    def __init__(self, node_array, triangulation_array, polygons_array_holder, polygon_keys):
        self.point_publisher = rospy.Publisher('~nodes_topic', Marker, queue_size=100)
        self.triangulation_publisher = rospy.Publisher('~triangulation_topic', Marker, queue_size=100)
        self.polygon_publisher = rospy.Publisher('~polygon_topic', Marker, queue_size=100)
        self.rate = rospy.Rate(5)
        self.pixel_arr = node_array
        self.connect_array = triangulation_array
        self.polygon_dict = polygons_array_holder
        self.polygon_keys = polygon_keys
        self.triangle_marker = Marker()  # Using a Line strip
        self.polygon_marker = Marker()  # Using a Line strip
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
        self.node_marker.scale.x = 5.0
        self.node_marker.scale.y = 5.0
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

    def pub_polygons(self):
        self.polygon_marker.header.frame_id = "/world"
        self.polygon_marker.ns = "Polygons"
        self.polygon_marker.id = 2
        self.polygon_marker.type = Marker.LINE_LIST
        self.polygon_marker.action = Marker.ADD
        self.polygon_marker.pose.orientation.w = 1.0
        self.polygon_marker.scale.x = 2.0
        self.polygon_marker.scale.y = 2.0
        self.polygon_marker.scale.z = 0.0
        self.polygon_marker.color.r = 0.0
        self.polygon_marker.color.g = 1.0
        self.polygon_marker.color.b = 0.0
        self.polygon_marker.color.a = 1.0
        self.polygon_marker.lifetime = rospy.Duration()

        # Creating list containing polygon publishing
        polygons = []
        # for polygon in self.polygon_keys:
        #     curr_polygon = self.pub_polygons(self.)

    # def create_polygon(self, polygon_points):

    def pub_triangulation(self):
        self.triangle_marker.header.frame_id = "/world"
        self.triangle_marker.ns = "Triangulation"
        self.triangle_marker.id = 1
        self.triangle_marker.type = Marker.LINE_LIST
        self.triangle_marker.action = Marker.ADD
        self.triangle_marker.pose.orientation.w = 1.0
        self.triangle_marker.scale.x = 3.0
        self.triangle_marker.scale.y = 3.0
        self.triangle_marker.scale.z = 0.0
        self.triangle_marker.color.r = 0.0
        self.triangle_marker.color.g = 0.0
        self.triangle_marker.color.b = 1.0
        self.triangle_marker.color.a = 1.0
        self.triangle_marker.lifetime = rospy.Duration()

        # Creating triangulation for nodes
        triangles = []
        for nodes in self.connect_array:
            triangle = self.triangulate(nodes)
            triangles.extend(triangle)

        self.triangle_marker.points = triangles

    def triangulate(self, node_points):
        node1_coord = self.pixel_arr[node_points[0]]
        node2_coord = self.pixel_arr[node_points[1]]
        node3_coord = self.pixel_arr[node_points[2]]

        p1 = Point()
        p1.x = node1_coord[0]
        p1.y = node1_coord[1]
        p1.z = 0

        p2 = Point()
        p2.x = node2_coord[0]
        p2.y = node2_coord[1]
        p2.z = 0

        p3 = Point()
        p3.x = node3_coord[0]
        p3.y = node3_coord[1]
        p3.z = 0

        triangle = [p1, p2, p2, p3, p1, p3]

        return triangle

    def start(self):
        while not rospy.is_shutdown():
            self.point_publisher.publish(self.node_marker)
            self.triangulation_publisher.publish(self.triangle_marker)
            # print(self.node_marker.points)
            self.rate.sleep()


def load_pixels(filename):
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


def load_polygons(filename):
    polygon_hold = dict()
    polygon_keys = []
    with open(filename, "r") as file:
        for i, line in enumerate(file):
            if i == 0:
                pass
            else:
                line = line.strip()
                line = line.split(',')
                polygon_keys.append(line[0])
                polygon_hold[line[0]] = []
                for j in range(1, len(line)):
                    polygon_hold[line[0]].append(float(line[j]))

    return polygon_keys, polygon_hold


if __name__ == "__main__":
    points_file = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/DT_Points.csv"
    connect_file = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/DT_Connectivity.csv"
    polygon_file = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/feature_shape_v1.csv"
    pnt_holder = load_pixels(points_file)
    triangle_holder = load_connections(connect_file)
    keys, polygon_holder = load_polygons(polygon_file)

    # print(keys)
    # print(len(holder))
    # print(polygon_holder['Ballpark'])
    # for i in connect_holder:
    #     print(i)
    # for ix in range(len(polygon_holder)):
    #     print(polygon_holder[ix+1])
    #     print('\n')

    rospy.init_node("pixel_nodes")
    node_obj = PointTriangulation(pnt_holder, triangle_holder, polygon_holder, keys)
    try:
        node_obj.start()
    except rospy.ROSInterruptException:
        pass
