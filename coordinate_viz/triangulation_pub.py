#!/usr/bin/python

import rospy
import std_msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class GeoRef(object):
    def __init__(self, filename):
        self.dx = 0
        self.dy = 0
        self.topleft_x = 0
        self.topleft_y = 0
        self.get_reference(filename)

    def get_reference(self, filename):
        geo_ref_list = []
        with open(filename, "r") as file:
            for line in file:
                line = line.strip()
                geo_ref_list.append(float(line))

        self.dx = geo_ref_list[0]
        self.dy = geo_ref_list[3]
        self.topleft_x = geo_ref_list[4]
        self.topleft_y = geo_ref_list[5]

    def conv_x_coord_to_pix(self, coordinate):
        x_pix = (coordinate - self.topleft_x)/self.dx
        return x_pix

    def conv_y_coord_to_pix(self, coordinate):
        y_pix = (coordinate - self.topleft_y)/self.dy
        return y_pix


class PointTriangulation(object):
    def __init__(self, node_array, triangulation_array, polygons_array_holder, polygon_keys, frame="/map"):
        self.point_publisher = rospy.Publisher('~nodes_topic', Marker, queue_size=100)
        self.triangulation_publisher = rospy.Publisher('~triangulation_topic', Marker, queue_size=100)
        self.polygon_publisher = rospy.Publisher('~polygon_topic', Marker, queue_size=100)
        self.rate = rospy.Rate(5)
        self.frame = frame
        self.pixel_arr = node_array
        self.connect_array = triangulation_array
        self.polygon_dict = polygons_array_holder
        self.polygon_keys = polygon_keys
        self.triangle_marker = Marker()  # Using a Line strip
        self.polygon_marker = Marker()  # Using a Line strip
        self.node_marker = Marker()
        self.pub_nodes()
        self.pub_triangulation()
        self.pub_polygons()

    def pub_nodes(self):
        self.node_marker.header.frame_id = "/map"
        self.node_marker.header.stamp = rospy.Time.now()
        self.node_marker.ns = "nodes"
        self.node_marker.id = 0
        self.node_marker.type = Marker.POINTS
        self.node_marker.action = Marker.ADD
        self.node_marker.pose.orientation.w = 1.0
        self.node_marker.scale.x = 1.0
        self.node_marker.scale.y = 1.0
        self.node_marker.scale.z = 1.0
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
            p.z = 4
            pixel_point_hold.append(p)

        self.node_marker.points = pixel_point_hold

    def pub_polygons(self):
        self.polygon_marker.header.frame_id = "/map"
        self.polygon_marker.ns = "Polygons"
        self.polygon_marker.id = 2
        self.polygon_marker.type = Marker.LINE_LIST
        self.polygon_marker.action = Marker.ADD
        self.polygon_marker.pose.orientation.w = 1.0
        self.polygon_marker.scale.x = 1.0
        self.polygon_marker.scale.y = 1.0
        self.polygon_marker.scale.z = 1.0
        self.polygon_marker.color.r = 0.5
        self.polygon_marker.color.g = 1.0
        self.polygon_marker.color.b = 0.5
        self.polygon_marker.color.a = 1.0
        self.polygon_marker.lifetime = rospy.Duration()

        # Creating list containing polygon publishing
        polygons_pnts = []
        for polygon in self.polygon_keys:
            curr_polygon = create_polygon(self.polygon_dict[polygon])
            polygons_pnts.extend(curr_polygon)

        self.polygon_marker.points = polygons_pnts

    def pub_triangulation(self):
        self.triangle_marker.header.frame_id = "/map"
        self.triangle_marker.ns = "Triangulation"
        self.triangle_marker.id = 1
        self.triangle_marker.type = Marker.LINE_LIST
        self.triangle_marker.action = Marker.ADD
        self.triangle_marker.pose.orientation.w = 1.0
        self.triangle_marker.scale.x = 1.0
        self.triangle_marker.scale.y = 1.0
        self.triangle_marker.scale.z = 1.0
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
            self.polygon_publisher.publish(self.polygon_marker)
            # print(self.node_marker.points)
            self.rate.sleep()


def create_polygon(polygon_points):
    polygon_list = []
    for i in range(len(polygon_points)):
        if i < len(polygon_points)-1:
            p1 = Point()
            p1.x = polygon_points[i][0]
            p1.y = polygon_points[i][1]
            p1.z = 0

            p2 = Point()
            p2.x = polygon_points[i+1][0]
            p2.y = polygon_points[i+1][1]
            p2.z = 0

            pair = [p1,p2]
            polygon_list.extend(pair)

        else:  # Connecting the last node with the first node to close the polygon
            p1 = Point()
            p1.x = polygon_points[i][0]
            p1.y = polygon_points[i][1]
            p1.z = 0

            p2 = Point()
            p2.x = polygon_points[0][0]
            p2.y = polygon_points[0][1]
            p2.z = 0

            pair = [p1, p2]
            polygon_list.extend(pair)

    return polygon_list


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


def load_polygons(filename, geo_ref):
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
                for j in range(1, len(line), 2):
                    # convert from UTM coordinates to pixel coordinates
                    x = geo_ref.conv_x_coord_to_pix(float(line[j]))
                    y = geo_ref.conv_y_coord_to_pix(float(line[j+1]))
                    polygon_hold[line[0]].append([x, y])

    return polygon_keys, polygon_hold


if __name__ == "__main__":
    points_file = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/DT_Points.csv"
    connect_file = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/DT_Connectivity.csv"
    polygon_file = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/feature_shape_v1.csv"
    geo_reference_file = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/test1.pgw"
    pnt_holder = load_pixels(points_file)
    triangle_holder = load_connections(connect_file)
    coordinate_ref = GeoRef(geo_reference_file)
    keys, polygon_holder = load_polygons(polygon_file, coordinate_ref)

    # print(polygon_holder["Ballpark"])
    # print(len(holder))
    # print(polygon_holder['Ballpark'])
    # for i in connect_holder:
    #     print(i)
    # for ix in range(len(polygon_holder)):
    #     print(polygon_holder[ix+1])
    #     print('\n')

    rospy.init_node("visualization")
    node_obj = PointTriangulation(pnt_holder, triangle_holder, polygon_holder, keys)
    try:
        node_obj.start()
    except rospy.ROSInterruptException:
        pass
