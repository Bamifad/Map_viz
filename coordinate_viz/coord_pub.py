#!/usr/bin/python

"""
Picture size is 10779 x 10000
"""

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class CoordMark(object):
    def __init__(self):
        self.pixel_marker_obj = rospy.Publisher('/pixel_mark', Marker, queue_size=1000)
        self.rate = rospy.Rate(30)
        self.pix_mark = Marker()
        self.line_mark = Marker()
        self.set_pixel_marker()
        self.set_line_marker()
        self.coord_generator()

    def set_pixel_marker(self):
        self.pix_mark.header.frame_id = "/map"
        #self.pix_mark.header.stamp = rospy.get_rostime()
        self.pix_mark.ns = "Points"
        self.pix_mark.id = 0
        self.pix_mark.type = Marker.POINTS
        self.pix_mark.action = Marker.ADD
        self.pix_mark.pose.orientation.w = 1.0
        self.pix_mark.scale.x = 100.0
        self.pix_mark.scale.y = 100.0
        self.pix_mark.scale.z = 0.0
        self.pix_mark.color.g = 1.0
        self.pix_mark.color.b = 0.0
        self.pix_mark.color.a = 1.0
        self.pix_mark.lifetime = rospy.Duration()

    def set_line_marker(self):
        self.line_mark.header.frame_id = "/map"
        #self.line_mark.header.stamp = rospy.get_rostime()
        self.line_mark.ns = "Lines"
        self.line_mark.id = 1
        self.line_mark.type = Marker.LINE_STRIP
        self.line_mark.action = Marker.ADD
        self.line_mark.pose.orientation.w = 1.0
        self.line_mark.scale.x = 100.0
        self.line_mark.scale.y = 100.0
        self.line_mark.scale.z = 0.0
        self.line_mark.color.r = 0.0
        self.line_mark.color.g = 0.0
        self.line_mark.color.b = 1.0
        self.line_mark.color.a = 1.0
        self.line_mark.lifetime = rospy.Duration()

    def coord_generator(self):
        filename = "/home/bamifad00/catkin_ws/src/coord_viz/scripts/coordinates.txt"
        x_coord_str = []
        y_coord_str = []
        with open(filename, "r") as file:
            i = 1
            for line in file:
                if line == "\n":
                    pass
                else:
                    if i % 2 == 1:
                        x_coord_str.append(line.strip('\n'))
                    else:
                        y_coord_str.append(line.strip('\n'))
                    i += 1

        # Converting the coordinates to pixel locations
        x_pixel = []
        y_pixel = []

        for coord in x_coord_str:
            x_pixel.append((float(coord) - 3505006.1712850006)/0.71193)
        for coord in y_coord_str:
            y_pixel.append((float(coord) - 10214478.1234050021)/-0.71193)

        pixel_points = []
        for index in range(len(x_pixel)):
            p = Point()
            p.x = x_pixel[index]
            p.y = y_pixel[index]
            p.z = 0
            pixel_points.append(p)
	
	#print(pixel_points)
        self.pix_mark.points = pixel_points
        self.line_mark.points = pixel_points

    def start(self):
        while not rospy.is_shutdown():
            self.pixel_marker_obj.publish(self.pix_mark)
            self.pixel_marker_obj.publish(self.line_mark)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('pixel_node', anonymous=True)
    marker_obj = CoordMark()
    try:
        marker_obj.start()
    except rospy.ROSInterruptException:
        pass



