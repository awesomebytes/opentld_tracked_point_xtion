#! /usr/bin/env python

import roslib; roslib.load_manifest('opentld_tracked_point_xtion')
import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point
from tld_msgs.msg import BoundingBox
from sensor_msgs.msg import PointCloud2, PointField
# from sensor_msgs.point_cloud2 import read_points
#from point_cloud2 import read_points
#import point_cloud2

from visualization_msgs.msg import Marker

import ctypes
import math
import struct

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    #assert isinstance(cloud, roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in xrange(height):
                offset = row_step * v
                for u in xrange(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in xrange(height):
                offset = row_step * v
                for u in xrange(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt




class opentld_tracked:

	def __init__(self):
		self.got_pointcloud = False
		self.pointcloud = None
		self.pub = rospy.Publisher('tracked_pose', PoseStamped)
		self.marker_publisher = rospy.Publisher("arrow", Marker)


	def p2callback(self, data):
		print "Received pointcloud2"
		self.got_pointcloud = True
		self.pointcloud = data


	def otld_callback(self, data):
		print "Heard: " + str(data)
		# get the depth image
		s = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.p2callback)
		while not self.got_pointcloud:
			# do nothing
			print "Waiting for pointcloud"
			rospy.sleep(0.1)

		s.unregister()
		self.got_pointcloud = False

		# get the depth of the tracked area
		# Header header
		# int32 x
		# int32 y
		# int32 width
		# int32 height
		# float32 confidence

		# probably first get the centroid, we should do a median here or something
		centerx = data.x + data.width / 2
		centery = data.y + data.height / 2
		#print "self.pointcloud.row_step: " + str(self.pointcloud.row_step) + " self.pointcloud.point_step: " + str(self.pointcloud.point_step)
		print "centerx: " + str(centerx) + " centery: " + str(centery) 
		#pointdepth = self.pointcloud.data[self.pointcloud.row_step * centery : self.pointcloud.row_step * centery + self.pointcloud.point_step]
		#point = read_points(self.pointcloud, field_names=None, skip_nans=False, uvs=[centerx, centery])
		point = read_points(self.pointcloud, field_names=None, skip_nans=False, uvs=[[centerx, centery]])
		print "point is: " + str(point)
		for item in point:
			print "item is: " + str(item)
			print "x: "+ str(item[0]) + " y: " + str(item[1]) + ", z: " + str(item[2])

		# create PoseStamped
		pose = PoseStamped()
		pose.header = std_msgs.msg.Header()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "camera_link" 
		pose.pose = Pose()
		pose.pose.position.x = item[2] #  kinect Z value, [2], is X in TF of camera_link
		pose.pose.position.y = - item[0] # kinect X value, [0], is -Y in TF of camera_link
		pose.pose.position.z = - item[1] # kinect Y value, [1], is -Z in TF of camera_link
		pose.pose.orientation.w = 1
		# send PoseStamped
		self.pub.publish(pose)

		arrow = Marker()
		arrow.header.frame_id = "camera_link"
		arrow.header.stamp = rospy.Time.now()
		arrow.type = Marker.ARROW
		arrow.points = [Point(0,0,0), Point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)]
		#arrow.points = [Point(0,0,0), Point(0,0,1)]
		arrow.scale.x = 0.1 # anchura del palo
		arrow.scale.y = 0.15 # anchura de la punta
		# arrow.scale.z = 0.1
		arrow.color.a = 1.0
		arrow.color.r = 1.0
		arrow.color.g = 1.0
		arrow.color.b = 1.0
		self.marker_publisher.publish(arrow)





	def listen_otld(self):
	    rospy.Subscriber("/tld_tracked_object", BoundingBox, self.otld_callback)
	    print "Waiting for boundingboxes"
	    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('visualize_tracked_point')
    o = opentld_tracked()
    o.listen_otld()