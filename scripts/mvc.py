#!/usr/bin/env python
import rospy
import struct
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

new_stuff = False

changes_pc2 = None
view_pc2 = None
map_pc2 = None

tf2_buffer = None

target_frame = "odom"

cpub = None

map_colour = [50,50,50]
map_colour = struct.unpack("f", struct.pack("i",int('%02x%02x%02x' % tuple(map_colour),16)))[0]
changes_colour = [255,0,0]
changes_colour = struct.unpack("f", struct.pack("i",int('%02x%02x%02x' % tuple(changes_colour),16)))[0]

# Shortcut of tf's lookup_transform
def lookupTF(source_frame):
    return tf2_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1))

# Changes callback
def changesCallback(msg):
    global new_stuff, changes_pc2
    try:
        tf = lookupTF(msg.header.frame_id)
        changes_pc2 = do_transform_cloud(msg, tf)
        new_stuff = True
    except Exception as e:
        rospy.logerr(e)

# Map callback
def mapCallback(msg):
    global new_stuff, map_pc2
    try:
        tf = lookupTF(msg.header.frame_id)
        map_pc2 = do_transform_cloud(msg, tf)
        new_stuff = True
    except Exception as e:
        rospy.logerr(e)

# Viewed area callback
def viewCallback(msg):
    global new_stuff, view_pc2
    try:
        tf = lookupTF(msg.header.frame_id)
        view_pc2 = do_transform_cloud(msg, tf)
        new_stuff = True
    except Exception as e:
        rospy.logerr(e)

# Combining the three pointclouds
def combineAndPublish():
    global changes_pc2, map_pc2, view_pc2, new_stuff
    if changes_pc2 is not None and map_pc2 is not None and view_pc2 is not None:
        # Visualization priority:
        # 1. Changes
        # 2. Viewed areas
        # 3. Map

        # Changes
        cc = list(pc2.read_points(changes_pc2, field_names=("x", "y", "z")))
        cc = [[k[0][0],k[0][1],k[0][2],k[1]] for k in list(zip(cc, [changes_colour for x in cc]))]

        # Viewed area
        vc = list(pc2.read_points(view_pc2, field_names=("x", "y", "z", "rgb")))
        #  vc = [k for k in vc if k not in cc]

        # Map
        mc = list(pc2.read_points(map_pc2, field_names=("x", "y", "z")))
        #  mc = [[k[0][0],k[0][1],k[0][2],k[1]] for k in list(zip(mc, [map_colour for x in mc])) if k not in cc and k not in map(lambda x:(x[0],x[1],x[2]),vc)]
        mc = [[k[0][0],k[0][1],k[0][2],k[1]] for k in list(zip(mc, [map_colour for x in mc]))] 

        # NOTE
        # Filtering duplicate points is too time-consuming to do
        # When (if) size becomes a problem maybe it's worth investigating
        # (Changing the order of merging keeps indirectly applies the colour priority)
        z = mc+vc+cc

        rgbfield = PointField()
        rgbfield.name = "rgb"
        rgbfield.offset = 16
        rgbfield.datatype = 7
        rgbfield.count = 1
        fields = view_pc2.fields

        cpub.publish(pc2.create_cloud(changes_pc2.header, fields, z))
        new_stuff = False

def init():
    global tf2_buffer, target_frame, cpub, new_stuff
    rospy.init_node("mvc")

    tf2_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf2_buffer)

    target_frame = rospy.get_param("/mvc/target_frame", "odom")
    rate = rospy.get_param("/mvc/range_rate", 5) # Hz

    ctopic= rospy.get_param("/mvc/combined_topic", "/mvc/combined_cloud")

    cpub = rospy.Publisher(ctopic, PointCloud2, queue_size=1)

    mtopic= rospy.get_param("/mvc/map_topic", "/octomap_tracking_server/octomap_point_cloud_centers")
    ctopic= rospy.get_param("/mvc/map_topic", "/octomap_tracking_server/changeset")
    vtopic= rospy.get_param("/mvc/map_topic", "/ros_rvv/viewed_area_pc2")

    msub = rospy.Subscriber(mtopic, PointCloud2, mapCallback)
    csub = rospy.Subscriber(ctopic, PointCloud2, changesCallback)
    vsub = rospy.Subscriber(vtopic, PointCloud2, viewCallback)

    while not rospy.is_shutdown():
        if new_stuff:
            combineAndPublish()
        rospy.sleep(1/rate)


if __name__ == "__main__":
    init()

