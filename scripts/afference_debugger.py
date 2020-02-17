#!/usr/bin/env python

"""
    this file is used to produce a visualization_msgs/Marker msg of type "line".
    to use it:
        1) add a node in your launch file
        --> <node type="afference_debugger.py" pkg="multirobot_interference" name="afference_debugger" />
        2) in config.yaml, set "debug_afference" to True
    
    this node subscribes to '/afference_debug' topic, which is published by toponavigator.py
    it receives:
        1) position of the robot
        2) position of the DWAPlanner afference
        3) position of the Euclidean afference
    then draws two arrows from the robot position to the two afferences.
    
    the GREEN arrow is the DWAPlanner afference
    the RED arrow is the Euclidean afference
    (if you don't see both at the same time, it means that they overlap, hence the afference is the same)
"""

import rospy

from multirobot_interference.msg import AfferenceDebug
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Quaternion, Vector3


def build_marker(start, goal, color):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    
    ns = ''
    type = 0  # arrow
    action = 0
    # pose = Pose(robot_posit, Quaternion(0, 0, 0, 1))
    points = [start, goal]
    lifetime = rospy.Duration(1)
    frame_locked = False
    scale = Vector3(0.1, 0.1, 0.1)
    
    marker = Marker()
    marker.header = header
    marker.ns = ns
    marker.type = type
    marker.action = action
    marker.points = points
    marker.color = color
    marker.lifetime = lifetime
    marker.frame_locked = frame_locked
    marker.scale = scale
    
    return marker


def on_afference(msg):
    robot_posit = msg.robot_posit
    eucl_posit = msg.eucl_afference
    mvbs_posit = msg.mvbs_afference
    
    red = ColorRGBA(1, 0, 0, 1)
    green = ColorRGBA(0, 1, 0, 1)
    
    arrow_to_eucl = build_marker(robot_posit, eucl_posit, color=red)
    arrow_to_mvbs = build_marker(robot_posit, mvbs_posit, color=green)
    
    publish(arrow_to_eucl, '/eucl_aff')
    publish(arrow_to_mvbs, '/mvbs_aff')
    

def publish(msg, topic):
    pub = rospy.Publisher(topic, Marker, queue_size=10)
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('afference_debugger')
    
    rospy.Subscriber('/afference_debug', AfferenceDebug, on_afference)
    
    rospy.spin()
