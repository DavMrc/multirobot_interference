#!/usr/bin/env python

"""
    A simple utility used to kill gzserver and gzclient
    processes when a SIGINT command is received, that is
    when the roscore shuts down.
"""

import rospy
import subprocess
import signal


def kill(signal, frame):
    pop = subprocess.Popen(['pkill', '-SIGTERM', 'gzserver'])
    pop.communicate()

    pop = subprocess.Popen(['pkill', '-SIGTERM', 'gzclient'])
    pop.communicate()
    
    rospy.logwarn('gzserver killed with SIGTERM')
    rospy.signal_shutdown('gzserver/gzclient killed')


if __name__ == '__main__':
    rospy.init_node('gz_killer')
    
    signal.signal(signal.SIGINT, kill)
    
    rospy.spin()