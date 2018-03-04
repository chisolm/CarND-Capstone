#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import copy
import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.waypoints = None
        self.final_waypoints = None
        self.waypoint_idx = None
        self.lookahead_wps = LOOKAHEAD_WPS
        self.upcoming_red = False

        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(2) # 50Hz
        while not rospy.is_shutdown():
            if ( self.pose != None and self.waypoints != None and self.final_waypoints != None):
                self.publish_final_waypoints()
            rate.sleep()

    def publish_final_waypoints(self):
        lane = Lane()
        # TODO what do I do for a header?
        lane.header = self.waypoints.header
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)
        pass

    def gen_final_waypoints(self):
        dl = lambda a, b: math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
        if ( self.pose != None and self.waypoints != None):
            # We are assuming that the pose orientation of the car aligns with
            # the positive increment of the waypoint indexes.  And correct track
            # direction is positive waypoint indexes.

            # TODO also assuming first time the first waypoint is ahead of us.  Could
            # just increment the waypoint_idx on the very first time.

            if self.waypoint_idx + LOOKAHEAD_WPS > self.waypoints_size:
                remainder = 1 + self.waypoint_idx + LOOKAHEAD_WPS - self.waypoints_size;
                self.final_waypoints = self.waypoints.waypoints[self.waypoint_idx:] + self.waypoints.waypoints[:remainder]
            else:
                self.final_waypoints = self.waypoints.waypoints[self.waypoint_idx:self.waypoint_idx + LOOKAHEAD_WPS]

            # Logic here to reduce speed to light.  Work backwords from light
            if self.upcoming_red:
                # distance required to decelerate to 0?  20m?
                decel_wp_count = 60.0
                # distance to red
                red_dist = dl(self.waypoints.waypoints[self.waypoint_idx].pose.pose.position,
                    self.waypoints.waypoints[self.light_wp_idx].pose.pose.position)
                red_final_idx = self.light_wp_idx - self.waypoint_idx - 12 # try 10 waypoints early
                for idx, wp in enumerate(self.final_waypoints):
                    wp_dist = dl(wp.pose.pose.position,
                        self.waypoints.waypoints[self.light_wp_idx].pose.pose.position)
                    if red_final_idx - idx <= 0:
                        wp.twist.twist.linear.x = 0.0
                    elif red_final_idx - idx < decel_wp_count:
                        precent_change = (((1.0 * red_final_idx) - (1.0 * idx)) / decel_wp_count) # - .3
                        # Hardcoding value rather than making deep copy
                        # Should make second copy of the waypoints a modify one and borrow values from the untouched copy.
                        wp.twist.twist.linear.x = 11.111 * max(precent_change, 0)
                    #rospy.logwarn("\t\tidx {} x {} ".format(idx + self.waypoint_idx, self.final_waypoints[idx].twist.twist.linear.x))
                rospy.logwarn("WU final idx 0 speed {} ".format(self.final_waypoints[0].twist.twist.linear.x) +
                    "red_dist {} red_final_idx {}".format(red_dist, red_final_idx))
                if self.debug_countdown > 0:
                    debug_string = ""
                    for fwp in self.final_waypoints:
                        debug_string = debug_string + "{:5.2f} ".format(fwp.twist.twist.linear.x)
                    rospy.logwarn(debug_string)
                    self.debug_countdown -= 1
            else:
                self.debug_countdown = 2
                for idx, wp in enumerate(self.final_waypoints):
                    wp.twist.twist.linear.x = 11.111


        else:
            rospy.logwarn("Not ready to generate final_waypoints")


        # publish in slower loop for load issues
        #self.publish_final_waypoints()
        return

    def pose_cb(self, msg):
        prior_pose = self.pose
        self.pose = msg
        if self.waypoints != None:
            self.waypoint_idx = self.closest_waypoint(self.pose)
            # TODO - Generate the /final_waypoints here or in a frequency controlled loop?
            self.gen_final_waypoints()


    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_size = np.shape(self.waypoints.waypoints)[0]
        pass

    def traffic_cb(self, msg):
        self.light_wp_idx = msg.data
        # light_wp_idx can be -1 for many cases
        # - no light seen
        # - light not red
        # Also check to see it is ahead of us
        # - how to determine ahead - base on relative waypoint
        if self.light_wp_idx >= 0 and (self.light_wp_idx - self.waypoint_idx > -10) and \
                                    (self.light_wp_idx - self.waypoint_idx < 200):
            self.upcoming_red = True
        else:
            self.upcoming_red = False

        rospy.logwarn("waypoint_updater.py traffic_cb {} wpidx {} dist {}".format(self.light_wp_idx,
            self.waypoint_idx, self.upcoming_red))
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def waypoint_range(self, start, end, buffer_size):
        i = start
        end %= buffer_size
        while i != end:
            yield i
            i += 1
            i %= buffer_size

    def closest_waypoint(self, pose):
        # Need this to determine what waypoints to publish...
        min_distance = 1000000
        min_idx = None
        dl = lambda a, b: math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

        # Brute force search
        # TODO - change to binary search - binary search could find a local
        # closest point that could be completely wrong.
        slow_machine = True
        if slow_machine and self.waypoint_idx != None:
            start = self.waypoint_idx - 5
            end = self.waypoint_idx + 50
            for idx in self.waypoint_range(start, end, self.waypoints_size):
                w = self.waypoints.waypoints[idx]
                dist =  dl(w.pose.pose.position, pose.pose.position)
                #rospy.logwarn("self.waypoint_idx {} dist {}".format(idx, dist))
                if (dist < min_distance):
                    min_distance = dist
                    min_idx = idx
            if (min_idx == None):
                rospy.logwarn("Error: waypoint not found, slow machine method")
        else:
            for idx, w in enumerate(self.waypoints.waypoints):
                dist =  dl(w.pose.pose.position, pose.pose.position)
                #rospy.logwarn("self.waypoint_idx {} dist {}".format(idx, dist))
                if (dist < min_distance):
                    min_distance = dist
                    min_idx = idx
            if (min_idx == None):
                rospy.logwarn("Error: waypoint not found, long method")
        return min_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
