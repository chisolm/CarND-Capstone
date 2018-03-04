#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from cv_bridge import CvBridge
import numpy as np
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_valid = False
        self.camera_image = None
        self.lights = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights',
                                TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb,
            queue_size=1, buff_size=2**22)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']

        self.upcoming_red_light_pub = rospy.Publisher(
            '/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.last_time = rospy.Time()  # Epoch
        self.traffic_one_shot = True

        # See wiki below
        self.bridge = CvBridge()

        rospy.spin()
        #self.ground_truth_loop()

    #def ground_truth_loop(self):
    #    rate = rospy.Rate(2) # 50Hz
    #    while not rospy.is_shutdown():
    #        if ( self.pose != None and self.waypoints != None):
    #            self.image_cb(True)
    #        rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg
        time = rospy.get_rostime()
        delta = time - self.last_time
        # Workaround to send not more than 2 images per second
        # TODO, is this dead code?
        if delta > rospy.Duration(.5):
            rospy.logwarn("calling image_cb {}".format(delta - rospy.Duration(.5)))
            self.image_cb(True)
            self.last_time = time

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_size = np.shape(self.waypoints.waypoints)[0]
        self.waypoints_valid = True

    def traffic_cb(self, msg):
        self.lights = msg.lights
        if self.traffic_one_shot is True and self.waypoints_valid is True:
            print("type stop_line_positions {}".format(type(self.stop_line_positions)))
            for idx, line in enumerate(self.stop_line_positions):
                print("self.stop_line_positions {}".format(line))
                #print("self.stop_line_positions 2 {} ".format(self.stop_line_positions[idx]))
            for idx, light in enumerate(self.lights):
                print("light [{}, {}], ".format(light.pose.pose.position.x, light.pose.pose.position.y))
            for idx, light in enumerate(self.lights):
                #print("idx ", idx, "light {}".format(light))
                print("idx ", idx, "light {} {}".format(light.pose.pose.position.x, light.pose.pose.position.y))
                light_wp = self.get_closest_waypoint(light.pose.pose)
                light_idx = self.get_closest_light_line(light.pose.pose, self.stop_line_positions)
                print("light_wp", light_wp, " light_idx", light_idx)
                print("waypoint", self.waypoints.waypoints[light_wp], " light_line", self.stop_line_positions[light_idx])
                self.traffic_one_shot = False

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera
        """

        if self.waypoints_valid is False or self.lights is None:
            return

        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            #rospy.logwarn("tl_detector image_cb {}".format(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            #rospy.logwarn("tl_detector image_cb {}".format(self.last_wp))
        self.state_count += 1
        #rospy.logwarn("tl_detector image_cb count {}".format(self.state_count))

    def waypoint_range(self, start, end, buffer_size):
        i = start
        end %= buffer_size
        while i != end:
            yield i
            i += 1
            i %= buffer_size

    def get_closest_waypoint_xy(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        min_distance = 1000000
        min_idx = None

        def dl(a, x, y): return math.sqrt((a.x - x) ** 2 + (a.y - y) ** 2)
        # TODO test
        # dl = lambda a, b: (a.x - b.x)**2 + (a.y - b.y)**2

        # Brute force search
        # TODO - change to binary search - binary search could find a local
        # closest point that could be completely wrong.
        #rospy.logwarn("get_closest_waypoint pose {}".format(pose))
        slow_machine = True
        if slow_machine and self.last_wp != -1:
            start = self.last_wp - 5
            end = self.last_wp + 50
            for idx in self.waypoint_range(start, end, self.waypoints_size):
                w = self.waypoints.waypoints[idx]
                #rospy.logwarn("pose.position {}".format(pose.position))
                #rospy.logwarn("w.pose.position {}".format(w.pose.position))
                dist = dl(w.pose.pose.position, x, y)
                #rospy.logwarn("self.waypoint_idx {} dist {}".format(idx, dist))
                if (dist < min_distance):
                    min_distance = dist
                    min_idx = idx
            if (min_idx is not None):
                return min_idx
            else:
                rospy.logwarn("Error: waypoint not found")

        for idx, w in enumerate(self.waypoints.waypoints):
            #rospy.logwarn("pose.position {}".format(pose.position))
            dist = dl(w.pose.pose.position, x, y)
            #rospy.logwarn("self.waypoint_idx {} dist {}".format(idx, dist))
            if (dist < min_distance):
                min_distance = dist
                min_idx = idx
        if (min_idx == None):
            rospy.logwarn("Error: waypoint not found")
        return min_idx

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        min_distance = 1000000
        min_idx = None

        #def dl(a, b): return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)
        def dl(a, b): return (a.x - b.x) ** 2 + (a.y - b.y) ** 2

        # Brute force search
        # A change to binary search could find a local
        # closest point that could be incorrect.  Ie the case of a very curvy
        # route.
        slow_machine = True
        if slow_machine and self.last_wp != -1:
            start = self.last_wp - 5
            end = self.last_wp + 50
            for idx in self.waypoint_range(start, end, self.waypoints_size):
                w = self.waypoints.waypoints[idx]
                #rospy.logwarn("pose.position {}".format(pose.position))
                #rospy.logwarn("w.pose.position {}".format(w.pose.position))
                dist = dl(w.pose.pose.position, pose.position)
                #rospy.logwarn("self.waypoint_idx {} dist {}".format(idx, dist))
                if (dist < min_distance):
                    min_distance = dist
                    min_idx = idx
            if (min_idx is not None):
                return min_idx
            else:
                rospy.logwarn("Error: waypoint not found")

        for idx, w in enumerate(self.waypoints.waypoints):
            #rospy.logwarn("pose.position {}".format(pose.position))
            dist = dl(w.pose.pose.position, pose.position)
            #rospy.logwarn("self.waypoint_idx {} dist {}".format(idx, dist))
            if (dist < min_distance):
                min_distance = dist
                min_idx = idx
        if (min_idx == None):
            rospy.logwarn("Error: waypoint not found")
        return min_idx

    def get_closest_light(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        min_distance = 1000000
        min_idx = None
        #dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        # TODO test

        def dl(a, b): return (a.x - b.x)**2 + (a.y - b.y)**2
        for idx, w in enumerate(self.lights):
            dist = dl(w.pose.pose.position, pose.position)
            #rospy.logwarn("self.waypoint_idx {} dist {}".format(idx, dist))
            if (dist < min_distance):
                min_distance = dist
                min_idx = idx
        if (min_idx == None):
            rospy.logwarn("Error: waypoint not found: get_closest_light")
        return min_idx

    def get_closest_light_line(self, pose, lines):
        min_distance = 1000000
        min_idx = None

        def dl(ax, ay, b): return (ax - b.x)**2 + (ay - b.y)**2
        for idx, line in enumerate(lines):
            dist = dl(line[0], line[1], pose.position)
            #rospy.logwarn("self.waypoint_idx {} dist {}".format(idx, dist))
            if (dist < min_distance):
                min_distance = dist
                min_idx = idx
        if (min_idx == None):
            rospy.logwarn("Error: waypoint not found: get_closest_light_line")
        return min_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        #return self.light_classifier.get_classification(cv_image)
        return TrafficLight.RED

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # List of positions that correspond to the line to stop in front of for a given intersection
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #light_idx = self.get_closest_light(self.pose.pose)
        line_idx = self.get_closest_light_line(self.pose.pose, self.stop_line_positions)
        # TODO find the closest visible traffic light (if one exists)
        # This assumes the stop_line_positions and the lights list are in the same order
        light_wp = self.get_closest_waypoint(self.lights[line_idx].pose.pose)
        line_wp = self.get_closest_waypoint_xy(self.stop_line_positions[line_idx][0],
            self.stop_line_positions[line_idx][1])

        def dl(a, b): return (a.x - b.x)**2 + (a.y - b.y)**2
        def dl2(a, x, y): return math.sqrt((a.x - x) ** 2 + (a.y - y) ** 2)

        #if light:
        if True:
            # TODO using ground truth now, finish detector
            #state = self.get_light_state(light)

            rospy.logwarn("TLD car {:07.2f} {:07.2f}".format(self.pose.pose.position.x, self.pose.pose.position.y) +
                        " line {:07.2f} {:07.2f} {:07.2f}".format(self.stop_line_positions[line_idx][0], self.stop_line_positions[line_idx][1],
                            dl2(self.pose.pose.position, self.stop_line_positions[line_idx][0], self.stop_line_positions[line_idx][1])) +
                        " light {:07.2f} {:07.2f} {:07.2f}".format(self.waypoints.waypoints[light_wp].pose.pose.position.x, self.waypoints.waypoints[light_wp].pose.pose.position.y,
                            dl(self.pose.pose.position, self.waypoints.waypoints[light_wp].pose.pose.position))
                )
            rospy.logwarn("TLD car {} {}".format(self.pose.pose.position.x, self.pose.pose.position.y) +
                        " line {} {} {} {} {}".format(self.stop_line_positions[line_idx][0], self.stop_line_positions[line_idx][1],
                            dl2(self.pose.pose.position, self.stop_line_positions[line_idx][0], self.stop_line_positions[line_idx][1]),
                            self.stop_line_positions[line_idx][0], self.stop_line_positions[line_idx][1]))

            # This assumes the stop_line_positions and the lights list are in the same order
            state = self.lights[line_idx].state
            #rospy.logwarn("lightwp {} state {}".format(light_wp, state))

            #return light_wp, state
            return line_wp, state

        if False:
            # Too far to try - ? 60m?
            dist_to_line = dl2(self.pose.pose.position, self.stop_line_positions[line_idx][0], self.stop_line_positions[line_idx][1])
            if ( dist_to_line >= 60):
                return -1, TrafficLight.UNKNOWN
            else:
                # reminder:
                # http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
                rgb_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
                state = self.light_classifier.get_classification(rgb_image)
                return line_wp, state

        self.waypoints = None
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
