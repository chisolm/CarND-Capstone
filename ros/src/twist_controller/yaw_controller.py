from math import atan
import rospy

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

        #rospy.logwarn("wheel_base: {:08.4f}".format(wheel_base) +
        #              " steer_ratio {:08.4f}".format(steer_ratio) +
        #              " min_speed {:08.4f}".format(min_speed) +
        #              " max_lat_accel {:08.4f}".format(max_lat_accel) +
        #              " max_steer_angle {:08.4f}".format(max_steer_angle))

    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        #rospy.logwarn("angle: {:10.4f}".format(angle) +
        #              " wheel_base {:10.4f}".format(self.wheel_base) +
        #              " radius {:10.4f}".format(radius) +
        #              " self.steer_ratio {:10.4f}".format(self.steer_ratio) +
        #              " self.min_angle {:10.4f}".format(self.min_angle))
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        '''
        if abs(current_velocity) > 0.1:
            rospy.logwarn("current_velocity: {:08.4f}".format(current_velocity) +
                      " max_yaw_rate {:08.4f}".format(max_yaw_rate) +
                      " angular_velocity {:08.4f}".format(angular_velocity) +
                      " return {:08.4f}".format(self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0))
        else:    
            rospy.logwarn("current_velocity: {:08.4f}".format(current_velocity) +
                      " junk {:08.4f}".format(current_velocity) +
                      " angular_velocity {:08.4f}".format(angular_velocity) +
                      " return {:08.4f}".format(self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0))
        '''
        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
