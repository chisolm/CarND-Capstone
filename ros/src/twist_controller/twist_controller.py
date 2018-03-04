import rospy

from lowpass import LowPassFilter
from yaw_controller import YawController
from pid import PID


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    # def __init__(self, *args, **kwargs):
    def __init__(self,
                 wheel_base,
                 steer_ratio,
                 min_speed,
                 max_lat_accel,
                 max_steer_angle,
                 decel_limit,
                 accel_limit):

        # TODO: Implement
        # Use same order of magnitude K values as PID project, these
        # are complete guesses for now.
        # Order of params is Kp, Ki, Kd
        self.accel_pid = PID(2, .01, .1)
        self.yaw_controller = YawController(wheel_base,
                                            steer_ratio,
                                            min_speed,
                                            max_lat_accel,
                                            max_steer_angle)
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.lowpass = LowPassFilter(1.0, 1.0)
        self.last_time = None

        pass

    def control(self, linear_v, angular_v, current_v, dbw_enabled):
        # def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.accel_pid.reset()
            return 0.0, 0.0, 0.0

        if self.last_time is None:
            self.last_time = rospy.get_time()
            return 0.0, 0.0, 0.0

        time = rospy.get_time()
        error = linear_v - current_v
        v_out = self.accel_pid.step(error, time - self.last_time)
        self.last_time = time

        steering = self.yaw_controller.get_steering(linear_v, angular_v,
                                                    current_v)
        # steering = self.lowpass.filt(steering)

        if v_out >= 0.0:     # Accelerate
            accel_out = v_out
            brake_out = 0.0
        elif v_out > - 0.05:  # Coast, should put through low pass filter
            accel_out = 0.0
            brake_out = 0.0
        else:                # Brake
            accel_out = 0.0
            #brake_out = min(v_out * 8, 5)
            #brake_out = min(- v_out * 8, -5)
            #brake_out = min(max(- error * 5, 1), 10)
            #brake_out = min(max(- error * 8, 5), 30)
            #brake_out = min(max(- error * 8, 5), 30)
            brake_out = max(-50.0 * error, 10.0) # what is the limit of brake value?  0-1 0-30? 0-100?

        rospy.logwarn("PID error: {:08.4f}".format(error) +
                      " v_out {:08.4f}".format(v_out) +
                      " current_v {:08.4f}".format(current_v) +
                      " linear_v {:08.4f}".format(linear_v) +
                      " brake raw {:08.4f}".format(- error * 5) +
                      " brake {:08.4f}".format(brake_out))

        return accel_out, brake_out, steering
