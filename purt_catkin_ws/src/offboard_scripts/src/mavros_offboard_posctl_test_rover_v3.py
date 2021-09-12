#!/usr/bin/env python3
import rospy
import math
import numpy as np

from mavros_msgs.msg import Altitude, ExtendedState, State
from mavros_msgs.srv import CommandBool, ParamGet, SetMode
from geometry_msgs.msg import PoseStamped, Quaternion
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler


class MavrosOffboardPosctlTest(object):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.
    """

    def __init__(self):
        self.extended_state = ExtendedState()
        self.local_position = PoseStamped()
        self.state = State()
        self.mav_type = None

        self.sub_topics_ready = {
            key: False
            for key in [
                'local_pos', 'state', 'ext_state',
            ]
        }

        # ROS services
        service_timeout = 10
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('uav0/mavros/cmd/arming', service_timeout)
            rospy.loginfo('arming found')
            rospy.wait_for_service('uav0/mavros/set_mode', service_timeout)
            rospy.loginfo('mode')
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")
        self.get_param_srv = rospy.ServiceProxy('uav0/mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('uav0/mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('uav0/mavros/set_mode', SetMode)

        # ROS subscribers
        self.ext_state_sub = rospy.Subscriber('uav0/mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)
        self.local_pos_sub = rospy.Subscriber('uav0/mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.qualisys_sub = rospy.Subscriber('/qualisys/rover/pose',
                                              PoseStamped,
                                              self.qualisys_callback)

        self.state_sub = rospy.Subscriber('uav0/mavros/state', State,
                                          self.state_callback)


        self.pos = PoseStamped()
        self.radius = 0.06
        self.pos_setpoint_pub = rospy.Publisher(
            'uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    #
    # Callback functions
    #
    def qualisys_callback(self, data):
        #rospy.loginfo("qualisys x: {:f} y: {:f} z: {:f}".format(
            #data.pose.position.x,
            #data.pose.position.y,
            #data.pose.position.z))
         a = 1

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            rospy.loginfo('ext_state ready')
            self.sub_topics_ready['ext_state'] = True

    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            rospy.loginfo('local_pos ready')
            self.sub_topics_ready['local_pos'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            rospy.loginfo('state ready')
            self.sub_topics_ready['state'] = True

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in range(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not arm_set:
            rospy.logwarn('failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}'.
            format(arm, old_arm, timeout))

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in range(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not mode_set:
            rospy.logwarn(
            "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
            format(mode, old_mode, timeout))

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in range(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not simulation_ready:
            rospy.logwarn(
            "failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
            format(self.sub_topics_ready, timeout))

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in range(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not landed_state_confirmed:
            rospy.logwarn(
            "landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                desired_landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][self.extended_state.landed_state].name,
                   index, timeout))

    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        res = False
        for i in range(timeout * loop_freq):
            try:
                res = self.get_param_srv('MAV_TYPE')
                if res.success:
                    self.mav_type = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received | type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type]
                               .name, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(res.success, (
            "MAV_TYPE param get failed | timeout(seconds): {0}".format(timeout)
        ))

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "map"

        while not rospy.is_shutdown():
            #rospy.loginfo('sending position {:f} {:f} {:f}'.format(self.pos.pose.position.x, self.pos.pose.position.y, self.pos.pose.position.z))
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.loginfo(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, wait, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0.0 # North (0 lateral, 90 tail, 270 head)
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        reach_time = None
        for i in range(timeout * loop_freq):
            now = rospy.Time.now()
            if reach_time is None:
                reach_time = now
            elapsed = (now - reach_time).to_sec()
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius) and elapsed > wait:
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        #rospy.loginfo('waiting for landed state')
        #self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        rospy.loginfo('setting offboard')
        self.set_mode("OFFBOARD", 10)
        rospy.loginfo('offboard set')
        rospy.loginfo('arming')
        self.set_arm(True, 5)
        rospy.loginfo('armed')

        rospy.loginfo("run mission")
        # ENU
        #positions = [(0, 0, 0.5), (0, 0, 3.5), (0, 0, 3.5), (0, 0, 0.05), (0, 0, -0.01)] # Takeoff - Landing
        positions = [(5, 0, -0.2), (0, 0, -0.2)]# ROVER
        #positions = [(0, 0, 1.75), (-8, 0, 1.75), (-8, 0, 0)]#, (0, 0, -0.05)] #Cruise
        #positions = [(0, 0, 0.5), (0, 0, 1.65), (0, 0, 1.65), (0, 0, 1.65), (0, 0, 1.65), (0, 0, -0.05)] # HOVER
        #[(0, 0, 0),(0, 0, 0.7),(0, 7, 0.7), (0, 7, 0.0)]#, (0, 3, 0)] 
        #[(0, 0, 0),(0, 0, 2.0),(0, 3, 2.0), (0, 3, 1.0), (0, 3, 0)]
        #[(0.0000,0.0000,1.5),(-0.1005,0.3750,1.5),(-0.3750,0.6495,1.5),(-0.7500,0.7500,1.5),(-1.1250,0.6495,1.5),(-1.3995,0.3750,1.5),(-1.5000,0.0000,1.5),(-1.3995,-0.3750,1.5),(-1.1250,-0.6495,1.5),(-0.7500,-0.7500,1.5),(-0.3750,-0.8505,1.5),(-0.1005,-1.1250,1.5),(0.0000,-1.5000,1.5),(-0.1005,-1.8750,1.5),(-0.3750,-2.1495,1.5),(-0.7500,-2.2500,1.5),(-1.1250,-2.1495,1.5),(-1.3995,-1.8750,1.5),(-1.5000,-1.5000,1.5),(-1.3995,-1.1250,1.5),(-0.7500,-0.7500,1.5),(-0.3750,-0.6495,1.5),(0.1005,-0.3750,1.5),(0.0000,0.0000,1.5)]
        #[(0, 0, 0),(0, 0, 1.5),(-2, 0, 1.5),(-2, 2, 1.5),(0, 2, 1.5),(0, 0, 1.5),(0, 0, 0)]

        wait = 8
        timeout = 12 #20 cruise
        #timeout2 = 6

        for i in range(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], wait, timeout)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':

    rospy.init_node('offboard')
    test = MavrosOffboardPosctlTest()
    test.test_posctl()
