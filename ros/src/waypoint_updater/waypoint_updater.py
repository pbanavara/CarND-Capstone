#!/usr/bin/env python

import rospy
import tf
import math

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from copy import deepcopy

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
RED_LIGHT_MIN_DIST = 0.0
AMPLIFICATION_FACTOR = 1

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.max_deceleration = 0.5

        self.saved_base_waypoints = None
        self.num_waypoints = None
        self.current_pose = None
        self.red_light_waypoint_idx = -1
	self.callback_time = rospy.get_time()
	self.initial_velocity = self.kmph2mps(rospy.get_param('~velocity')) 

        # Enter loop to process topic messages
        self.loop()


    def loop(self):
        '''
        Loop to process messages coming from '/current_pose'
        and '/traffic_waypoint' topics.
        @return: None
        '''

        rate = rospy.Rate(50) # 10Hz
        while not rospy.is_shutdown():
	    rospy.loginfo("info: ")
	    rospy.loginfo("info: ")
	    rospy.loginfo("info: ")
	    rospy.loginfo("info: **************************************************")
	    rospy.loginfo("info: **************************************************")
	    rospy.loginfo("info: **************************************************")
            if self.saved_base_waypoints is None or self.current_pose is None:
		rospy.loginfo("info: saved_base_waypoints or current_pose is None")
                continue

            self.publish_new_trajectory()

            rate.sleep()

    def waypoints_cb(self, msg):
        '''
        Message Type: styx_msgs/Lane
        '''
        self.saved_base_waypoints = msg.waypoints
        self.num_waypoints = len(msg.waypoints)
	#self.initial_velocity = self.saved_base_waypoints[0].twist.twist.linear.x


    def pose_cb(self, msg):
        '''
        Messsage type: geometry_msgs/PoseStamped
        '''
        self.current_pose = msg.pose


    def traffic_cb(self, msg):
        '''
        Message Type: Int32
        '''
        self.red_light_waypoint_idx = msg.data-10
	self.callback_time = rospy.get_time()
	rospy.loginfo("info: red light callback with index: {}".format(self.red_light_waypoint_idx))

    def obstacle_cb(self, msg):
        '''
        Message Type: Int32
        '''
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity


    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def distance_2p(self, p1, p2):
        '''
        input : two waypoint pose
        output: distance between 2 waypoint pose
        '''
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        return dl(p1,p2)


    def find_closest_waypoint_index(self, cur_pose, waypoints):
        '''
        This method is based on the same concepts as
        the respective one in the Path Planning Project

        :param cur_pose: car current position
        :param waypoints: list of waypoints
        :return: index of the closest waypoint
        '''

        closest_idx = None
        closest_dist = 1e10
        for idx, wp in enumerate(waypoints):
            dist = self.distance_2p(wp.pose.pose.position, cur_pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = idx

        return closest_idx


    def get_next_waypoint_index(self, cur_pose, waypoints):
        '''
        This method is based on the same concepts as
        the respective one in the Path Planning Project

        :param cur_pose: car current position
        :param waypoints: list of waypoints
        :return: index of the next (closest) waypoint
        '''

        closest_waypoint_idx = self.find_closest_waypoint_index(cur_pose, waypoints)

        base_wpx = waypoints[closest_waypoint_idx].pose.pose.position.x
        base_wpy = waypoints[closest_waypoint_idx].pose.pose.position.y

        heading = math.atan2((base_wpy - cur_pose.position.y), (base_wpx - cur_pose.position.x))
        q = (cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w)

        _, _, yaw = tf.transformations.euler_from_quaternion(q)
        angle = abs(yaw - heading)

        if angle > math.pi/4:
            closest_waypoint_idx += 1

        return closest_waypoint_idx


    def decelerate(self, index):
        '''
        Starting from the car waypoint it
        decelerates all the next ones

        :param index: index of the car
        :return: new velocity value
        '''

        d = self.distance(self.saved_base_waypoints, index, self.red_light_waypoint_idx)
        car_wp = self.saved_base_waypoints[index]
        car_vel = car_wp.twist.twist.linear.x
        vel = 0.0

        if d > RED_LIGHT_MIN_DIST:
            #vel = (d - RED_LIGHT_MIN_DIST) * (car_vel ** (1-self.max_deceleration))
            vel = math.sqrt(2*d*self.max_deceleration)
        if vel < 1.0:
            vel = 0.0

        if vel >self.initial_velocity:
            vel = self.initial_velocity

        return vel


    def publish_new_trajectory(self):
        car_index = self.find_closest_waypoint_index(self.current_pose, self.saved_base_waypoints)
        m = min(self.num_waypoints, car_index + LOOKAHEAD_WPS)
        #ahead_waypoints = deepcopy(self.saved_base_waypoints[car_index:m])
        ahead_waypoints = self.saved_base_waypoints[car_index:m]
	rospy.loginfo("info: ahead waypoints num {}".format(len(ahead_waypoints)))
	rospy.loginfo("info: ahead waypoints velocity {}".format(ahead_waypoints[0].twist.twist.linear.x))
	
	is_red_light_not_passed = rospy.get_time() - self.callback_time < 1 
	is_red_light_ahead = False

        if self.red_light_waypoint_idx > -1:
	    rospy.loginfo("info: there is red light. {} -- {}".format(car_index, self.red_light_waypoint_idx))
            if (self.red_light_waypoint_idx - car_index) > 0:
                d = self.distance(self.saved_base_waypoints, car_index, self.red_light_waypoint_idx)
                car_wp = self.saved_base_waypoints[car_index]
		d2 = AMPLIFICATION_FACTOR * (car_wp.twist.twist.linear.x **2)/(2* self.max_deceleration)
		rospy.loginfo("info: distance: {}, decelerated {}".format(d, d2))
                if d>0:
                    is_red_light_ahead = True

            else:
	        for i, waypoint in enumerate(ahead_waypoints):
                    waypoint.twist.twist.linear.x = self.initial_velocity
                    rospy.loginfo("info: velocity - {}".format(self.initial_velocity))


            if is_red_light_ahead and is_red_light_not_passed:
		rospy.loginfo("info: red light is ahead")
                for i, waypoint in enumerate(ahead_waypoints):
                    velocity = self.decelerate(car_index + i)
		    waypoint.twist.twist.linear.x = velocity
		    rospy.loginfo("info: velocity - {}".format(velocity))
#	    else:
#		for i, waypoint in enumerate(ahead_waypoints):
#                    waypoint.twist.twist.linear.x = self.initial_velocity
#                    rospy.loginfo("info: velocity - {}".format(self.initial_velocity))
        else:
	    for i, waypoint in enumerate(ahead_waypoints):
                waypoint.twist.twist.linear.x = self.initial_velocity
                rospy.loginfo("info: velocity - {}".format(self.initial_velocity))



        lane = Lane()
        lane.header.frame_id = '/world'
        lane.waypoints = ahead_waypoints
        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
