#!/usr/bin/env python3


import rospy
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class TurtleBotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=False)

        # --- Wait for sim clock (critical when Gazebo is running) ---
        rospy.loginfo("Waiting for /clock to start ticking...")
        while rospy.Time.now().to_sec() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.01)
        rospy.loginfo("Clock active.")

        # --- Publishers / Subscribers ---
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Control params
        self.rate = rospy.Rate(10)
        self.linear_speed = 0.15
        self.angular_speed = 0.5

        # --- Wait for connections ---
        rospy.loginfo("Waiting for /cmd_vel subscriber (Gazebo)...")
        while self.cmd_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("cmd_vel connected.")

        rospy.loginfo("Waiting for first /odom message...")
        rospy.wait_for_message('/odom', Odometry)
        rospy.loginfo("Controller ready.")

    # ---------- Callback ----------
    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_theta = yaw
        self.odom_received = True

    # ---------- Required functions ----------
    def driving_straight(self, distance):
        start_x, start_y = self.current_x, self.current_y
        twist = Twist()
        twist.linear.x = self.linear_speed if distance >= 0 else -self.linear_speed
        traveled = 0.0
        while traveled < abs(distance) and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            traveled = math.sqrt(
                (self.current_x - start_x) ** 2 + (self.current_y - start_y) ** 2
            )
            self.rate.sleep()
        self.stopping()

    def rotating(self, angle):
        start_theta = self.current_theta
        twist = Twist()
        twist.angular.z = self.angular_speed if angle >= 0 else -self.angular_speed
        rotated = 0.0
        while rotated < abs(angle) and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            delta = self.current_theta - start_theta
            delta = math.atan2(math.sin(delta), math.cos(delta))
            rotated = abs(delta)
            self.rate.sleep()
        self.stopping()

    def spinning_wheels(self, duration):
        twist = Twist()
        twist.angular.z = self.angular_speed
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            self.rate.sleep()
        self.stopping()

    def stopping(self):
        twist = Twist()
        for _ in range(5):
            self.cmd_pub.publish(twist)
            self.rate.sleep()

    def navigating_to_pose(self, pose_msg):
        target_x = pose_msg.position.x
        target_y = pose_msg.position.y
        q = pose_msg.orientation
        (_, _, target_theta) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        heading = math.atan2(dy, dx)
        angle_to_goal = heading - self.current_theta
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))
        self.rotating(angle_to_goal)

        distance = math.sqrt(dx ** 2 + dy ** 2)
        self.driving_straight(distance)

        final_rot = target_theta - self.current_theta
        final_rot = math.atan2(math.sin(final_rot), math.cos(final_rot))
        self.rotating(final_rot)

    def drive_circle(self, radius):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.linear_speed / radius
        period = (2 * math.pi * radius) / self.linear_speed
        end_time = rospy.Time.now() + rospy.Duration(period)
        rospy.loginfo("Driving circle: r=%.2fm, T=%.1fs", radius, period)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            self.rate.sleep()
        self.stopping()

    def drive_square(self, side_length):
        for i in range(4):
            rospy.loginfo("Square side %d/4", i + 1)
            self.driving_straight(side_length)
            self.rotating(math.pi / 2)

    def random_dance(self, num_moves=8):
        import random
        for i in range(num_moves):
            choice = random.choice(['forward', 'back', 'spin_l', 'spin_r', 'circle'])
            rospy.loginfo("Dance move %d: %s", i + 1, choice)
            if choice == 'forward':
                self.driving_straight(random.uniform(0.1, 0.4))
            elif choice == 'back':
                self.driving_straight(-random.uniform(0.1, 0.3))
            elif choice == 'spin_l':
                self.rotating(random.uniform(0.5, math.pi))
            elif choice == 'spin_r':
                self.rotating(-random.uniform(0.5, math.pi))
            elif choice == 'circle':
                self.drive_circle(random.uniform(0.2, 0.5))


def main():
    controller = TurtleBotController()

    # controller.drive_circle(0.5)
    # controller.drive_square(0.5)
    controller.random_dance()

    rospy.loginfo("Done.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass