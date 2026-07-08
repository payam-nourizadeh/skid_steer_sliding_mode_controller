"""
Sketch of how to wrap `smc_controller` in a ROS node.

This file is NOT meant to be run as-is (it imports rospy, which is not a
dependency of this package). It illustrates the recommended pattern if you
still want to run this controller inside ROS1 or ROS2: keep all ROS
plumbing (subscribers, publishers, tf, messages) in a thin wrapper, and
call into the framework-agnostic `smc_controller` package for the actual
control law. This is the same math as the original ROS1 (Melodic) node
this package was extracted from, just decoupled from ROS.

--------------------------------------------------------------------------
Example ROS1 wrapper (pseudo-code, fill in message types for your stack):

    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    from smc_controller import SlidingModeController, compute_tracking_error


    class SMCNode:
        def __init__(self):
            self.controller = SlidingModeController.from_yaml(
                rospy.get_param("~config_path")
            )
            self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
            self.goal = None  # (x_d, y_d, th_d, vx_d, omega_d, vx_dot_d, omega_dot_d)
            rospy.Subscriber("odom", Odometry, self.odom_cb, queue_size=1)
            rospy.Subscriber("goal", ..., self.goal_cb, queue_size=1)

        def goal_cb(self, msg):
            self.goal = (...)  # unpack your goal message here

        def odom_cb(self, msg):
            if self.goal is None:
                return

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            theta = ...  # extract yaw from msg.pose.pose.orientation
            vx = msg.twist.twist.linear.x
            omega = msg.twist.twist.angular.z

            x_d, y_d, th_d, vx_d, omega_d, vx_dot_d, omega_dot_d = self.goal
            error = compute_tracking_error(x, y, theta, x_d, y_d, th_d)
            vx_r, omega_r = self.controller.compute(
                error, vx, omega, vx_d, omega_d, vx_dot_d, omega_dot_d
            )

            cmd = Twist()
            cmd.linear.x = vx_r
            cmd.angular.z = omega_r
            self.pub_cmd_vel.publish(cmd)


    if __name__ == "__main__":
        rospy.init_node("sliding_mode_controller")
        SMCNode()
        rospy.spin()

--------------------------------------------------------------------------
For ROS2, the same pattern applies: create an `rclpy.node.Node` subclass,
subscribe to odometry and goal topics, call `SlidingModeController.compute`
in the odometry callback, and publish a `geometry_msgs/Twist` (or
`TwistStamped`) on `cmd_vel`.
"""
