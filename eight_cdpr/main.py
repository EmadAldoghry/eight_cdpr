import rclpy
import numpy               as np
import tf_transformations  as tf_trans

from rclpy.node        import Node
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg   import Joy, JointState
from tf2_ros           import TransformStamped, TransformBroadcaster


class eight_cdpr(Node):
	def __init__(self,name):
		super().__init__(name)

		self.odom_broadcaster = TransformBroadcaster(self, 10)
		self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
		self.joystick_sub = self.create_subscription(Joy, 'joy', self.getJoystickInput, 10)
		
		self.joystick_sub  # prevent unused variable warning

		self.scale_factor = 0.1

		# Attributes to define the starting position of the robot
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.theta = 0.0
		self.phi = 0.0
		self.psi = 0.0

		# endeffector transformation odom -> endeffector
		self.odom_trans = TransformStamped()
		self.odom_trans.header.frame_id = "odom"
		self.odom_trans.child_frame_id = "base_link"
		
		
		# Definition attributes for the joints state transformations

		self.jointstate = JointState()
		self.jointstate.name = ["ee_continuous_x", "ee_continuous_y", "ee_continuous_z"]

		self.get_logger().info(f"eight cable parallel driven robot is on...") #just to know it works

	def getJoystickInput(self, joy_msg):
		self.x += self.scale_factor * joy_msg.axes[4]
		self.y += self.scale_factor * joy_msg.axes[5]
		self.z += self.scale_factor * joy_msg.buttons[4]
		self.z -= self.scale_factor * joy_msg.buttons[6]

		self.theta += joy_msg.axes[3]
		self.phi += joy_msg.axes[1]
		self.psi += joy_msg.axes[0]

		self.broadcastTransformations()

	def broadcastTransformations(self):
		self.time_now = self.get_clock().now().to_msg()
		self.odom_trans.header.stamp = self.time_now
		self.odom_trans.transform.translation.x = self.x
		self.odom_trans.transform.translation.y = self.y
		self.odom_trans.transform.translation.z = self.z

		q = tf_trans.quaternion_from_euler(0.0, 0.0, 0.0)

		self.odom_trans.transform.rotation = Quaternion(x = q[0],y = q[1],z = q[2],w = q[3])

		self.odom_broadcaster.sendTransform(self.odom_trans)

		self.jointstate.header.stamp = self.time_now
		self.jointstate.position =[np.radians(self.theta), np.radians(self.phi), np.radians(self.psi)]

		self.joint_state_pub.publish(self.jointstate)

def main(args=None):
	rclpy.init(args=args)
	EightCDPR = eight_cdpr('eight_cdpr')
	rclpy.spin(EightCDPR) # equal to while true
	eight_cdpr.destroy_node()
	rclpy.shutdown()

if __name__=='__main__':
    main()