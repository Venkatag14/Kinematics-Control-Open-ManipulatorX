import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
import time
from std_msgs.msg import Float64MultiArray
import math


class BasicRobotControl(Node):
    def __init__(self,init_pos):
        super().__init__('basic_robot_control')
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        # self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control') # Creating client to control gripper
        self.q  = init_pos # [0.0, math.radians(-70), math.radians(60), math.radians(20)]
        self.set_init_position()
        time.sleep(10)
        self.sub = self.create_subscription(Float64MultiArray, "ee_jv_pub", self.msgCallBack, 10)
        self.pub = self.create_publisher(Float64MultiArray, 'updated_joint_vel' , 10)
        
        
    def set_init_position(self):
        self.init_pose_pub = self.create_publisher(Float64MultiArray , 'init_pose_pub' , 10)
        msg = Float64MultiArray()
        msg.data = self.q
        self.init_pose_pub.publish(msg)

    def msgCallBack(self,msg):
        del_q = msg.data
        path_time = 0.1
        q1, q2, q3, q4 = [i+(j*path_time) for i,j in zip(self.q,del_q)]
        self.q = [q1, q2, q3, q4]
        msg = Float64MultiArray()
        msg.data = self.q
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # [0.0, math.radians(-70), math.radians(60), math.radians(20)] 

    val = [math.radians(0.0) , math.radians(0.0) , math.radians(-10.3) , math.radians(85.25)]

    init_pos = val #[float(sys.argv[1]) , float(sys.argv[2]) , float(sys.argv[3]) , float(sys.argv[4])]

    basic_robot_control = BasicRobotControl(init_pos)


    while rclpy.ok():
        rclpy.spin(basic_robot_control)
        if basic_robot_control.future.done():
            try:
                response = basic_robot_control.future.result()
            except Exception as e:
                basic_robot_control.get_logger().error('Service call failed %r' % (e,))
            break

    basic_robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
