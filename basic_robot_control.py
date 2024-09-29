import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
import time
from std_msgs.msg import Float64MultiArray
import math


class BasicRobotControl(Node):
    def __init__(self):
        super().__init__('basic_robot_control')

        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control') # Creating client to control gripper
        self.int_pose_sub = self.create_subscription(Float64MultiArray , 'init_pose_pub' , self.SetInitPose , 10)
        self.sub = self.create_subscription(Float64MultiArray , 'updated_joint_vel' , self.updatePosition , 10)

        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        
        # self.send_request()

    def SetInitPose(self,msg) :
        request = SetJointPosition.Request()
        request.planning_group = ''
        q1 , q2 , q3 , q4 = msg.data
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        print(f'Init Poses : ',math.degrees(q1),math.degrees(q2),math.degrees(q3),math.degrees(q4))
        request.joint_position.position = [q1, q2, q3, q4, 0.0]
        path_time = 10.0
        request.path_time = path_time
        self.future = self.client.call_async(request) 

    def updatePosition(self,msg) :
        request = SetJointPosition.Request()
        request.planning_group = ''
        q1 , q2 , q3 , q4 = msg.data
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        print(math.degrees(q1),math.degrees(q2),math.degrees(q3),math.degrees(q4))
        request.joint_position.position = [q1, q2, q3, q4, 0.0]
        path_time = 0.1
        request.path_time = path_time

        self.future = self.client.call_async(request)

    def send_request(self):

        """
            Passing arguments from the terminal to move the robot to a certain position .
        """
        # x,y,z,angle,gripper = [float(sys.argv[1]) , 
        #                float(sys.argv[2]) , 
        #                float(sys.argv[3]), 
        #                float(sys.argv[4]),
        #                float(sys.argv[5])]

        # positions = [0,0,0,0,0]

        # q1 , q2 , q3 , q4 , g = [float(sys.argv[1]) , 
        #                float(sys.argv[2]) , 
        #                float(sys.argv[3]), 
        #                float(sys.argv[4]),
        #                float(sys.argv[5])]
        
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        print(math.degrees(q1),math.degrees(q2),math.degrees(q3),math.degrees(q4))
        request.joint_position.position = [q1, q2, q3, q4, g]
        path_time = 5.0
        request.path_time = path_time

        self.future = self.client.call_async(request)

        """
            Positions to move the robot to grip and drop the object from known position.
        """

        # positions = [[150,0,50,80,0.01],
        #              [150,100,100,80,0.01],
        #              [150,100,50,80,0.01],
        #              [150,100,50,80,0.0],
        #              [150,100,100,80,0.0],
        #              [150,-100,100,80,0.0],
        #              [150,-100,50,80,0.0],
        #              [150,-100,50,80,0.01],
        #              [150,-100,100,80,0.01],
        #              [150,0,50,80,0.01],]
    
        # for x,y,z,angle,gripper in positions:

        #     """
        #         Inverse Kinematics 
        #     """

        #     l1= 96.326
        #     ld = 130.230
        #     l3 = 124
        #     l4 = 133.4

        #     angle = math.radians(angle) # angle to decide the orientation of gripper w.r.t X-axis

        #     z1 = z+l4*math.sin(angle)
        #     diag = math.sqrt(x**2+y**2)-l4*math.cos(angle)
        #     q1 = math.atan2(y,x) # Angle of Joint 1
        #     x1 = diag*math.sin(q1)
        #     y1 = diag*math.cos(q1)

        #     offset = z1-l1
        #     th_side = math.sqrt(offset**2+diag**2)
        #     b = math.acos((l3**2+ld**2-th_side**2)/(2*l3*ld))
        #     q3 = math.radians(100.619199)-b # Angle of Joint 3

        #     a = math.acos((ld**2+th_side**2-l3**2)/(2*ld*th_side))
        #     q2 = math.radians(90)-math.atan2(24,128)-a-math.atan2(offset,diag) # Angle of Joint 2
            
        #     q4 = angle-q2-q3 # Angle of Joint 4

        #     """
        #         request => to set the angles for first 4 robots .
        #     """

        #     request = SetJointPosition.Request()
        #     request.planning_group = ''
        #     request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        #     print(math.degrees(q1),math.degrees(q2),math.degrees(q3),math.degrees(q4))
        #     request.joint_position.position = [q1, q2, q3, q4, gripper]
        #     path_time = 5.0
        #     request.path_time = path_time

        #     """
        #         self.tool_control_req => to control the gripper .
        #     """
        #     self.tool_control_req = SetJointPosition.Request()
        #     self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        #     self.tool_control_req.joint_position.position = [q1, q2, q3, q4, gripper]
        #     self.tool_control_req.path_time = path_time


        #     """
        #         Setting positions of the robot
        #     """
        #     self.future = self.client.call_async(request)
        #     self.future = self.tool_control.call_async(self.tool_control_req)

        #     time.sleep(5)

        pass

def main(args=None):
    rclpy.init(args=args)

    basic_robot_control = BasicRobotControl()

    while rclpy.ok():
        rclpy.spin_once(basic_robot_control)
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
