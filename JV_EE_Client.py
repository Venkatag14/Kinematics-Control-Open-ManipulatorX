
import rclpy
from rclpy.node import Node
# from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import InvService
from open_manipulator_msgs.srv import JointtoEndEffector
import sys

class Client_(Node) :
    def __init__(self) :
        super().__init__('client_')
        self.client = self.create_client(JointtoEndEffector , "joint_to_end_effector" )
        self.res = self.send_request()
    
    def send_request(self) :

        # Raising request

        request = JointtoEndEffector.Request()
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()

def main() :
    rclpy.init()
    client = Client_()

    eVel = [client.res.av_x, client.res.av_y, client.res.av_z, client.res.v_x, client.res.v_y, client.res.v_z]

    client.get_logger().info(
        f'End Effector Velocities : {eVel}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" :
    main()