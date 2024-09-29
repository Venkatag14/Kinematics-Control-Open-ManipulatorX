
import rclpy
from rclpy.node import Node
# from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import InvService
from open_manipulator_msgs.srv import JointtoEndEffector , EndEffectortoJoint
import sys
import time

class Client_(Node) :
    def __init__(self) :
        super().__init__('client_')
        self.client = self.create_client(EndEffectortoJoint , "end_effector_to_joint" )
    
    def send_request(self,endEffVel) :

        # Raising request

        request = EndEffectortoJoint.Request()
        request.av_x,request.av_y,request.av_z,request.v_x,request.v_y,request.v_z = endEffVel

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()

def main() :
    rclpy.init()
    client = Client_()
    endEffVel = [float(sys.argv[1]), float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6])]
    run_time = float(sys.argv[7])
    ini_time = time.time()
    while True :
        res = client.send_request(endEffVel)
        if time.time() - ini_time >= run_time :
            break
    
    eVel = [res.j_v1, res.j_v2, res.j_v3, res.j_v4]

    client.get_logger().info(
        f'Joint Velocities : {eVel}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" :
    main()