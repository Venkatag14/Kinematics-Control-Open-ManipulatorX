import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import JointtoEndEffector , EndEffectortoJoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray , String

import numpy as np
import math
import time

class VelocityConverterNode(Node) :
    def __init__(self):
        super().__init__('velocity_control_node')
        self.f = open('/home/venkata/Desktop/Save_XYZ.csv','w+')
        print('File created',self.f)
        self.check = False
        
        """
            Creating services to convert Joint to End Effector velocities AND to convert End Effector velocities to Joints .
        """

        self.sub = self.create_subscription(JointState, "joint_states", self.saveValues, 10)
        self.pub = self.create_publisher(Float64MultiArray , 'ee_jv_pub', 10)
        self.joint_to_ee_ser = self.create_service(JointtoEndEffector , "joint_to_end_effector" , self.Joint_To_EE)

        self.ee_to_joint_ser = self.create_service(EndEffectortoJoint , "end_effector_to_joint" , self.EE_To_Joint)
        self.xyz_data = self.create_subscription(String , 'fwd' , self.Save_data ,  10)
        

        
    def saveValues(self,JointState) :
        self.jv = JointState.position
        self.jvel = JointState.velocity

    def Save_data(self,msg) :
        self.check = True
        self.msg_xyz = msg


    def Joint_To_EE(self , request , response) :

        jv = self.jv
        jvel = self.jvel # Joint state velocity values
        # print(f'---')
        jv = np.degrees(jv)
        # print(f'-> {jv}')

        """
            DH Parameters
        """

        dh = [[jv[0], 96.326, 0, -90],
               [jv[1]-79.3809, 0, 130.230, 0],
               [79.3809+jv[2], 0, 124, 0],
               [jv[3], 0, 133.4, 0]]
        
        T = np.identity(4 , dtype = float)
        transf = []
        transf.append(np.identity(4 , dtype = float))
        for i in dh:
            theta = math.radians(i[0])
            d = i[1]
            a = i[2]
            alpha = math.radians(i[3])
            A = [
                    [np.cos(theta) , -np.sin(theta)*np.cos(alpha) , np.sin(theta)*np.sin(alpha) , a * np.cos(theta)],
                    [np.sin(theta) , np.cos(theta)*np.cos(alpha) , -np.cos(theta)*np.sin(alpha) , a * np.sin(theta)],
                    [0 , np.sin(alpha) , np.cos(alpha) , d],
                    [0 , 0 , 0 , 1]
                ]
            T = np.dot(T,A)
            transf.append(T)
        T = T.round(2)
        o4 = transf[-1].T[3][:3].T
        Jacob = []
        for i in range(len(dh)):
            z = transf[i].T[2][:3].T
            o = transf[i].T[3][:3].T
            lin = np.cross(z,(o4-o))
            ang = z
            a = [i for i in ang]
            b = [i for i in lin]
            a.extend(b)
            Jacob.append(a)
        Jacob = np.array(Jacob)
        Jacob = Jacob.T

        endvel = np.matmul(Jacob,jvel[:4])
        print(f'end vel : {endvel}')
        response.av_x, response.av_y, response.av_z, response.v_x, response.v_y, response.v_z = endvel
        # response.ee_velocities = endvel
        """
            Publishing the end_effector poses to the ros topic 'fwd'
        """
        # float64 av_x
        # float64 av_y
        # float64 av_z
        # float64 v_x
        # float64 v_y
        # float64 v_z

        # self.pub.publish(msg)
        return response

    def EE_To_Joint(self , request , response) :
        endvel =  [request.av_x, request.av_y, request.av_z, request.v_x, request.v_y, request.v_z]
        jv = self.jvel #JointState.position
        # jvel = JointState.velocity # Joint state position values
        # print(jv,jvel)
        # print(f'jv : {jv}')

        jv = np.degrees(jv)

        """
            DH Parameters
        """

        dh = [[jv[0], 96.326, 0, -90],
               [jv[1]-79.3809, 0, 130.230, 0],
               [79.3809+jv[2], 0, 124, 0],
               [jv[3], 0, 133.4, 0]]
        
        T = np.identity(4 , dtype = float)
        transf = []
        transf.append(np.identity(4 , dtype = float))
        for i in dh:
            theta = math.radians(i[0])
            d = i[1]
            a = i[2]
            alpha = math.radians(i[3])
            A = [
                [np.cos(theta) , -np.sin(theta)*np.cos(alpha) , np.sin(theta)*np.sin(alpha) , a * np.cos(theta)],
                [np.sin(theta) , np.cos(theta)*np.cos(alpha) , -np.cos(theta)*np.sin(alpha) , a * np.sin(theta)],
                [0 , np.sin(alpha) , np.cos(alpha) , d],
                [0 , 0 , 0 , 1]
                ]
            T = np.dot(T,A)
            transf.append(T)
        T = T.round(2)
        o4 = transf[-1].T[3][:3].T
        Jacob = []
        for i in range(len(dh)):
            z = transf[i].T[2][:3].T
            o = transf[i].T[3][:3].T
            lin = np.cross(z,(o4-o))
            ang = z
            a = [i for i in ang]
            b = [i for i in lin]
            a.extend(b)
            Jacob.append(a)
        Jacob = np.array(Jacob)
        Jacob = Jacob.T
        jvel = np.matmul(np.linalg.pinv(Jacob),endvel)
        msg = Float64MultiArray()
        msg.data = [i for i in jvel]
        response.j_v1,response.j_v2,response.j_v3 ,response.j_v4 = msg.data

        self.f.write(f'{self.msg_xyz.data},{time.time()}\n')
        self.pub.publish(msg)

        return response


def main():
    rclpy.init()
    vconv = VelocityConverterNode()
    rclpy.spin(vconv)
    # vconv.destroynode()
    vconv.f.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()