"""Part 1: Inverse kinematics and trajectory tracking on a single leg.

Run the controller stack first (separate terminal):
    ros2 launch part_1.launch.py
Then run this node:
    python3 part_1_ik.py

Only the three front-right joints are commanded in Part 1 (see part_1.yaml).
"""
#shivam chopra test github
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import pdb

# Gains for the on-board joint controller. We command positions here; the motor
# controller closes the PD loop around them.
Kp = 3
Kd = 0.1


class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.t = 0
       # print("hello")
        self.ee_triangle_positions = np.array([
            [0.05, 0.0, -0.12],  # Touchdown
            [-0.05, 0.0, -0.12], # Liftoff
            [0.0, 0.0, -0.06]    # Mid-swing
        ])
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )

        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 20   # 20 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None

        

        center_to_rf_hip = np.array([0.07500, -0.08350, 0])
        self.ee_triangle_positions = self.ee_triangle_positions + center_to_rf_hip
        self.current_target = 0
        

    def listener_callback(self, msg):
        joints_of_interest = ['leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3']
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def forward_kinematics(self, theta1, theta2, theta3):

        def rotation_x(angle):
            # rotation about the x-axis implemented for you
            return np.array(
                [
                    [1, 0, 0, 0],
                    [0, np.cos(angle), -np.sin(angle), 0],
                    [0, np.sin(angle), np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_y(angle):
            #rotation about y axis
           return np.array(
                [
                    [np.cos(angle),0, np.sin(angle), 0],
                    [0, 1, 0, 0],
                    [-np.sin(angle),0, np.cos(angle), 0],
                    [0, 0, 0, 1],
                ])
                

        def rotation_z(angle):
            ## TODO: Implement the rotation matrix about the z-axis
             return np.array([     
                    [np.cos(angle), -np.sin(angle),0, 0],
                    [np.sin(angle), np.cos(angle),0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                    ])
                


        def translation(x, y, z):
            ## TODO: Implement the translation matrix
            return np.array([
                    [1,0, 0, x],
                    [0, 1,0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1],
                    ])
            

        # T_0_1 (base_link to leg_front_l_1)
        T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(-1.57080) @ rotation_z(-theta1)

        # T_1_2 (leg_front_l_1 to leg_front_l_2)
        ## TODO: Implement the transformation matrix from leg_front_l_1 to leg_front_l_2
        T_1_2 = translation(0,0,-0.039) @rotation_y(-np.pi/2) @rotation_z(theta2)

        # T_2_3 (leg_front_l_2 to leg_front_l_3)
        ## TODO: Implement the transformation matrix from leg_front_l_2 to leg_front_l_3
        T_2_3 = translation(0,0.0494,0.0685) @rotation_y(np.pi/2) @rotation_z(-theta3)

        # T_3_ee (leg_front_l_3 to end-effector)
        T_3_ee = translation(.06321,0.06216,-0.018)
        debug = translation(.06321,0.06216,0)
        # TODO: Compute the final transformation. T_0_ee is the multiplication of the previous transformation matrices
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        # TODO: Extract the end-effector position. The end effector position is a 3x1 vector (not in homogenous coordinates)
        end_effector_position = T_0_ee @[0,0,0,1]

        return end_effector_position[0:3]

    def inverse_kinematics(self, target_ee, initial_guess=[0, 0, 0]):
        def cost_function(theta):
            # Compute the cost function and the squared L2 norm of the error
            # return the cost and the squared L2 norm of the error
            error = np.abs(target_ee - self.forward_kinematics(*theta))
            L2_norm = np.linalg.norm(error)
            cost = (L2_norm)**2
            ################################################################################################
            # TODO 2: Implement the cost function
            # HINT: You can use the * notation on a list to "unpack" a list
            ################################################################################################
            return cost, L2_norm

        def gradient(theta, epsilon=1e-3):
            # Compute the gradient of the cost function using finite differences
            ################################################################################################
            # TODO 3: Implement the gradient computation
            ################################################################################################
            grad = np.zeros(len(theta))
            cost,_ = cost_function(theta)
            for i in range(len(theta)):
                nudged_theta = theta.copy()
                nudged_theta[i]= nudged_theta[i]+epsilon 
                nudged_cost,_ = cost_function(nudged_theta) 
                grad[i] = (nudged_cost - cost)/epsilon
            return grad

        theta = np.array(initial_guess)
        learning_rate = 5 # TODO 4: Set the learning rate
        max_iterations = 50 # TODO 4: Set the maximum number of iterations
        tolerance = 1e-3 # TODO 4: Set the tolerance for the L1 norm of the error

        cost_l = []
        for _ in range(max_iterations):
            grad = gradient(theta)

            # Update the theta (parameters) using the gradient and the learning rate
            ################################################################################################
            # TODO 4: Implement the gradient update. Use the cost function you implemented, and use tolerance
            # to determine if IK has converged
            theta = theta - learning_rate* grad

            cost,_ = cost_function(theta)
            cost_l.append(cost)
            error = target_ee - self.forward_kinematics(*theta)
            if (abs(error[0])+abs(error[1])+abs(error[2]))<tolerance:
                break
            # TODO (BONUS): Implement the (quasi-)Newton's method instead of finite differences for faster
            # convergence
            ################################################################################################

        #print(f'Cost: {cost_l}') # Use to debug to see if your cost function converges within max_iterations

        return theta

    def interpolate_triangle(self, t):
        # Interpolate between the three triangle positions in self.ee_triangle_positions
        # based on the current time t
        ################################################################################################
        # TODO 5: Implement the interpolation function
        ################################################################################################
        t = t%3
        start = None
        end = None
        v = self.ee_triangle_positions
        if t<1:
            start = v[0]
            end = v[1]
        elif t<2:
            start = v[1]
            end = v[2]
        else:
            start = v[2]
            end = v[0]
         
        return start + (end - start)*(t%1)

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            target_ee = self.interpolate_triangle(self.t)
            self.target_joint_positions = self.inverse_kinematics(target_ee, self.joint_positions)
            current_ee = self.forward_kinematics(*self.joint_positions)
            
            # update the current time for the triangle interpolation
            ################################################################################################
            # TODO 6: Implement the time update
            ################################################################################################
            self.t = self.t + self.ik_timer_period
            self.get_logger().info(f'Target EE: {target_ee}, Current EE: {current_ee}, Target Angles: {self.target_joint_positions}, Target Angles to EE: {self.forward_kinematics(*self.target_joint_positions)}, Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:

            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)


def main():
    rclpy.init()
    inverse_kinematics = InverseKinematics()

    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0, 0.0, 0.0]
        #inverse_kinematics.command_publisher.publish(zero_torques)

        inverse_kinematics.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
