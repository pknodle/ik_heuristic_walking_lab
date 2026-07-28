"""Part 2: Heuristic walking - a trotting gait on all four legs.

Run the controller stack first (separate terminal):
    ros2 launch part_2.launch.py
Then run this node:
    python3 part_2_walking.py

All twelve joints are commanded in Part 2 (see part_2.yaml).

Building the joint cache runs your gradient-descent IK 200 times and takes a few
seconds. Once your gait is working you can save and reuse it:

    python3 part_2_walking.py --save-cache   # compute, then write the cache to disk
    python3 part_2_walking.py --use-cache    # skip the solve and load it back

Rebuild the cache (plain run, or --save-cache) any time you change the keyframes.
"""

import argparse
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(precision=3, suppress=True)

CACHE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'joint_positions_cache.npz')


def rotation_x(angle):
    ################################################################################################
    # TODO 7: [already done in Part 1] paste your forward kinematics helpers here
    ################################################################################################
    return

def rotation_y(angle):
    ################################################################################################
    # TODO 7: [already done in Part 1] paste your forward kinematics helpers here
    ################################################################################################
    return

def rotation_z(angle):
    ################################################################################################
    # TODO 7: [already done in Part 1] paste your forward kinematics helpers here
    ################################################################################################
    return

def translation(x, y, z):
    ################################################################################################
    # TODO 7: [already done in Part 1] paste your forward kinematics helpers here
    ################################################################################################
    return


class InverseKinematics(Node):

    def __init__(self, use_cache=False, save_cache=False):
        super().__init__('inverse_kinematics')
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

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None
        self.counter = 0

        # Keyframes of a single leg's cycle, in the leg's own frame. The per-leg
        # offsets below place them under the correct hip.
        touch_down_position = np.array([0.05, 0.0, -0.14])
        stand_position_1 = np.array([0.025, 0.0, -0.14])
        stand_position_2 = np.array([0.0, 0.0, -0.14])
        stand_position_3 = np.array([-0.025, 0.0, -0.14])
        liftoff_position = np.array([-0.05, 0.0, -0.14])
        mid_swing_position = np.array([0.0, 0.0, -0.05])

        ## trotting
        # TODO 9: Implement each leg's trajectory in the trotting gait.
        rf_ee_offset = np.array([0.06, -0.09, 0])
        rf_ee_triangle_positions = np.array([
            ################################################################################################
            # TODO 9: Implement the trotting gait
            ################################################################################################
        ]) + rf_ee_offset

        lf_ee_offset = np.array([0.06, 0.09, 0])
        lf_ee_triangle_positions = np.array([
            ################################################################################################
            # TODO 9: Implement the trotting gait
            ################################################################################################
        ]) + lf_ee_offset

        rb_ee_offset = np.array([-0.11, -0.09, 0])
        rb_ee_triangle_positions = np.array([
            ################################################################################################
            # TODO 9: Implement the trotting gait
            ################################################################################################
        ]) + rb_ee_offset

        lb_ee_offset = np.array([-0.11, 0.09, 0])
        lb_ee_triangle_positions = np.array([
            ################################################################################################
            # TODO 9: Implement the trotting gait
            ################################################################################################
        ]) + lb_ee_offset

        self.ee_triangle_positions = [rf_ee_triangle_positions, lf_ee_triangle_positions, rb_ee_triangle_positions, lb_ee_triangle_positions]
        self.fk_functions = [self.fr_leg_fk, self.fl_leg_fk, self.br_leg_fk, self.bl_leg_fk]

        if use_cache:
            self.target_joint_positions_cache, self.target_ee_cache = self.load_cached_joint_positions()
        else:
            self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()
            if save_cache:
                self.save_cached_joint_positions()
        print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
        print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')

        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 100  # 100 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

    def fr_leg_fk(self, theta):
        # Already implemented in Lab 2
        T_RF_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def fl_leg_fk(self, theta):
        ################################################################################################
        # TODO 8: implement forward kinematics here
        ################################################################################################
        return

    def br_leg_fk(self, theta):
        ################################################################################################
        # TODO 8: implement forward kinematics here
        ################################################################################################
        return

    def bl_leg_fk(self, theta):
        ################################################################################################
        # TODO 8: implement forward kinematics here
        ################################################################################################
        return

    def forward_kinematics(self, theta):
        return np.concatenate([self.fk_functions[i](theta[3*i: 3*i+3]) for i in range(4)])

    def listener_callback(self, msg):
        joints_of_interest = [
            'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3',
            'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3',
            'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3',
            'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3'
        ]
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def inverse_kinematics_single_leg(self, target_ee, leg_index, initial_guess=[0, 0, 0]):
        leg_forward_kinematics = self.fk_functions[leg_index]

        def cost_function(theta):
            current_position = leg_forward_kinematics(theta)
            ################################################################################################
            # TODO 7: [already done in Part 1] paste your inverse kinematics here
            ################################################################################################
            return None, None

        def gradient(theta, epsilon=1e-3):
            grad = np.zeros(3)
            ################################################################################################
            # TODO 7: [already done in Part 1] paste your inverse kinematics here
            ################################################################################################
            return grad

        theta = np.array(initial_guess).astype(np.float64)
        learning_rate = None # TODO 7: [already done in Part 1] paste your inverse kinematics here
        max_iterations = None # TODO 7: [already done in Part 1] paste your inverse kinematics here
        tolerance = None # TODO 7: [already done in Part 1] paste your inverse kinematics here

        cost_l = []
        for _ in range(max_iterations):
            ################################################################################################
            # TODO 7: [already done in Part 1] paste your inverse kinematics here
            ################################################################################################
            continue

        return theta

    def interpolate_triangle(self, t, leg_index):
        ################################################################################################
        # TODO 10: implement interpolation for all 4 legs here
        # Unlike Part 1, t is a float between 0 and 1 covering one full gait cycle, and each leg has
        # six keyframes instead of three.
        ################################################################################################

        return

    def cache_target_joint_positions(self):
        # Calculate and store the target joint positions for a cycle and all 4 legs
        target_joint_positions_cache = []
        target_ee_cache = []
        for leg_index in range(4):
            target_joint_positions_cache.append([])
            target_ee_cache.append([])
            target_joint_positions = [0] * 3
            for t in np.arange(0, 1, 0.02):
                print(f'Leg {leg_index}, t={t:.2f}')
                target_ee = self.interpolate_triangle(t, leg_index)
                target_joint_positions = self.inverse_kinematics_single_leg(target_ee, leg_index, initial_guess=target_joint_positions)

                target_joint_positions_cache[leg_index].append(target_joint_positions)
                target_ee_cache[leg_index].append(target_ee)

        # (4, 50, 3) -> (50, 12)
        target_joint_positions_cache = np.concatenate(target_joint_positions_cache, axis=1)
        target_ee_cache = np.concatenate(target_ee_cache, axis=1)

        return target_joint_positions_cache, target_ee_cache

    def save_cached_joint_positions(self):
        np.savez(CACHE_FILE,
                 target_joint_positions=self.target_joint_positions_cache,
                 target_ee=self.target_ee_cache)
        print(f'Saved joint position cache to {CACHE_FILE}')

    def load_cached_joint_positions(self):
        # Load a previously saved cache. Regenerate it whenever you change the keyframes!
        try:
            data = np.load(CACHE_FILE)
        except FileNotFoundError:
            print(f'No cache at {CACHE_FILE}. Run with --save-cache once to create it.')
            raise
        print(f'Loaded cached joint positions from {CACHE_FILE}')
        return data['target_joint_positions'], data['target_ee']

    def get_target_joint_positions(self):
        target_joint_positions = self.target_joint_positions_cache[self.counter]
        target_ee = self.target_ee_cache[self.counter]
        self.counter += 1
        if self.counter >= self.target_joint_positions_cache.shape[0]:
            self.counter = 0
        return target_ee, target_joint_positions

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            target_ee, self.target_joint_positions = self.get_target_joint_positions()
            current_ee = self.forward_kinematics(self.joint_positions)

            self.get_logger().info(
                f'Target EE: {target_ee}, \
                Current EE: {current_ee}, \
                Target Angles: {self.target_joint_positions}, \
                Target Angles to EE: {self.forward_kinematics(self.target_joint_positions)}, \
                Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:
            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--use-cache', action='store_true',
                        help='load the joint cache from disk instead of solving IK')
    parser.add_argument('--save-cache', action='store_true',
                        help='solve IK, then save the joint cache to disk')
    args, _ = parser.parse_known_args()

    rclpy.init()
    inverse_kinematics = InverseKinematics(use_cache=args.use_cache, save_cache=args.save_cache)

    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0] * 12
        inverse_kinematics.command_publisher.publish(zero_torques)

        inverse_kinematics.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
