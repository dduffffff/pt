# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class ZSL1RoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.40] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            # 'FL_ABAD_JOINT': 0.1,   # [rad]
            # 'RL_ABAD_JOINT': 0.1,   # [rad]
            # 'FR_ABAD_JOINT': -0.1 ,  # [rad]
            # 'RR_ABAD_JOINT': -0.1,   # [rad]

            # 'FL_HIP_JOINT': 0.8,     # [rad]
            # 'RL_HIP_JOINT': 1.,   # [rad]
            # 'FR_HIP_JOINT': 0.8,     # [rad]
            # 'RR_HIP_JOINT': 1.,   # [rad]

            # 'FL_KNEE_JOINT': -1.5,   # [rad]
            # 'RL_KNEE_JOINT': -1.5,    # [rad]
            # 'FR_KNEE_JOINT': -1.5,  # [rad]
            # 'RR_KNEE_JOINT': -1.5,    # [rad]
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 20.0}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        hip_reduction = 1.0

    class commands( LeggedRobotCfg.commands ):
            curriculum = True
            max_curriculum = 2.0
            num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
            resampling_time = 10. # time before command are changed[s]
            heading_command = False # if true: compute ang vel command from heading error
            class ranges( LeggedRobotCfg.commands.ranges):
                lin_vel_x = [-4.0, 4.0] # min max [m/s]
                lin_vel_y = [-2.0, 2.0]   # min max [m/s]
                ang_vel_yaw = [-3.14, 3.14]    # min max [rad/s]
                heading = [-3.14, 3.14]

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/zsl1/urdf/DOG.urdf'
        name = "DOG"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf", "base"]
        terminate_after_contacts_on = ["base"]
        privileged_contacts_on = ["base", "thigh", "calf"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False # Some .obj meshes must be flipped from y-up to z-up
  
    class rewards( LeggedRobotCfg.rewards ):
        class scales:
            # termination = -0.1 #-0.0
            # tracking_lin_vel = 2.0
            # tracking_ang_vel = 1.0
            # lin_vel_z = -2.0
            # ang_vel_xy = -0.05
            # orientation = -0.2
            # dof_acc = -2.5e-7
            # joint_power = -2e-5
            # base_height = -15.0
            # foot_clearance = -0.01
            # action_rate = -0.01
            # smoothness = -0.01
            # feet_air_time =  0.0
            # collision = -0.01
            # feet_stumble = -0.1
            # stand_still = -0.5
            # torques = -0.0002
            # dof_vel = -0.01
            # dof_pos_limits = -0.7 #-0.0
            # dof_vel_limits = -0.0003
            # torque_limits = -0.0
            # hip_same = -0.0001
            # hip_out=-1.0
            termination = -0.1        # 终止条件惩罚（如机器人摔倒）
            tracking_lin_vel = 2.0    # 线速度跟踪奖励（鼓励实现目标前进速度）
            tracking_ang_vel = 1.0    # 角速度跟踪奖励（鼓励实现目标转向速度）
            lin_vel_z = -2.0          # Z轴线速度惩罚（防止机器人跳跃/不稳定）
            ang_vel_xy = -0.05        # XY轴旋转惩罚（防止身体侧翻）
            orientation = -0.2        # 姿态稳定性惩罚（防止身体倾斜）
            dof_acc = -2.5e-7         # 关节加速度惩罚（减少关节抖动）
            joint_power = -2e-5       # 关节功率惩罚（降低能耗）
            base_height = -1.0       # 底盘高度偏离惩罚（保持标准高度）
            foot_clearance = -0.03    # 抬脚高度不足惩罚（防止拖地）
            action_rate = -0.01       # 动作突变惩罚（动作平滑性）
            smoothness = -0.001        # 运动平滑性惩罚（减少急停急动）
            feet_air_time =  0.0      # 脚掌悬空时间（当前无奖励/惩罚）
            collision = -0.01         # 身体碰撞惩罚
            feet_stumble = -0.1       # 脚掌打滑惩罚
            stand_still = -0.5        # 静止惩罚（鼓励持续运动）
            torques = -0.0002         # 关节扭矩惩罚（减小电机负载）
            dof_vel = -0.01           # 关节速度惩罚（防止过速）
            dof_pos_limits = -0.7     # 关节位置越界惩罚（保护机械结构）
            dof_vel_limits = -0.0003  # 关节速度越界惩罚
            torque_limits = -0.0      # 扭矩越界惩罚（当前未启用）
            hip_same = -0.0001        # 髋关节同向运动惩罚（鼓励交替迈步）
            hip_out = -1.0            # 髋关节外扩惩罚（保持腿部直立）
                        
        only_positive_rewards = False # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)
        soft_dof_pos_limit = 1. # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 1.
        soft_torque_limit = 1.
        base_height_target = 0.34 #0.30
        max_contact_force = 100. # forces above this value are penalized
        clearance_height_target = -0.20


class ZSL1RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_zsl1'

        num_steps_per_env = 100 # per iteration
        max_iterations = 200000 # number of policy updates

        # logging
        save_interval = 300 # check for potential saves every this many iterations

        # load and resume
        resume = False
        load_run = -1 # -1 = last run
        checkpoint = -1 # -1 = last saved model
        resume_path = None # updated from load_run and chkpt  