import numpy as np
from scipy.spatial.transform import Rotation
import pybullet
import sys
from copy import deepcopy
from HandState import HandState, HAND_TYPE_LEFT, HAND_TYPE_RIGHT

class IKBaseTransform():
    def __init__(self, pos, rotmat, inv_rotmat):
        self.ik_base_pos = pos
        self.ik_base_rotmat = rotmat
        self.ik_base_inv_rotmat = inv_rotmat
    
    def transformToIKBase(self, pos : np.ndarray, rot : np.ndarray):
        tf_pos = self.ik_base_inv_rotmat @ (pos - self.ik_base_pos)
        tf_rotmat = self.ik_base_inv_rotmat @ Rotation.from_quat(rot).as_matrix()
        return tf_pos, Rotation.from_matrix(tf_rotmat).as_quat()

    def transformToWorld(self, pos : np.ndarray, rot : np.ndarray):
        tf_pos = self.ik_base_rotmat @ pos + self.ik_base_pos
        tf_rotmat = self.ik_base_rotmat @ Rotation.from_quat(rot).as_matrix()
        return tf_pos, Rotation.from_matrix(tf_rotmat).as_quat()
    
    def transformToWorldPos(self, pos : np.ndarray):
        return self.ik_base_rotmat @ pos + self.ik_base_pos
    
    def transformToWorldDir(self, dir : np.ndarray):
        return self.ik_base_rotmat @ dir

class PepperArmIK:
    def __init__(self, operator_scale, max_joint_speed, pepper, use_real_robot, qi_motion_service=None, 
                 show_markers=True, show_left_ik=False, show_right_ik=False, show_status=False):
        # Configuration checks
        if use_real_robot and not qi_motion_service:
            sys.exit("ERROR: PepperArmIK requires a qi_motion_service object when use_real_robot is True!")
        if not use_real_robot and not pepper:
            sys.exit("ERROR: PepperArmIK requires a pepper object when use_real_robot is False!")
        if (show_markers or show_left_ik or show_right_ik or show_status) and not pepper:
            print("WARNING: PepperArmIK needs a pepper object when any of show_*** are True to display debug visuals.")
            print("WARNING: Continuing without debug visuals in PepperArmIK.")
            show_markers, show_left_ik, show_right_ik, show_status = False, False, False, False
        
        # Robot/simulation interface
        self.use_real_robot = use_real_robot
        self.pepper = pepper
        self.qi_motion_service = qi_motion_service

        # Debug drawing
        self.show_markers = show_markers
        self.show_left_ik = show_left_ik
        self.show_right_ik = show_right_ik
        self.show_status = show_status
        self.sim_arm_debug_ids = {
            HAND_TYPE_LEFT : [-1 for _ in range(15)],
            HAND_TYPE_RIGHT : [-1 for _ in range(15)]
        }
        
        # Geometric robot properties
        self.operator_scale = operator_scale
        self.upper_arm_length = np.sqrt(0.1812**2 + 0.015**2)
        self.palm_wrist_x_offset = np.float64(0.072)
        self.lower_arm_length = np.float64(0.15) + self.palm_wrist_x_offset
        self.shoulder_frame_offset = {
            HAND_TYPE_LEFT : Rotation.from_rotvec([0, 0, -1 * np.atan(0.015 / 0.1812)]).as_matrix(),
            HAND_TYPE_RIGHT : Rotation.from_rotvec([0, 0, np.atan(0.015 / 0.1812)]).as_matrix()
        }
        self.elbow_frame_offset = Rotation.from_rotvec([0, 0.157079, 0]).as_matrix()
        self.max_shoulder_target_dist = np.sqrt(self.lower_arm_length**2 + self.upper_arm_length**2 - 2 * self.lower_arm_length * 
                                                self.upper_arm_length * np.cos(np.pi - 0.16))
        self.shoulder_position = { # Reference point is center between shoulders
            HAND_TYPE_LEFT : np.array([0, 0.14974, 0]),
            HAND_TYPE_RIGHT : np.array([0, -0.14974, 0])
        }
        # self.palm_wrist_offset = np.array([0.072, 0, -0.013])
        
        # Robot joint properties
        self.arm_joint_names = {
            HAND_TYPE_LEFT : ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"],
            HAND_TYPE_RIGHT : ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        }
        self.wrist_joint_name = {
            HAND_TYPE_LEFT : "LWristYaw",
            HAND_TYPE_RIGHT : "RWristYaw"
        }
        self.gripper_joint_name = {
            HAND_TYPE_LEFT : "LHand",
            HAND_TYPE_RIGHT : "RHand"
        }
        if self.pepper:
            self.arm_joint_indices = {
                HAND_TYPE_LEFT : [self.pepper.joint_dict[n].getIndex() for n in self.arm_joint_names[HAND_TYPE_LEFT]],
                HAND_TYPE_RIGHT : [self.pepper.joint_dict[n].getIndex() for n in self.arm_joint_names[HAND_TYPE_RIGHT]]
            }
            self.wrist_joint_index = {
                HAND_TYPE_LEFT : self.pepper.joint_dict["LWristYaw"].getIndex(),
                HAND_TYPE_RIGHT : self.pepper.joint_dict["RWristYaw"].getIndex()
            }
            self.gripper_joint_index = {
                HAND_TYPE_LEFT : self.pepper.joint_dict["LHand"].getIndex(),
                HAND_TYPE_RIGHT : self.pepper.joint_dict["RHand"].getIndex()
            }
            self.hip_joint_index = self.pepper.joint_dict["HipRoll"].getIndex()
        self.arm_joint_limits = {
            HAND_TYPE_LEFT : np.array([[-2.08567, 0.00872665, -2.08567, -1.56207, -1.82387], 
                                       [2.08567, 1.56207, 2.08567, -0.00872665, 1.82387]]),
            HAND_TYPE_RIGHT : np.array([[-2.08567, -1.56207, -2.08567, 0.00872665, -1.82387], 
                                        [2.08567, -0.00872665, 2.08567, 1.56207, 1.82387]])
        }
        self.arm_joint_angles = {
            HAND_TYPE_LEFT : np.average(self.arm_joint_limits[HAND_TYPE_LEFT], axis=1),
            HAND_TYPE_RIGHT : np.average(self.arm_joint_limits[HAND_TYPE_RIGHT], axis=1)
        }
        max_joint_speed = max(min(max_joint_speed, 1.0), 0.0)
        self.max_joint_speeds_sim = [max_joint_speed for _ in range(5)]
        self.max_joint_speeds_real_robot = [max_joint_speed * 0.2 for _ in range(5)] # Real robot and sim joint speeds don't match

        # Query initial robot hand states
        self.robot_hand_state = {
            HAND_TYPE_LEFT : HandState(HAND_TYPE_LEFT), 
            HAND_TYPE_RIGHT : HandState(HAND_TYPE_RIGHT)
        }
        self.updateRobotState()

        # Set initial target hand states equal to initial robot hand states
        self.target_hand_state = {
            HAND_TYPE_LEFT : HandState(HAND_TYPE_LEFT), 
            HAND_TYPE_RIGHT : HandState(HAND_TYPE_RIGHT)
        }
        self.setTargetHandState(self.robot_hand_state[HAND_TYPE_LEFT], False)
        self.setTargetHandState(self.robot_hand_state[HAND_TYPE_RIGHT], False)

        # Spawn hand markers (if enabled)
        if show_markers:
            self.sim_hand_marker_ids = [self.spawnSimulationMarkers() for _ in range(4)]
            self.updateSimulationHandMarkers()
    
    def computeIKBaseTransform(self, for_real_robot):
        if for_real_robot:
            # Compute reference frame of IK -> center between shoulders in hip orientation
            x, y, z, wx, wy, wz = self.qi_motion_service.getPosition("HipRoll", 0, True)
            pos = np.array([x, y, z])
            ik_base_rotation = Rotation.from_euler('zyx', [wz, wy, wx])
        else:
            # Compute reference frame -> center between shoulders in hip orientation
            hip_state = pybullet.getLinkState(bodyUniqueId=self.pepper.robot_model, linkIndex=self.hip_joint_index)
            pos = np.array(hip_state[4])
            ik_base_rotation = Rotation.from_quat(hip_state[5])
        inv_rotmat = ik_base_rotation.inv().as_matrix()
        rotmat = ik_base_rotation.as_matrix()
        pos = pos + rotmat @ np.array([-0.057, 0, 0.22582])
        return IKBaseTransform(pos, rotmat, inv_rotmat)    

    def _getHandState(self, state : HandState, in_operator_scale : bool):
        copied_state = deepcopy(state)
        if in_operator_scale:
            # Correct position by remote operator scale
            shoulder_pos = self.shoulder_position[copied_state.hand_type]
            delta_pos = copied_state.palm_pos - shoulder_pos
            scaled_pos = shoulder_pos + delta_pos * self.operator_scale
            copied_state.palm_pos = scaled_pos
        return copied_state
    
    def getRobotHandState(self, hand_type, in_operator_scale : bool):
        return self._getHandState(self.robot_hand_state[hand_type], in_operator_scale)
    
    def getTargetHandState(self, hand_type, in_operator_scale : bool):
        return self._getHandState(self.target_hand_state[hand_type], in_operator_scale)

    def updateRobotState(self):
        # Compute robot frame <-> ik base transform (if only simulation is used, the robot frame is the sim world frame)
        robot_base_ik_transfrom = self.computeIKBaseTransform(self.use_real_robot)

        for hand_type, hand_state in self.robot_hand_state.items():
            # Query robot hand state and joint angles
            if self.use_real_robot:
                x, y, z, wx, wy, wz = self.qi_motion_service.getPosition(self.wrist_joint_name[hand_type], 0, True)
                pos = np.array([x, y, z])
                rot = Rotation.from_euler('ZYX', [wz, wy, wx]).as_quat()
                #ik_hand.state.grip_strength, _, _, _, _, _ = self.qi_motion_service.getPosition(ik_hand.gripper_name, 0, True)
            else:
                hand_pose = pybullet.getLinkState(bodyUniqueId=self.pepper.robot_model, linkIndex=self.wrist_joint_index[hand_type])
                pos = np.array(hand_pose[4])
                rot = np.array(hand_pose[5])
                hand_state.grip_strength = pybullet.getJointState(bodyUniqueId=self.pepper.robot_model, jointIndex=self.gripper_joint_index[hand_type])[0]
                joint_states = pybullet.getJointStates(bodyUniqueId=self.pepper.robot_model, jointIndices=self.arm_joint_indices[hand_type])
                self.arm_joint_angles[hand_type] = np.array([js[0] for js in joint_states])
            
            # Adjust hand position to be at robot's palm (where the target hand position is located)
            pos += Rotation.from_quat(rot).as_matrix() @ np.array((self.palm_wrist_x_offset, 0, 0))
            
            # Received hand state is in robot's own frame. Convert to ik base frame and store.
            hand_state.palm_pos, hand_state.palm_rot = robot_base_ik_transfrom.transformToIKBase(pos, rot)
    
    def setTargetHandState(self, new_hand_state : HandState, is_operator_scale : bool):
        # Received hand state is in base ik frame.
        self.target_hand_state[new_hand_state.hand_type] = deepcopy(new_hand_state)
        self.clipAndApplyHandGripStrength(new_hand_state.hand_type)

        if is_operator_scale:
            # Correct position by remote operator scale
            shoulder_pos = self.shoulder_position[new_hand_state.hand_type]
            delta_pos = new_hand_state.palm_pos - shoulder_pos
            scaled_pos = shoulder_pos + delta_pos / self.operator_scale
            self.target_hand_state[new_hand_state.hand_type].palm_pos = scaled_pos
    
    def changeTargetHandState(self, delta_pos : np.ndarray, delta_euler_rot: np.ndarray, delta_grip, hand_type):
        state = self.target_hand_state[hand_type]
        state.palm_pos += delta_pos
        new_euler_rot = np.array(pybullet.getEulerFromQuaternion(list(state.palm_rot))) + delta_euler_rot
        state.palm_rot = np.array(pybullet.getQuaternionFromEuler(list(new_euler_rot)))
        state.grip_strength += delta_grip
        self.clipAndApplyHandGripStrength(hand_type)

    def spawnSimulationMarkers(self):
        # Coordinate-sytem shaped marker
        x_axis_shape = pybullet.createVisualShape(
            shapeType=pybullet.GEOM_BOX,
            rgbaColor=[1, 0, 0, 1],
            halfExtents=[0.05, 0.005, 0.005]
        )
        y_axis_shape = pybullet.createVisualShape(
            shapeType=pybullet.GEOM_BOX,
            rgbaColor=[0, 1, 0, 1],
            halfExtents=[0.005, 0.05, 0.005]
        )
        z_axis_shape = pybullet.createVisualShape(
            shapeType=pybullet.GEOM_BOX,
            rgbaColor=[0, 0, 1, 1],
            halfExtents=[0.005, 0.005, 0.05]
        )
        marker_id = pybullet.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=x_axis_shape,
            basePosition=[0.015, 0, 0],
            baseOrientation=[0, 0, 0, 1],
            linkMasses=[0, 0],
            linkCollisionShapeIndices=[-1, -1],
            linkVisualShapeIndices=[y_axis_shape, z_axis_shape],
            linkPositions=[[0, 0.025, 0], [0, 0, 0.025]],
            linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
            linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],
            linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
            linkParentIndices=[0, 0],
            linkJointTypes=[pybullet.JOINT_FIXED, pybullet.JOINT_FIXED],
            linkJointAxis=[[0, 0, 0], [0, 0, 0]]
        )
        return marker_id

    def updateSimulationHandMarkers(self):
        if self.show_markers:
            # Compute simulation world <-> ik base transform
            sim_base_ik_transfrom = self.computeIKBaseTransform(for_real_robot=False)

            # Transform all hand poses to sim world frame and update marker poses
            hand_states = list(self.robot_hand_state.values()) + list(self.target_hand_state.values())
            for state, marker_id in zip(hand_states, self.sim_hand_marker_ids):
                pos, rot = sim_base_ik_transfrom.transformToWorld(state.palm_pos, state.palm_rot)
                pybullet.resetBasePositionAndOrientation(
                    bodyUniqueId=marker_id,
                    posObj=list(pos),
                    ornObj=list(rot)
                )

    def clipAndApplyHandGripStrength(self, hand_type):
        state = self.target_hand_state[hand_type]
        state.grip_strength = max(min(state.grip_strength, 1.0), 0.0)
        if self.use_real_robot:
            self.qi_motion_service.setAngles(self.gripper_joint_name[hand_type], state.grip_strength, self.max_joint_speeds_sim[0])
        if self.pepper:
            self.pepper.setAngles(self.gripper_joint_name[hand_type], state.grip_strength, self.max_joint_speeds_real_robot[0])

    def computeJointSpeeds(self, new_joint_angles):
        pass

    def computeAndApplyIK(self, hand_type):
        # Adjust target pose to be located at hand wrist for IK computations
        target_pos = np.copy(self.target_hand_state[hand_type].palm_pos)
        target_rotmat = Rotation.from_quat(self.target_hand_state[hand_type].palm_rot).as_matrix()
        # target_pos -= target_rotmat @ self.palm_wrist_offset

        # Get hand directions
        hand_dir = target_rotmat[:,0]
        thumb_dir = target_rotmat[:,1]
        palm_dir = -target_rotmat[:,2]

        # Compute IK: Step 1.
        shoulder_pos = self.shoulder_position[hand_type]
        shoulder_to_target = target_pos - shoulder_pos
        shoulder_target_dist = np.linalg.vector_norm(shoulder_to_target)
        shoulder_to_target_dir = shoulder_to_target / shoulder_target_dist
        if shoulder_target_dist > self.max_shoulder_target_dist:
            shoulder_target_dist = self.max_shoulder_target_dist
            shoulder_to_target = shoulder_to_target_dir * shoulder_target_dist
        # Compute IK: Step 2.
        tau = np.acos((self.upper_arm_length**2 - shoulder_target_dist**2 - self.lower_arm_length**2) / (-2 * shoulder_target_dist * self.lower_arm_length))
        # Compute IK: Step 3.
        arm_plane_up_dir = np.cross(shoulder_to_target_dir, hand_dir)
        arm_plane_up_dir /= np.linalg.vector_norm(arm_plane_up_dir)
        if (hand_type is HAND_TYPE_LEFT and arm_plane_up_dir[2] > 0) or (hand_type is HAND_TYPE_RIGHT and arm_plane_up_dir[2] < 0):
            arm_plane_up_dir[2] *= -1
        elbow_to_target_dir = Rotation.from_rotvec(arm_plane_up_dir * tau).as_matrix() @ shoulder_to_target_dir
        # Compute IK: Step 4.
        elbow_pos = target_pos - elbow_to_target_dir * self.lower_arm_length
        # Compute IK: Step 5.
        shoulder_to_elbow = elbow_pos - shoulder_pos
        # Compute IK: Step 6.
        shoulder_pitch = np.atan2(-shoulder_to_elbow[2], shoulder_to_elbow[0])
        # Compute IK: Step 7.
        shoulder_pitch_frame = Rotation.from_euler('y', -shoulder_pitch).as_matrix()
        shoulder_to_elbow_shoulder_pitch_frame = self.shoulder_frame_offset[hand_type] @ shoulder_pitch_frame @ shoulder_to_elbow
        shoulder_roll = np.asin(shoulder_to_elbow_shoulder_pitch_frame[1] / self.upper_arm_length)
        # Compute IK: Step 8.
        elbow_frame = self.elbow_frame_offset @ Rotation.from_euler('z', -shoulder_roll).as_matrix() @ shoulder_pitch_frame
        elbow_to_target_dir_elbow_frame = elbow_frame @ elbow_to_target_dir
        if hand_type is HAND_TYPE_RIGHT:
            elbow_yaw = np.atan2(elbow_to_target_dir_elbow_frame[2], elbow_to_target_dir_elbow_frame[1])
            # Compute IK: Step 9.
            elbow_roll = np.acos(elbow_to_target_dir_elbow_frame[0])
        else:
            elbow_yaw = np.atan2(-1 * elbow_to_target_dir_elbow_frame[2], -1 * elbow_to_target_dir_elbow_frame[1])
            # Compute IK: Step 9.
            elbow_roll = -1 * np.acos(elbow_to_target_dir_elbow_frame[0])
        # Compute IK: Step 10.
        phi = np.acos(np.dot(hand_dir, elbow_to_target_dir))
        palm_target_dir = Rotation.from_rotvec(arm_plane_up_dir * -phi).as_matrix() @ palm_dir
        wrist_frame = Rotation.from_euler('xz', [-elbow_yaw, -elbow_roll]).as_matrix() @ elbow_frame
        palm_target_dir_wrist_frame = wrist_frame @ palm_target_dir
        wrist_yaw = np.atan2(palm_target_dir_wrist_frame[1], -palm_target_dir_wrist_frame[2])

        # Sanity-check of computed joint angles
        joint_angles = [shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, wrist_yaw]
        if np.isnan(joint_angles).any():
            if self.show_status:
                ik_status_text = "IK computed NaN"
                ik_status_color = [1, 0, 0]
        else:
            # Limit joint angles
            clipped_joint_angles = np.clip(joint_angles, self.arm_joint_limits[hand_type][0], self.arm_joint_limits[hand_type][1])
            if self.show_status:
                if np.not_equal(clipped_joint_angles, joint_angles).any():
                    ik_status_text = "IK out of limits"
                    ik_status_color = [1, 0, 0]
                else:
                    ik_status_text = "IK computed OK"
                    ik_status_color = [0, 1, 0]

            # Apply joint angles to robot
            if self.use_real_robot:
                self.qi_motion_service.setAngles(self.arm_joint_names[hand_type], clipped_joint_angles.tolist(), self.max_joint_speeds_real_robot)
            if self.pepper:
                self.pepper.setAngles(self.arm_joint_names[hand_type], clipped_joint_angles.tolist(), self.max_joint_speeds_sim)

        # Debug information drawing
        show_ik = (self.show_left_ik and hand_type is HAND_TYPE_LEFT) or (self.show_right_ik and hand_type is HAND_TYPE_RIGHT)
        if show_ik or self.show_status:
            ids = self.sim_arm_debug_ids[hand_type]
            
            # Compute simulation world <-> ik base transform
            sim_base_ik_transfrom = self.computeIKBaseTransform(for_real_robot=False)

            # Convert positions and directions to be in world frame
            shoulder_pos = sim_base_ik_transfrom.transformToWorldPos(shoulder_pos)
            if show_ik:
                elbow_pos = sim_base_ik_transfrom.transformToWorldPos(elbow_pos)
                target_pos = sim_base_ik_transfrom.transformToWorldPos(target_pos)
                shoulder_to_elbow = sim_base_ik_transfrom.transformToWorldDir(shoulder_to_elbow)
                shoulder_to_target = sim_base_ik_transfrom.transformToWorldDir(shoulder_to_target)
                arm_plane_up_dir = sim_base_ik_transfrom.transformToWorldDir(arm_plane_up_dir)
                elbow_to_target_dir = sim_base_ik_transfrom.transformToWorldDir(elbow_to_target_dir)
                hand_dir = sim_base_ik_transfrom.transformToWorldDir(hand_dir)
                thumb_dir = sim_base_ik_transfrom.transformToWorldDir(thumb_dir)
                palm_dir = sim_base_ik_transfrom.transformToWorldDir(palm_dir)

                ids[0] = pybullet.addUserDebugLine(shoulder_pos, 
                                                   shoulder_pos + shoulder_to_elbow / np.linalg.vector_norm(shoulder_to_elbow) * self.upper_arm_length, 
                                                   [1,0,0], 10, replaceItemUniqueId=ids[0])
                ids[1] = pybullet.addUserDebugLine(shoulder_pos, shoulder_pos + shoulder_to_target, [0,0,1], 25, replaceItemUniqueId=ids[1])
                ids[2] = pybullet.addUserDebugLine(shoulder_pos, shoulder_pos + arm_plane_up_dir * 0.1, [0,0,0], 25, replaceItemUniqueId=ids[2])
                ids[3] = pybullet.addUserDebugLine(elbow_pos, elbow_pos + elbow_to_target_dir * self.lower_arm_length, [1,0,0], 25, 
                                                   replaceItemUniqueId=ids[3])
                ids[4] = pybullet.addUserDebugLine(target_pos, target_pos + hand_dir * 0.1, [0,1,0], 25, replaceItemUniqueId=ids[4])
                ids[5] = pybullet.addUserDebugLine(target_pos, target_pos + thumb_dir * 0.1, [0,1,0], 25, replaceItemUniqueId=ids[5])
                ids[6] = pybullet.addUserDebugLine(target_pos, target_pos + palm_dir * 0.1, [0,1,0], 25, replaceItemUniqueId=ids[6])
                shoulder_joint_name = "RShoulderPitch" if hand_type is HAND_TYPE_RIGHT else "LShoulderPitch"
                shoulder_joint_index = self.pepper.joint_dict[shoulder_joint_name].getIndex()
                robot_shoulder_pos = np.array(pybullet.getLinkState(bodyUniqueId=self.pepper.robot_model, linkIndex=shoulder_joint_index)[4])
                elbow_joint_name = "RElbowYaw" if hand_type is HAND_TYPE_RIGHT else "LElbowYaw"
                elbow_joint_index = self.pepper.joint_dict[elbow_joint_name].getIndex()
                robot_elbow_pos = np.array(pybullet.getLinkState(bodyUniqueId=self.pepper.robot_model, linkIndex=elbow_joint_index)[4])
                wrist_joint_name = "RWristYaw" if hand_type is HAND_TYPE_RIGHT else "LWristYaw"
                wrist_joint_index = self.pepper.joint_dict[wrist_joint_name].getIndex()
                robot_wrist_state = pybullet.getLinkState(bodyUniqueId=self.pepper.robot_model, linkIndex=wrist_joint_index)[4:6]
                robot_wrist_pos = np.array(robot_wrist_state[0])
                ids[7] = pybullet.addUserDebugPoints([robot_shoulder_pos, robot_elbow_pos, robot_wrist_pos, elbow_pos, target_pos], 
                                                     [[1,0,0],[1,0,0],[1,0,0],[0,1,0],[0,1,0]], 15, replaceItemUniqueId=ids[7])

                robot_wrist_rot = Rotation.from_quat(robot_wrist_state[1]).as_matrix()
                robot_hand_dir = robot_wrist_rot[:,0]
                robot_thumb_dir = robot_wrist_rot[:,1]
                robot_palm_dir = -robot_wrist_rot[:,2]
                ids[10] = pybullet.addUserDebugLine(robot_wrist_pos, robot_wrist_pos + robot_hand_dir * 0.1, [1,0,0], 25, replaceItemUniqueId=ids[10])
                ids[11] = pybullet.addUserDebugLine(robot_wrist_pos, robot_wrist_pos + robot_thumb_dir * 0.1, [1,0,0], 25, replaceItemUniqueId=ids[11])
                ids[12] = pybullet.addUserDebugLine(robot_wrist_pos, robot_wrist_pos + robot_palm_dir * 0.1, [1,0,0], 25, replaceItemUniqueId=ids[12])

                elbow_error = f"Error: {round(np.linalg.vector_norm(elbow_pos - robot_elbow_pos), 4) * 1000}mm"
                ids[8] = pybullet.addUserDebugText(elbow_error, robot_elbow_pos + [0, -0.05, 0.05], [1, 0, 0], 1, replaceItemUniqueId=ids[8])
                wrist_error = f"Error: {round(np.linalg.vector_norm(target_pos - robot_wrist_pos), 4) * 1000}mm"
                ids[9] = pybullet.addUserDebugText(wrist_error, robot_wrist_pos + [0, -0.05, 0.05], [1, 0, 0], 1, replaceItemUniqueId=ids[9])
                rot_error = (
                    f"Error: "
                    f"{round(np.acos(np.dot(hand_dir, robot_hand_dir)) / np.pi * 180, 2)}deg, "
                    f"{round(np.acos(np.dot(palm_dir, robot_palm_dir)) / np.pi * 180, 2)}deg, "
                    f"{round(np.acos(np.dot(thumb_dir, robot_thumb_dir)) / np.pi * 180, 2)}deg"
                )
                ids[13] = pybullet.addUserDebugText(rot_error, robot_wrist_pos + [0, 0.05, 0.05], [1, 0, 0], 1, replaceItemUniqueId=ids[13])
            if self.show_status:
                ids[14] = pybullet.addUserDebugText(ik_status_text, shoulder_pos + [0, -0.1, 0.3], ik_status_color, 1, replaceItemUniqueId=ids[14])
