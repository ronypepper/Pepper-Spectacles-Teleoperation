from multiprocessing import Process, shared_memory, Lock, Event
from MPServer import start_mp_server, send_text_msg
from MPServer import MAX_TX_MSG_SIZE, MAX_RX_MSG_SIZE, MSG_HEADER_SIZE
from qibullet import SimulationManager, PepperVirtual
import pybullet
import numpy as np
import struct, signal, time
from datetime import datetime
import os
import qi
from OneEuroFilter import OneEuroFilter

from HandState import HandState, HAND_STATE_MSG_SIZE, HAND_TYPE_LEFT, HAND_TYPE_RIGHT
from LocalController import LocalController
from PepperArmIK import PepperArmIK

# Configuration
SERVER_IP = "10.0.0.124"
SERVER_PORT = 51347
SERVER_USE_LOCALHOST = True
SERVER_AUTHENTICATION_TOKEN = "jas73i46nuas6q02n3b5lsdjfoia"
POSE_QUERY_INTERVAL = 0#1./200.
POSE_SEND_INTERVAL = 0#1./200.
ROBOT_MAX_JOINT_SPEED = 0.7
ROBOT_MAX_STIFFNESS = 0.85
REMOTE_OPERATOR_SCALE = 60.0 / 40.0
TRACK_RELATIVE_MOVEMENTS = False
APPLY_ONE_EURO_FILTER = False
USE_REMOTE_CONTROL = True
SEND_ROBOT_POSE_TO_REMOTE = True
USE_REAL_PEPPER_ROBOT = True
RUN_SYNCED_SIM_WITH_REAL_ROBOT = False

PRINT_RX_MSGS_RATE = True
PRINT_CTRL_FREQUENCY = True

LOG_POSES_TO_FILE = True

# Constants
SIM_TIME_STEP = 1./240.

class PepperTeleoperator():
    def __init__(self):
        # Rx message rate statistic
        self.rx_msgs_rate_count = 0
        self.rx_msgs_rate_timestamp = 0

        # Control loop frequency statistic
        self.ctrl_freq_count = 0
        self.ctrl_freq_timestamp = 0

        # Setup hand states log
        if LOG_POSES_TO_FILE:
            self.log_max_num_samples = 1000 * 120 # 2 Minutes @ 1000 Hz
            self.log_hand_states = np.zeros((self.log_max_num_samples, 1 + 4 * 8)) # Timestamp + 4 hand states
            self.log_num_samples = 0

        # Real robot initialization
        self.qi_motion_service = None
        self.qi_posture_service = None
        if USE_REAL_PEPPER_ROBOT:
            self.qi_app = qi.Application()
            self.qi_app.start()
            self.qi_session = self.qi_app.session
            self.qi_motion_service = self.qi_session.service("ALMotion")
            self.qi_posture_service = self.qi_session.service("ALRobotPosture")

            # Gracefully increase to max stiffness on all possible motors
            joint_names  = ["Head", "LArm", "RArm", "LHand", "RHand"]
            for name in joint_names:
                self.qi_motion_service.stiffnessInterpolation(name, ROBOT_MAX_STIFFNESS, 1.0) # Blocking call

        # Remote / Control initalization
        if USE_REMOTE_CONTROL or SEND_ROBOT_POSE_TO_REMOTE:
            print("Using remote robot control.")
            self.initial_remote_pose_acquired = False

            # Start async server
            self.shm_msg_tx = shared_memory.SharedMemory(create=True, size=MSG_HEADER_SIZE + MAX_TX_MSG_SIZE)
            self.shm_msg_rx = shared_memory.SharedMemory(create=True, size=MSG_HEADER_SIZE + MAX_RX_MSG_SIZE)
            self.lock_msg_tx = Lock()
            self.lock_msg_rx = Lock()
            self.stop_event = Event()
            server_p_kwargs = {
                "stop_event" : self.stop_event,
                "shm_msg_tx_name" : self.shm_msg_tx.name,
                "shm_msg_rx_name" : self.shm_msg_rx.name,
                "lock_msg_tx" : self.lock_msg_tx,
                "lock_msg_rx" : self.lock_msg_rx,
                "server_ip" : SERVER_IP, 
                "server_port" : SERVER_PORT, 
                "use_localhost" : SERVER_USE_LOCALHOST, 
                "auth_token" : SERVER_AUTHENTICATION_TOKEN
            }
            self.server_p = Process(target=start_mp_server, kwargs=server_p_kwargs)
            self.server_p.start()
        if not USE_REMOTE_CONTROL:
            print("Using local keyboard/gamepad control.")
            self.local_controller = LocalController()
        
        # Initialize one-euro filter
        if APPLY_ONE_EURO_FILTER:
            config = {
                'freq': 120,       # Hz
                'mincutoff': 1.0,  # Hz
                'beta': 0.1,       
                'dcutoff': 1.0    
                }
            self.one_euro_filter = OneEuroFilter(**config)
    
    def start(self):
        try:
            if not USE_REAL_PEPPER_ROBOT or RUN_SYNCED_SIM_WITH_REAL_ROBOT:
                # Start the simulation
                self.simulation_manager = SimulationManager()
                self.sim_client_id = self.simulation_manager.launchSimulation(gui=True, auto_step=False)
                print("Simulation started.")
                
                # Spawn pepper robot
                self.pepper = self.simulation_manager.spawnPepper(
                    self.sim_client_id,
                    translation=[0, 0, 0],
                    quaternion=[0, 0, 0, 1],
                    spawn_ground_plane=True
                )
            else:
                self.simulation_manager = None
                self.pepper = None

            # Move real/sim robot(s) to upright posture with outstretched arms
            print("Moving robot to initial upright posture...")
            if USE_REAL_PEPPER_ROBOT:
                self.qi_posture_service.goToPosture("StandZero", 0.5) # Blocking call
                time.sleep(0.5)
            if self.simulation_manager:
                self.pepper.goToPosture("StandZero", 0.5)
                init_pose_end_time = time.time() + 3
                while time.time() < init_pose_end_time:
                    cur_frame_start_time = time.time()
                    self.simulation_manager.stepSimulation(self.sim_client_id)

                    # Sleep surplus unspent simulation delta time to keep simulation real-time
                    delta_time = time.time() - cur_frame_start_time
                    time.sleep(max(SIM_TIME_STEP - delta_time, 0.))
            
            # Initialize robot arm inverse kinematics
            print("Starting inverse kinematics.")
            self.pepper_arm_ik = PepperArmIK(REMOTE_OPERATOR_SCALE, ROBOT_MAX_JOINT_SPEED, self.pepper, USE_REAL_PEPPER_ROBOT, 
                                             self.qi_motion_service, show_markers=True, show_left_ik=False, 
                                             show_right_ik=False, show_status=False)

            # # Test target position
            # sim_base_ik_transfrom = pepper_arm_ik.computeIKBaseTransform(for_real_robot=False)
            # a, b = sim_base_ik_transfrom.transformToIKBase(np.array([0.1807878315448761, 0.09563484787940979, 0.7492444515228271]),
            #                                         np.array([0.4378834366798401, 0.03729911893606186, -0.5032546520233154, 0.7440441250801086]))
            # pepper_arm_ik.setTargetHandState(HandState(HAND_TYPE_LEFT, a, b, 0))
            # c, d = sim_base_ik_transfrom.transformToIKBase(np.array([0.2415480613708496, -0.15686166286468506, 0.9513795971870422]),
            #                                         np.array([0.5360865592956543, 0.15571333467960358, 0.2026689499616623, 0.8045432567596436]))
            # pepper_arm_ik.setTargetHandState(HandState(HAND_TYPE_RIGHT, c, d, 0))

            # # Test move
            # pybullet.setRealTimeSimulation(1)
            # pepper.moveTo(0.3, 0.2, 0.6, PepperVirtual.FRAME_ROBOT, 1.5)
            # time.sleep(5)
            # pybullet.setRealTimeSimulation(0)

            # Initialize pose query/send timers
            self.next_pose_query_time = time.time() + POSE_QUERY_INTERVAL
            self.next_pose_send_time = time.time() + POSE_SEND_INTERVAL

            # Simulation / control loop
            teleop_start_time = time.time()
            delta_time = 0
            while True:
                # Log frame start time
                cur_frame_start_time = time.time()

                # Update robot hand states
                self.pepper_arm_ik.updateRobotState()

                # Periodically send robot hand poses to remote operator
                if SEND_ROBOT_POSE_TO_REMOTE and (not TRACK_RELATIVE_MOVEMENTS or self.initial_remote_pose_acquired):
                    cur_time = time.time()
                    if self.next_pose_send_time <= cur_time:
                        if TRACK_RELATIVE_MOVEMENTS:
                            left_hand_state = self.pepper_arm_ik.getRobotHandState(HAND_TYPE_LEFT, False)
                            right_hand_state = self.pepper_arm_ik.getRobotHandState(HAND_TYPE_RIGHT, False)
                            delta = (left_hand_state.palm_pos - self.initial_robot_left_hand_pos) * REMOTE_OPERATOR_SCALE
                            left_hand_state.palm_pos = self.initial_remote_left_hand_pos + delta
                            delta = (right_hand_state.palm_pos - self.initial_robot_right_hand_pos) * REMOTE_OPERATOR_SCALE
                            right_hand_state.palm_pos = self.initial_remote_right_hand_pos + delta
                        else:
                            left_hand_state = self.pepper_arm_ik.getRobotHandState(HAND_TYPE_LEFT, True)
                            right_hand_state = self.pepper_arm_ik.getRobotHandState(HAND_TYPE_RIGHT, True)
                        if self.sendRobotStateToRemote(left_hand_state, right_hand_state):
                            self.next_pose_send_time = cur_time + POSE_SEND_INTERVAL

                # Query for target hand state updates
                if USE_REMOTE_CONTROL:
                    # Periodically query for operator hand pose updates from server
                    cur_time = time.time()
                    if self.next_pose_query_time <= cur_time:
                        success, left_hand_state, right_hand_state = self.queryRemoteOperatorState()
                        if success:
                            self.next_pose_query_time = cur_time + POSE_QUERY_INTERVAL

                            # Apply one-euro filter
                            if APPLY_ONE_EURO_FILTER:
                                timestamp = time.time() - teleop_start_time
                                lh_x = self.one_euro_filter(float(left_hand_state.palm_pos[0]), timestamp)
                                lh_y = self.one_euro_filter(float(left_hand_state.palm_pos[1]), timestamp)
                                lh_z = self.one_euro_filter(float(left_hand_state.palm_pos[2]), timestamp)
                                left_hand_state.palm_pos = np.array([lh_x, lh_y, lh_z])
                                rh_x = self.one_euro_filter(float(right_hand_state.palm_pos[0]), timestamp)
                                rh_y = self.one_euro_filter(float(right_hand_state.palm_pos[1]), timestamp)
                                rh_z = self.one_euro_filter(float(right_hand_state.palm_pos[2]), timestamp)
                                right_hand_state.palm_pos = np.array([rh_x, rh_y, rh_z])
                            
                            if TRACK_RELATIVE_MOVEMENTS:
                                if not self.initial_remote_pose_acquired:
                                    # Acquire initial hand positions - detected hand movements will be relative to them
                                    self.initial_remote_left_hand_pos = left_hand_state.palm_pos
                                    self.initial_remote_right_hand_pos = right_hand_state.palm_pos
                                    self.initial_robot_left_hand_pos = self.pepper_arm_ik.getRobotHandState(HAND_TYPE_LEFT, False).palm_pos
                                    self.initial_robot_right_hand_pos = self.pepper_arm_ik.getRobotHandState(HAND_TYPE_RIGHT, False).palm_pos
                                    self.initial_remote_pose_acquired = True

                                    # Compute target hand pose
                                    delta = (left_hand_state.palm_pos - self.initial_remote_left_hand_pos) / REMOTE_OPERATOR_SCALE
                                    left_hand_state.palm_pos = self.initial_robot_left_hand_pos + delta
                                    delta = (right_hand_state.palm_pos - self.initial_remote_right_hand_pos) / REMOTE_OPERATOR_SCALE
                                    right_hand_state.palm_pos = self.initial_robot_right_hand_pos + delta

                                    # Update target hand pose
                                    self.pepper_arm_ik.setTargetHandState(left_hand_state, False)
                                    self.pepper_arm_ik.setTargetHandState(right_hand_state, False)
                            else:
                                # Update target hand pose
                                self.pepper_arm_ik.setTargetHandState(left_hand_state, True)
                                self.pepper_arm_ik.setTargetHandState(right_hand_state, True)

                            # Rx message rate statistic
                            if PRINT_RX_MSGS_RATE and USE_REMOTE_CONTROL:
                                self.rx_msgs_rate_count += 1
                else:
                    # Control target hand position with gamepad/keyboard input
                    delta_pos, delta_euler_rot, delta_grip, hand_type = self.local_controller.process_inputs(delta_time)

                    # Update target hand pose
                    self.pepper_arm_ik.changeTargetHandState(delta_pos, delta_euler_rot, delta_grip, hand_type)
                
                # Update hand markers (if enabled)
                self.pepper_arm_ik.updateSimulationHandMarkers()
                
                # Compute inverse kinematics and apply to robot
                self.pepper_arm_ik.computeAndApplyIK(HAND_TYPE_LEFT)
                self.pepper_arm_ik.computeAndApplyIK(HAND_TYPE_RIGHT)

                # Log poses
                if LOG_POSES_TO_FILE and self.log_num_samples < self.log_max_num_samples:
                    log_timestamp = time.time() - teleop_start_time
                    robot_left_hs = self.pepper_arm_ik.getRobotHandState(HAND_TYPE_LEFT, False)
                    robot_right_hs = self.pepper_arm_ik.getRobotHandState(HAND_TYPE_RIGHT, False)
                    target_left_hs = self.pepper_arm_ik.getTargetHandState(HAND_TYPE_LEFT, False)
                    target_right_hs = self.pepper_arm_ik.getTargetHandState(HAND_TYPE_RIGHT, False)
                    log_entry = np.concatenate([[log_timestamp], 
                                                robot_left_hs.palm_pos, robot_left_hs.palm_rot, [robot_left_hs.grip_strength],
                                                robot_right_hs.palm_pos, robot_right_hs.palm_rot, [robot_right_hs.grip_strength],
                                                target_left_hs.palm_pos, target_left_hs.palm_rot, [target_left_hs.grip_strength],
                                                target_right_hs.palm_pos, target_right_hs.palm_rot, [target_right_hs.grip_strength]])
                    self.log_hand_states[self.log_num_samples, :] = log_entry
                    self.log_num_samples += 1

                # Step simulation
                if self.simulation_manager:
                    self.simulation_manager.stepSimulation(self.sim_client_id)

                # Rx message rate statistic
                if PRINT_RX_MSGS_RATE and USE_REMOTE_CONTROL:
                    if self.rx_msgs_rate_timestamp + 1.0 <= time.time():
                        self.rx_msgs_rate_timestamp = time.time()
                        rx_msgs_stat_text = "RX msgs last sec: " + str(self.rx_msgs_rate_count)
                        self.rx_msgs_rate_count = 0
                        print(rx_msgs_stat_text)
                
                # Control loop frequency statistic
                if PRINT_CTRL_FREQUENCY:
                    self.ctrl_freq_count += 1
                    if self.ctrl_freq_timestamp + 1.0 <= time.time():
                        self.ctrl_freq_timestamp = time.time()
                        ctrl_freq_text = "Control frequency: " + str(self.ctrl_freq_count) + "Hz"
                        self.ctrl_freq_count = 0
                        print(ctrl_freq_text)

                # Compute frame delta time
                delta_time = time.time() - cur_frame_start_time

                # Sleep surplus unspent simulation delta time to keep simulation real-time (only applies when sim is running)
                if self.simulation_manager:
                    time.sleep(max(SIM_TIME_STEP - delta_time, 0.))

        except pybullet.error as e:
            print(f"Physics simulation error: {e}")
            print("Shutting down...")
        except KeyboardInterrupt:
            print("Interrupted — shutting down...")
        finally:
            # Ignore KeyboardInterrupt now to prevent shared memory leak in case someone gets impatient and spams Ctrl+C
            signal.signal(signal.SIGINT, signal.SIG_IGN)

            # Move real robot to safe crouch posture
            if USE_REAL_PEPPER_ROBOT:
                print("Moving real robot to safe crouch posture...")
                self.qi_posture_service.goToPosture("Crouch", 0.5)

                # Gracefully decrease to min stiffness on all possible motors
                joint_names  = ["Head", "LArm", "RArm", "LHand", "RHand"]
                for name in joint_names:
                    self.qi_motion_service.stiffnessInterpolation(name, 0.0, 1.0) # Blocking call

            # Save loged poses to file
            if LOG_POSES_TO_FILE and self.log_num_samples > 0:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                os.makedirs("teleop_logs", exist_ok=True)
                log_file_name = f"teleop_logs/teleop_log_{timestamp}.npz"
                np.savez(log_file_name, data=self.log_hand_states[0:self.log_num_samples])
                print(f"{self.log_num_samples} hand state samples logged to: {log_file_name}")

            # Stop the simulation (if running)
            if self.simulation_manager:
                self.simulation_manager.stopSimulation(self.sim_client_id)

            if USE_REMOTE_CONTROL or SEND_ROBOT_POSE_TO_REMOTE:
                # Stop the server
                self.stop_event.set()
                self.server_p.join(timeout=3)

                # Terminate server if still alive
                if self.server_p.is_alive():
                    print("Child didn't shut down cleanly, terminating...")
                    self.server_p.terminate()
                    self.server_p.join()

                # Clean up shared memory
                self.shm_msg_tx.close()
                self.shm_msg_tx.unlink()
                self.shm_msg_rx.close()
                self.shm_msg_rx.unlink()

            print("Cleanup done. Bye.")
        
    def queryRemoteOperatorState(self):
        with self.lock_msg_rx:
            if self.shm_msg_rx.buf[0] == 1:
                msg_len = struct.unpack('H', self.shm_msg_rx.buf[1:MSG_HEADER_SIZE])[0]
                if msg_len == HAND_STATE_MSG_SIZE * 2:
                    data_left_hand = bytes(self.shm_msg_rx.buf[MSG_HEADER_SIZE:MSG_HEADER_SIZE + HAND_STATE_MSG_SIZE])
                    data_right_hand = bytes(self.shm_msg_rx.buf[MSG_HEADER_SIZE + HAND_STATE_MSG_SIZE:MSG_HEADER_SIZE + HAND_STATE_MSG_SIZE * 2])
                    return (True, HandState.decodeHandState(data_left_hand), HandState.decodeHandState(data_right_hand))
        return (False, None, None)

    def sendRobotStateToRemote(self, left_hand_state, right_hand_state):
        with self.lock_msg_tx:
            if self.shm_msg_tx.buf[0] == 0:
                self.shm_msg_tx.buf[0] = 1
                self.shm_msg_tx.buf[1:MSG_HEADER_SIZE] = struct.pack('H', HAND_STATE_MSG_SIZE * 2)
                self.shm_msg_tx.buf[MSG_HEADER_SIZE:MSG_HEADER_SIZE + HAND_STATE_MSG_SIZE] = HandState.encodeHandState(left_hand_state)
                self.shm_msg_tx.buf[MSG_HEADER_SIZE + HAND_STATE_MSG_SIZE:MSG_HEADER_SIZE + HAND_STATE_MSG_SIZE * 2] = HandState.encodeHandState(right_hand_state)
                return True
            return False

if __name__ == "__main__":
    pepper_teleoperator = PepperTeleoperator()
    pepper_teleoperator.start()
