import pygame
import numpy as np
from HandState import HAND_TYPE_LEFT, HAND_TYPE_RIGHT
import pybullet
import sys

GAMEPAD_CTRL_POS_SPEED = 0.1
GAMEPAD_CTRL_ROT_SPEED = 0.5
GAMEPAD_CTRL_GRIP_SPEED = 1

KEYBOARD_CTRL_POS_SPEED = 0.1
KEYBOARD_CTRL_ROT_SPEED = 1.2
KEYBOARD_CTRL_GRIP_SPEED = 1.5

class LocalController:
    def __init__(self):
        # Gamepad initalization
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.gamepad = None
            print("No gamepad detected - falling back to keyboard")

            self.ctrl_keys = {
                ord('i'): np.array([0, 1, 0]),
                ord('k'): np.array([0, -1, 0]),
                ord('j'): np.array([-1, 0, 0]),
                ord('l'): np.array([1, 0, 0]),
                ord('u'): np.array([0, 0, 1]),
                ord('o'): np.array([0, 0, -1])
            }
            self.key_toggle_hand = ord('h')
            self.key_toggle_mode = ord('m')
            self.key_open_gripper = ord('t')
            self.key_close_gripper = ord('g')
        else:
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()
            print("Detected:", self.gamepad.get_name())
        
        self.cur_ctrl_hand = HAND_TYPE_LEFT
        self.is_controlling_pos = True

    def _apply_deadzone(self, value, deadzone=0.1):
        return 0.0 if abs(value) < deadzone else value

    def process_inputs(self, delta_time):
        delta_pos, delta_euler_rot, delta_grip = np.array([0, 0, 0]), np.array([0, 0, 0]), 0

        if self.gamepad is not None:
            pygame.event.pump()

            # Toggle currently controlled hand and mode
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == pygame.CONTROLLER_BUTTON_A: # ps4-x-button
                        self.cur_ctrl_hand = HAND_TYPE_LEFT if self.cur_ctrl_hand == HAND_TYPE_RIGHT else HAND_TYPE_RIGHT
                    elif event.button == pygame.CONTROLLER_BUTTON_B: # ps4-o-button
                        self.is_controlling_pos = not self.is_controlling_pos
            
            # Open/close gripper
            if self.gamepad.get_button(pygame.CONTROLLER_BUTTON_LEFTSHOULDER):
                delta_grip += delta_time * GAMEPAD_CTRL_GRIP_SPEED
            if self.gamepad.get_button(pygame.CONTROLLER_BUTTON_RIGHTSHOULDER):
                delta_grip -= delta_time * GAMEPAD_CTRL_GRIP_SPEED
            
            # Compute delta hand position/orientation
            lx = self._apply_deadzone(self.gamepad.get_axis(0))
            ly = self._apply_deadzone(self.gamepad.get_axis(1))
            ry = self._apply_deadzone(self.gamepad.get_axis(3))
            delta = np.array([ly, lx, -ry])

            # Normalize delta hand position/orientation
            length = np.linalg.vector_norm(delta)
            if length != 0:
                delta = delta / length
            if self.is_controlling_pos:
                delta_pos = delta * delta_time * GAMEPAD_CTRL_POS_SPEED
            else:
                delta_euler_rot = delta * delta_time * GAMEPAD_CTRL_ROT_SPEED
        else:
            if not pybullet.isConnected():
                sys.exit("ERROR: LocalController in keyboard mode requires a running simulation! Use a gamepad instead.")
            key_events = pybullet.getKeyboardEvents()

            # Toggle currently controlled hand and mode
            if self.key_toggle_hand in key_events and key_events[self.key_toggle_hand] & pybullet.KEY_WAS_TRIGGERED:
                self.cur_ctrl_hand = HAND_TYPE_LEFT if self.cur_ctrl_hand == HAND_TYPE_RIGHT else HAND_TYPE_RIGHT
            if self.key_toggle_mode in key_events and key_events[self.key_toggle_mode] & pybullet.KEY_WAS_TRIGGERED:
                self.is_controlling_pos = not self.is_controlling_pos

            # Open/close gripper
            if self.key_open_gripper in key_events and key_events[self.key_open_gripper] & pybullet.KEY_IS_DOWN:
                delta_grip += delta_time * KEYBOARD_CTRL_GRIP_SPEED
            if self.key_close_gripper in key_events and key_events[self.key_close_gripper] & pybullet.KEY_IS_DOWN:
                delta_grip -= delta_time * KEYBOARD_CTRL_GRIP_SPEED
            
            # Compute delta hand position/orientation
            delta = np.array([0, 0, 0])
            for key, state in key_events.items():
                if state & pybullet.KEY_IS_DOWN:
                    if key in self.ctrl_keys:
                        delta += self.ctrl_keys[key]
            
            # Normalize delta hand position/orientation
            length = np.linalg.vector_norm(delta)
            if length != 0:
                delta = delta / length
            if self.is_controlling_pos:
                delta_pos = delta * delta_time * KEYBOARD_CTRL_POS_SPEED
            else:
                delta_euler_rot = delta * delta_time * KEYBOARD_CTRL_ROT_SPEED
        
        return delta_pos, delta_euler_rot, delta_grip, self.cur_ctrl_hand
