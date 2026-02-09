import struct
import numpy as np

HAND_STATE_MSG_SIZE = 33

HAND_TYPE_LEFT = 0
HAND_TYPE_RIGHT = 1

class HandState:
    def __init__(self, hand_type, palm_pos = np.array([0, 0, 0]), palm_rot = np.array([0, 0, 0, 1]), grip_strength = 0):
        self.hand_type = hand_type
        self.palm_pos = palm_pos
        self.palm_rot = palm_rot
        self.grip_strength = grip_strength

    def encodeHandState(hand_state):
        return struct.pack(
            '<Bffffffff',
            hand_state.hand_type,
            *(hand_state.palm_pos), # Pos [x,y,z] in meters
            *hand_state.palm_rot, # Rot [x,y,z,w]
            hand_state.grip_strength
        )

    def decodeHandState(byte_data):
        hand_type = HAND_TYPE_LEFT if byte_data[0] == 0 else HAND_TYPE_RIGHT
        float_data = struct.unpack_from('<ffffffff', byte_data, offset=1)
        return HandState(hand_type, 
                         np.array(float_data[0:3]), # Pos [x,y,z] in meters
                         np.array(float_data[3:7]), # Rot [x,y,z,w]
                         float_data[7])             # Grip strength
