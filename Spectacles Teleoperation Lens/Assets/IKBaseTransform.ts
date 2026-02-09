import {HandState} from "./HandState"

export class IKBaseTransform {
    private ik_base_rot : quat;
    private ik_base_inv_rot : quat;
    private ik_base_pos : vec3;

    constructor(camera : Camera, left_hand_state : HandState, right_hand_state : HandState) {
        let tf = camera.getTransform();
        this.ik_base_rot = tf.getWorldRotation();
        this.ik_base_inv_rot = this.ik_base_rot.invert();
        this.ik_base_pos = tf.getWorldPosition();
        this.ik_base_pos.y = (left_hand_state.palmPosition.y + right_hand_state.palmPosition.y) / 2.0;
    }

    public worldToIKBaseFrame(hand_state : HandState) : HandState {
        let world_hs = new HandState();
        world_hs.handType = hand_state.handType;
        world_hs.gripStrength = hand_state.gripStrength;

        // Transform to camera's local frame (x: left->right, y: bottom->top, z: front->back)
        let delta_pos = hand_state.palmPosition.sub(this.ik_base_pos);
        world_hs.palmPosition = this.ik_base_inv_rot.multiplyVec3(delta_pos);
        world_hs.palmRotation = this.ik_base_inv_rot.multiply(hand_state.palmRotation);
        print(world_hs.palmRotation.getAngle)

        // Transform to camera's local frame (x: left->right, y: bottom->top, z: front->back)
        world_hs.palmPosition = new vec3(
            -world_hs.palmPosition.z, // X' = -Z
            -world_hs.palmPosition.x, // Y' =  -X
            world_hs.palmPosition.y  // Z' =  Y
        );

        let basisRotationMat = new mat3();
        basisRotationMat.column0 = new vec3(0, -1, 0);
        basisRotationMat.column1 = new vec3(0, 0, 1);
        basisRotationMat.column2 = new vec3(-1, 0, 0);
        let B = quat.fromRotationMat(basisRotationMat);
        let Binv = B.invert();
        world_hs.palmRotation = B.multiply(world_hs.palmRotation).multiply(Binv);

        let AC = quat.fromEulerVec(new vec3(0, -Math.PI / 2, 0));
        AC = quat.fromEulerVec(new vec3(0, 0, Math.PI)).multiply(AC);
        world_hs.palmRotation = world_hs.palmRotation.multiply(AC);
        // let rrr = hand_state.palmRotation.toEulerAngles();
        // rrr = new vec3(-rrr.z, -rrr.x, rrr.y);
        // hand_state.palmRotation = quat.fromEulerAngles(rrr.x, rrr.y, rrr.z);

        return world_hs;
    }

    public ikBaseToWorldFrame(hand_state : HandState) : HandState {
        let ikbase_hs = new HandState();
        ikbase_hs.handType = hand_state.handType;
        ikbase_hs.gripStrength = hand_state.gripStrength;

        let adj_pos = new vec3(
            -hand_state.palmPosition.y,
            hand_state.palmPosition.z,
            -hand_state.palmPosition.x
        );
        let basisRotationMat = new mat3();
        basisRotationMat.column0 = new vec3(0, 0, -1);
        basisRotationMat.column1 = new vec3(-1, 0, 0);
        basisRotationMat.column2 = new vec3(0, 1, 0);
        let B = quat.fromRotationMat(basisRotationMat);
        let Binv = B.invert();
        let adj_rot = B.multiply(hand_state.palmRotation).multiply(Binv);


        let rot_pos = this.ik_base_rot.multiplyVec3(adj_pos);
        ikbase_hs.palmPosition = rot_pos.add(this.ik_base_pos);
        ikbase_hs.palmRotation = this.ik_base_rot.multiply(adj_rot);
        return ikbase_hs;
    }
}
