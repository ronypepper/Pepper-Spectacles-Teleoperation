export const HAND_STATE_MSG_SIZE : number = 33;

export class HandState {
    handType : "left" | "right";
    palmPosition : vec3;
    palmRotation : quat;
    gripStrength : number;

    constructor(handType : string = "left", 
                palmPosition : vec3 = new vec3(0, 0, 0), 
                palmRotation : quat = new quat(1, 0, 0, 0),
                gripStrength : number = 0) {
        if (handType === "left" || handType === "right") {
            this.handType = handType;
        }
        else {
            this.handType = "left";
        }
        this.palmPosition = palmPosition;
        this.palmRotation = palmRotation;
        this.gripStrength = gripStrength;
    }

    public encode(view : DataView) {
        // Encode hand type
        if (this.handType === "left") {
            view.setUint8(0, 0);
        }
        else {
            view.setUint8(0, 1);
        }

        // Encode palm position [x,y,z] - converted from centimeters to meters
        view.setFloat32(1, this.palmPosition.x / 100, true);
        view.setFloat32(5, this.palmPosition.y / 100, true);
        view.setFloat32(9, this.palmPosition.z / 100, true);

        // Encode palm rotation [x,y,z,w]
        view.setFloat32(13, this.palmRotation.x, true);
        view.setFloat32(17, this.palmRotation.y, true);
        view.setFloat32(21, this.palmRotation.z, true);
        view.setFloat32(25, this.palmRotation.w, true);

        // Encode grip strength
        view.setFloat32(29, this.gripStrength, true);
    }

    public decode(view : DataView) {
        // Decode hand state
        if (view.getUint8(0) == 0) {
            this.handType = "left";
        }
        else {
            this.handType = "right";
        }

        // Decode palm position [x,y,z] - converted from meters to centimeters
        this.palmPosition.x = view.getFloat32(1, true) * 100;
        this.palmPosition.y = view.getFloat32(5, true) * 100;
        this.palmPosition.z = view.getFloat32(9, true) * 100;

        // Decode palm rotation [x,y,z,w]
        this.palmRotation.x = view.getFloat32(13, true);
        this.palmRotation.y = view.getFloat32(17, true);
        this.palmRotation.z = view.getFloat32(21, true);
        this.palmRotation.w = view.getFloat32(25, true);

        // Decode grip strength
        this.gripStrength = view.getFloat32(29, true);
    }
}
