import SIK from 'SpectaclesInteractionKit.lspkg/SIK';
import TrackedHand from 'SpectaclesInteractionKit.lspkg/Providers/HandInputData/TrackedHand';
import { Keypoint } from 'SpectaclesInteractionKit.lspkg/Providers/HandInputData/Keypoint';
import {HandState} from "./HandState"
import { IKBaseTransform } from 'IKBaseTransform';

const numberOfKeypointVisuals : number = 21;

@component
export class HandPoseTracker extends BaseScriptComponent {
    // Configuration
    @input handType : string = "";
    @input fingerKeypointPrefab : ObjectPrefab;
    @input robotHandPrefab : ObjectPrefab;
    @input userHandPrefab : ObjectPrefab;
    @input enableUserHandVisuals : boolean;
    @input enableKeypointsVisuals : boolean;

    // Hand tracking
    private hand : TrackedHand;
    private hastracking : boolean = false;
    private handState : HandState = new HandState(this.handType);
    private newHandStateAvailable : boolean = false;

    // Visuals
    private robotHand : SceneObject;
    private userHand : SceneObject;
    private keypointVisuals : SceneObject[] = Array(numberOfKeypointVisuals);

    ik_base_tf : IKBaseTransform;

    onAwake() {
        // Get tracked hand
        if (this.handType == "left") {
            this.hand = SIK.HandInputData.getHand("left");
        }
        else if (this.handType == "right") {
            this.hand = SIK.HandInputData.getHand("right");
        }
        else {
            globalThis.textLogger.log("Invalid hand selection. Must be \"left\" or \"right\"!");
            return;
        }

        // Create hand visuals
        this.robotHand = this.robotHandPrefab.instantiate(this.getSceneObject());
        this.robotHand.enabled = false;
        if (this.enableUserHandVisuals) {
            this.userHand = this.userHandPrefab.instantiate(this.getSceneObject());
            this.userHand.enabled = false;
        }

        // Create tracked keypoint visuals
        if (this.enableKeypointsVisuals) {
            this.createTrackedKeypointVisuals();
        }
        
        this.createEvent('UpdateEvent').bind(() => {
            if (this.hand.isTracked()) {
                // Check if tracking was newly acquired
                if (!this.hastracking) {
                    this.hastracking = true;
                    //globalThis.textLogger.log("Tracking acquired.");

                    // Show tracked keypoint visuals
                    if (this.enableKeypointsVisuals) {
                        for (let i = 0; i < numberOfKeypointVisuals; i++) {
                            this.keypointVisuals[i].enabled = true;
                        }
                    }
                }

                // Update current hand state
                this.handState.palmPosition = this.hand.getPalmCenter();
                this.handState.palmRotation = this.hand.wrist.rotation;
                this.handState.gripStrength = Math.max(Math.min(1.0 - this.hand.getPinchStrength(), 1.0), 0.0);
                this.newHandStateAvailable = true;

                // Update user hand visual
                if (this.enableUserHandVisuals && this.ik_base_tf) {
                    let userHandTransform = this.userHand.getTransform();
                    let hs = this.ik_base_tf.ikBaseToWorldFrame(this.ik_base_tf.worldToIKBaseFrame(this.handState));
                    userHandTransform.setWorldPosition(hs.palmPosition);
                    userHandTransform.setWorldRotation(hs.palmRotation);
                    this.userHand.enabled = true;
                }

                // Update position of tracked hand visuals
                if (this.enableKeypointsVisuals) {
                    for (let i = 0; i < numberOfKeypointVisuals - 1; i++) {
                        const transform = this.keypointVisuals[i].getTransform();
                        let keypoint = this.hand.points[i];
                        transform.setWorldPosition(keypoint.position);
                    }
                    const transform = this.keypointVisuals[numberOfKeypointVisuals - 1].getTransform();
                    transform.setWorldPosition(this.hand.getPalmCenter());
                }
            }
            else {
                // Check if tracking was newly lost.
                if (this.hastracking) {
                    this.hastracking = false;

                    // Hider user hand visuals
                    this.userHand.enabled = false;

                    // Hide tracked hand visuals
                    if (this.enableKeypointsVisuals) {
                        for (let i = 0; i < numberOfKeypointVisuals; i++) {
                            this.keypointVisuals[i].enabled = false;
                        }
                    }
                }
            }
        });
    }

    private createTrackedKeypointVisuals() {
        let mySceneObject = this.getSceneObject();
        for (let i = 0; i < numberOfKeypointVisuals; i++) {
            this.keypointVisuals[i] = this.fingerKeypointPrefab.instantiate(mySceneObject);
            this.keypointVisuals[i].enabled = false;
        }
    }

    private destroyTrackedHandVisuals() {
        for (let i = 0; i < numberOfKeypointVisuals; i++) {
            this.keypointVisuals[i].destroy();
            this.keypointVisuals[i] = null;
        }
    }

    public isNewTrackedHandStateAvailable() : boolean {
        return this.newHandStateAvailable;
    }

    public getHandState() : HandState {
        this.newHandStateAvailable = false;
        return this.handState;
    }

    public setRobotHandState(handState : HandState) {
        // Update robot hand visual
        this.robotHand.enabled = true;
        let robotHandTransform = this.robotHand.getTransform();
        robotHandTransform.setWorldPosition(handState.palmPosition);
        robotHandTransform.setWorldRotation(handState.palmRotation);
    }
}
