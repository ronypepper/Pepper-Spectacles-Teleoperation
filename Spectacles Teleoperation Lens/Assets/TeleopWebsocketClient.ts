import { HandPoseTracker } from "./HandPoseTracker";
import { HandState, HAND_STATE_MSG_SIZE } from "./HandState"
import { IKBaseTransform } from "./IKBaseTransform"

@component
export class TeleopWebsocketClient extends BaseScriptComponent {
    // Websocket connection
    private internet_module : InternetModule = require("LensStudio:InternetModule");
    private socket : WebSocket;
    private websocket_ip : string = '10.0.0.124'
    private websocket_port : number = 51347
    private websocket_auth_token : string = 'jas73i46nuas6q02n3b5lsdjfoia';
    private use_localhost : boolean = true;
    private attempt_connection_interval : number = 2.0
    private attempt_connection_timer : number = 0.0
    private auth_token_sent : boolean = false;

    // Websocket rx msgs per seconds statistic
    private show_rx_msgs_statistic : boolean = true;
    private rx_msgs_stat_count : number = 0;
    private rx_msgs_stat_timer : number = 1;

    // Hand tracking
    @input camera : Camera;
    @input left_hand_tracker : HandPoseTracker;
    @input right_hand_tracker : HandPoseTracker;
    private has_acquired_ik_base_frame : boolean = false;
    private ik_base_frame_snapshot_timer_interval : number = 5.0;
    private ik_base_frame_snapshot_timer : number = this.ik_base_frame_snapshot_timer_interval;
    private ik_base_tf : IKBaseTransform;

    onAwake() {
        globalThis.textLogger.log("Acquiring IK base frame in " + this.ik_base_frame_snapshot_timer.toString() + " seconds.");
        globalThis.textLogger.log("Look straight ahead and stretch out arms.");
        this.createEvent('UpdateEvent').bind(() => {
            // Countdown a timer and then process current camera pose to determine the IK base frame
            if (!this.has_acquired_ik_base_frame) {
                if (this.ik_base_frame_snapshot_timer <= 0.0) {
                    if (this.left_hand_tracker.isNewTrackedHandStateAvailable() && this.right_hand_tracker.isNewTrackedHandStateAvailable()) {
                        let left_hand_state = this.left_hand_tracker.getHandState();
                        let right_hand_state = this.right_hand_tracker.getHandState();
                        this.ik_base_tf = new IKBaseTransform(this.camera, left_hand_state, right_hand_state);
                        globalThis.textLogger.log("IK base frame acquired. Starting teleoperation.");
                        this.has_acquired_ik_base_frame = true;
                    } else {
                        this.ik_base_frame_snapshot_timer = this.ik_base_frame_snapshot_timer_interval;
                        globalThis.textLogger.log("No hand tracking data available. Re-attempting to acquire IK base frame in " + this.ik_base_frame_snapshot_timer.toString() + " seconds.");
                    }
                }
                else {
                    this.ik_base_frame_snapshot_timer -= getDeltaTime();
                }
                return;
            }

            this.left_hand_tracker.ik_base_tf = this.ik_base_tf;
            this.right_hand_tracker.ik_base_tf = this.ik_base_tf;

            // Socket (re)connection handling and hand state transmission (once IK base frame has been determined above)
            if (!this.socket) {
                // If no connection active, periodically attempt to connect
                if (this.attempt_connection_timer <= 0.0) {
                    this.setupWebsocket()
                    this.attempt_connection_timer = this.attempt_connection_interval;
                }
                else {
                    this.attempt_connection_timer -= getDeltaTime();
                }
            }
            else if (this.auth_token_sent) {
                // Send new tracked hand pose
                this.sendTrackedHandPoses();

                // Rx messages per second statistic
                if (this.show_rx_msgs_statistic) {
                    this.rx_msgs_stat_timer -= getDeltaTime();
                    if (this.rx_msgs_stat_timer <= 0.0) {
                        this.rx_msgs_stat_timer = 1.0;
                        globalThis.textLogger.log("RX msgs last sec: " + this.rx_msgs_stat_count.toString());
                        this.rx_msgs_stat_count = 0
                    }
                }
            }
        });
    }

    private setupWebsocket() {
        // Establish websocket connection
        let url = this.use_localhost ? "localhost" : this.websocket_ip;
        url = "ws://" + url + ":" + this.websocket_port.toString();
        globalThis.textLogger.log("Trying to connect to " + url + " ...");
        this.socket = this.internet_module.createWebSocket(url);
        if (!this.socket) {
            globalThis.textLogger.log("Socket failed to create.");
        }
        else {
            globalThis.textLogger.log("Socket created.");
        }
        this.socket.binaryType = 'blob';

        this.socket.onopen = async (event: WebSocketEvent) => {
            // Socket has opened, send authentication token
            globalThis.textLogger.log("Connection established.");
            globalThis.textLogger.log("Authenticating...");
            this.socket.send(this.websocket_auth_token);
            this.auth_token_sent = true;
        };

        this.socket.onmessage = this.onWebsocketMessage.bind(this);

        this.socket.onclose = (event: WebSocketCloseEvent) => {
            if (event.wasClean) {
                globalThis.textLogger.log('Socket closed cleanly');
            } else {
                globalThis.textLogger.log('Socket closed with error');
                globalThis.textLogger.log('-- Code: ' + event.code);
                globalThis.textLogger.log('-- Reason: ' + event.reason);
            }
            this.auth_token_sent = false;
            this.socket = null;
        };

        this.socket.onerror = (event: WebSocketEvent) => {
            globalThis.textLogger.log('Socket error');
            this.socket = null;
        };
    }

    private async onWebsocketMessage(event: WebSocketMessageEvent) : Promise<void> {
        // Rx messages per second statistic
        if (this.show_rx_msgs_statistic) {
            this.rx_msgs_stat_count += 1;
        }

        // Process message
        if (event.data instanceof Blob) {
            let data = await event.data.bytes();
            
            // Decode left hand state and transform to world frame
            let left_hand_view = new DataView(data.buffer, data.byteOffset, data.byteLength);
            let ik_base_left_hs = new HandState();
            ik_base_left_hs.decode(left_hand_view);
            let world_left_hs = this.ik_base_tf.ikBaseToWorldFrame(ik_base_left_hs);
            this.left_hand_tracker.setRobotHandState(world_left_hs);

            // Decode right hand state and transform to world frame
            let right_hand_view = new DataView(data.buffer, data.byteOffset + HAND_STATE_MSG_SIZE, data.byteLength - HAND_STATE_MSG_SIZE);
            let ik_base_right_hs = new HandState();
            ik_base_right_hs.decode(right_hand_view);
            let world_right_hs = this.ik_base_tf.ikBaseToWorldFrame(ik_base_right_hs);
            this.right_hand_tracker.setRobotHandState(world_right_hs);
        }
        else {
            // Just log a text message
            globalThis.textLogger.log("Received text message: " + event.data);
        }
    };

    private sendTrackedHandPoses() {
        if (this.left_hand_tracker.isNewTrackedHandStateAvailable() && this.right_hand_tracker.isNewTrackedHandStateAvailable()) {
            // Transform left hand pose to ik base frame
            let world_left_hs = this.left_hand_tracker.getHandState();
            let ik_base_left_hs = this.ik_base_tf.worldToIKBaseFrame(world_left_hs);

            // Transform right hand pose to ik base frame
            let world_right_hs = this.right_hand_tracker.getHandState();
            let ik_base_right_hs = this.ik_base_tf.worldToIKBaseFrame(world_right_hs);

            // Encode hand poses for transmission
            let buffer = new ArrayBuffer(66);
            let left_hand_view = new DataView(buffer, 0, buffer.byteLength);
            ik_base_left_hs.encode(left_hand_view);
            let right_hand_view = new DataView(buffer, HAND_STATE_MSG_SIZE, buffer.byteLength - HAND_STATE_MSG_SIZE);
            ik_base_right_hs.encode(right_hand_view);
            let data = new Uint8Array(buffer);

            // Transmit hand poses
            if (this.socket.readyState == 1) { // Connection is open
                this.socket.send(data);
            }
            
            // // Show debug visual of transformed hand
            // let debug_hs = this.ik_base_tf.ikBaseToWorldFrame(ik_base_hs);
            // hand_tracker.setUserHandState(debug_hs);
        }
    }
}
