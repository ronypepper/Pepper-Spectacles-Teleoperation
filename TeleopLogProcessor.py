import numpy as np
import matplotlib.pyplot as plt
import scienceplots
from scipy.spatial.transform import Rotation, Slerp
from scipy.interpolate import interp1d
from scipy.signal import correlate, correlation_lags
import os

# Set IEEE matplotlib format
plt.style.use(['science', 'ieee', 'no-latex'])

import matplotlib.pyplot as plt

# Name of the .npz log file located in "teleop_logs/"
log_file_name = "teleop_log_20260203_183800"

# Clip window of data recordings
clip_start_time = 8 # seconds
clip_end_time = 38 # seconds

# Detail window of data recordings
detail_start_time = 13 # seconds
detail_end_time = 17 # seconds

# Extract and clip log data
hand_states_log = np.load("teleop_logs/" + log_file_name + ".npz")
data = hand_states_log['data']

timestamps = data[:, 0]
clip_indices = np.searchsorted(timestamps, [clip_start_time, clip_end_time])
timestamps = timestamps[clip_indices[0]:clip_indices[1]] - clip_start_time

robot_left_pos = data[:, 1:4][clip_indices[0]:clip_indices[1]]
robot_left_rot = Rotation.from_quat(data[:, 4:8][clip_indices[0]:clip_indices[1]])
robot_left_grip = data[:, 8][clip_indices[0]:clip_indices[1]]
robot_right_pos = data[:, 9:12][clip_indices[0]:clip_indices[1]]
robot_right_rot = Rotation.from_quat(data[:, 12:16][clip_indices[0]:clip_indices[1]])
robot_right_grip = data[:, 16][clip_indices[0]:clip_indices[1]]
target_left_pos = data[:, 17:20][clip_indices[0]:clip_indices[1]]
target_left_rot = Rotation.from_quat(data[:, 20:24][clip_indices[0]:clip_indices[1]])
target_left_grip = data[:, 24][clip_indices[0]:clip_indices[1]]
target_right_pos = data[:, 25:28][clip_indices[0]:clip_indices[1]]
target_right_rot = Rotation.from_quat(data[:, 28:32][clip_indices[0]:clip_indices[1]])
target_right_grip = data[:, 32][clip_indices[0]:clip_indices[1]]

# ------------------------------------------ Resampling ------------------------------------------

# Frequency statistic
freqs = np.divide(1, timestamps[1:] - timestamps[:-1])
print(f"Minimum sample frequency: {np.min(freqs):.3f}Hz")
print(f"Maximum sample frequency: {np.max(freqs):.3f}Hz")
print(f"Mean sample frequency: {np.mean(freqs):.3f}Hz")

# Interpolate position and orientation trajectories to constant sample rate
resample_rate = 120 # Hz
print(f"Estimated latency computed from resampled trajectories at {resample_rate}Hz")
dt = 1.0 / resample_rate
resampled_timestamps = np.arange(timestamps[0], timestamps[-1] + dt/2, dt)
interp1d_robot_left_pos = interp1d(timestamps, robot_left_pos, kind='cubic', axis=0, bounds_error=False, fill_value="extrapolate")
interp1d_robot_right_pos = interp1d(timestamps, robot_right_pos, kind='cubic', axis=0, bounds_error=False, fill_value="extrapolate")
resampled_robot_left_pos = interp1d_robot_left_pos(resampled_timestamps)
resampled_robot_right_pos = interp1d_robot_right_pos(resampled_timestamps)
interp1d_target_left_pos = interp1d(timestamps, target_left_pos, kind='cubic', axis=0, bounds_error=False, fill_value="extrapolate")
interp1d_target_right_pos = interp1d(timestamps, target_right_pos, kind='cubic', axis=0, bounds_error=False, fill_value="extrapolate")
resampled_target_left_pos = interp1d_target_left_pos(resampled_timestamps)
resampled_target_right_pos = interp1d_target_right_pos(resampled_timestamps)
slerp_robot_left_rot = Slerp(timestamps, robot_left_rot)
slerp_robot_right_rot = Slerp(timestamps, robot_right_rot)
resampled_robot_left_rot = slerp_robot_left_rot(resampled_timestamps)
resampled_robot_right_rot = slerp_robot_right_rot(resampled_timestamps)
slerp_target_left_rot = Slerp(timestamps, target_left_rot)
slerp_target_right_rot = Slerp(timestamps, target_right_rot)
resampled_target_left_rot = slerp_target_left_rot(resampled_timestamps)
resampled_target_right_rot = slerp_target_right_rot(resampled_timestamps)

# ------------------------------------------ Latency ------------------------------------------

# Normalize trajectories to focus on trajectory shape and not amplitude
normalized_robot_pos = np.concatenate([resampled_robot_left_pos, resampled_robot_right_pos], axis=1)
normalized_target_pos = np.concatenate([resampled_target_left_pos, resampled_target_right_pos], axis=1)
normalized_robot_pos = (normalized_robot_pos - np.mean(normalized_robot_pos, axis=0)) / np.std(normalized_robot_pos, axis=0)
normalized_target_pos = (normalized_target_pos - np.mean(normalized_target_pos, axis=0)) / np.std(normalized_target_pos, axis=0)

# Compute cross-correlation of all positional components separately
correlations = []
for i in range(6):
    corr = correlate(normalized_target_pos[:, i], normalized_robot_pos[:, i], mode="full")
    correlations.append(corr)

# Compute lag that maximizes the avarge cross-correlation
correlations = np.mean(correlations, axis=0)
lags = correlation_lags(len(normalized_target_pos[:, 0]), len(normalized_robot_pos[:, 0]), mode="full")
avg_lag = lags[np.argmax(correlations)]
latency = -1 * avg_lag * dt
print(f"Estimated latency: {latency:.3f} seconds")

# Interpolate robot positions and orientations to latency-adjusted timestamps
matched_timestamps = resampled_timestamps + latency
clip_index = np.searchsorted(matched_timestamps, timestamps[-1])
matched_timestamps = matched_timestamps[:clip_index]
matched_robot_left_pos = interp1d_robot_left_pos(matched_timestamps)
matched_robot_right_pos = interp1d_robot_right_pos(matched_timestamps)
matched_robot_left_rot = slerp_robot_left_rot(matched_timestamps)
matched_robot_right_rot = slerp_robot_right_rot(matched_timestamps)

# ------------------------------------------ Trajectory plots ------------------------------------------

# Create trajectory plots
def plot_trajectory(timestamps, pos1, pos2):
    # Create figure and subplots
    traj_fig, (ax_posx, ax_posy, ax_posz) = plt.subplots(3, 1, sharex=True, figsize=(4, 2.5))
    traj_fig.align_ylabels()
    traj_fig.subplots_adjust(hspace=0.05)

    # Plot position components
    ax_posx.plot(timestamps, pos1[:,0], label='Robot')
    ax_posx.plot(timestamps, pos2[:,0], label='Operator')
    ax_posx.set_ylabel('X (m)')
    ax_posx.grid(True)

    ax_posy.plot(timestamps, pos1[:,1] * -1, label='Robot')
    ax_posy.plot(timestamps, pos2[:,1] * -1, label='Operator')
    ax_posy.set_ylabel('-Y (m)')
    ax_posy.grid(True)

    # Plot orientation
    ax_posz.plot(timestamps, pos1[:,2], label='Robot')
    ax_posz.plot(timestamps, pos2[:,2], label='Operator')
    ax_posz.set_xlabel('Time (s)')
    ax_posz.set_ylabel('Z (m)')
    ax_posz.grid(True)

    # Legend
    legend = ax_posx.legend(
        loc="lower right",
        bbox_to_anchor=(0.995, 0.01),
        frameon=True,
        edgecolor="black",
        facecolor="white",
        labelspacing=0.2
    )
    legend.get_frame().set_linewidth(0.55)

    return traj_fig

traj_fig = plot_trajectory(timestamps, robot_right_pos, target_right_pos)
matched_traj_fig = plot_trajectory(matched_timestamps, matched_robot_right_pos, resampled_target_right_pos[:clip_index])

# Create detail trajectory plots
detail_indices = np.searchsorted(timestamps, [detail_start_time, detail_end_time])
traj_detail_fig = plot_trajectory(timestamps[detail_indices[0]:detail_indices[1]], 
                                  robot_right_pos[detail_indices[0]:detail_indices[1]], target_right_pos[detail_indices[0]:detail_indices[1]])
matched_detail_indices = np.searchsorted(matched_timestamps, [detail_start_time, detail_end_time])
matched_traj_detail_fig = plot_trajectory(matched_timestamps[matched_detail_indices[0]:matched_detail_indices[1]], 
                                          matched_robot_right_pos[matched_detail_indices[0]:matched_detail_indices[1]], 
                                          resampled_target_right_pos[:clip_index][matched_detail_indices[0]:matched_detail_indices[1]])

# ------------------------------------------ Error plots ------------------------------------------

# Compute position error
error_left_pos = np.linalg.vector_norm(resampled_robot_left_pos - resampled_target_left_pos, axis=1) * 1000 # meter to millimeter
error_right_pos = np.linalg.vector_norm(resampled_robot_right_pos - resampled_target_right_pos, axis=1) * 1000 # meter to millimeter
error_pos = np.concatenate([error_left_pos, error_right_pos])
print(f"Mean position error: {np.mean(error_pos):.3f}mm")
print(f"Standard deviation position error: {np.std(error_pos):.3f}mm")

# Compute orientation error
forward_vector = np.array([1, 0, 0])
error_left_rot_deg = np.acos(np.clip(np.vecdot(resampled_robot_left_rot.apply(forward_vector), 
                                               resampled_target_left_rot.apply(forward_vector)), 0, 1)) / np.pi * 180
error_right_rot_deg = np.acos(np.clip(np.vecdot(resampled_robot_right_rot.apply(forward_vector), 
                                                resampled_target_right_rot.apply(forward_vector)), 0, 1)) / np.pi * 180
error_rot_deg = np.concatenate([error_left_rot_deg, error_right_rot_deg])
print(f"Mean rotation error: {np.mean(error_rot_deg):.3f}deg")
print(f"Standard deviation rotation error: {np.std(error_rot_deg):.3f}deg")

# Compute matched position error
matched_error_left_pos = np.linalg.vector_norm(matched_robot_left_pos - resampled_target_left_pos[:clip_index], axis=1) * 1000 # meter to millimeter
matched_error_right_pos = np.linalg.vector_norm(matched_robot_right_pos - resampled_target_right_pos[:clip_index], axis=1) * 1000 # meter to millimeter
matched_error_pos = np.concatenate([matched_error_left_pos, matched_error_right_pos])
print(f"Latency-adjusted mean position error: {np.mean(matched_error_pos):.3f}mm")
print(f"Latency-adjusted standard deviation position error: {np.std(matched_error_pos):.3f}mm")

# Compute matched orientation error
forward_vector = np.array([1, 0, 0])
matched_error_left_rot_deg = np.acos(np.clip(np.vecdot(matched_robot_left_rot.apply(forward_vector), 
                                                       resampled_target_left_rot[:clip_index].apply(forward_vector)), 0, 1)) / np.pi * 180
matched_error_right_rot_deg = np.acos(np.clip(np.vecdot(matched_robot_right_rot.apply(forward_vector), 
                                                        resampled_target_right_rot[:clip_index].apply(forward_vector)), 0, 1)) / np.pi * 180
matched_error_rot_deg = np.concatenate([matched_error_left_rot_deg, matched_error_right_rot_deg])
print(f"Latency-adjusted mean rotation error: {np.mean(matched_error_rot_deg):.3f}deg")
print(f"Latency-adjusted standard deviation rotation error: {np.std(matched_error_rot_deg):.3f}deg")

# Create error plots
def plot_error(timestamps, matched_timestamps, error_pos, matched_error_pos, error_rot_deg, matched_error_rot_deg):
    error_fig, (ax_pos, ax_rot) = plt.subplots(2, 1, sharex=True, figsize=(4, 2.5))
    error_fig.align_ylabels()
    error_fig.subplots_adjust(hspace=0.05)

    # Plot position
    ax_pos.plot(timestamps, error_pos, label='Original')
    ax_pos.plot(matched_timestamps, matched_error_pos, label='Latency-Adjusted')
    ax_pos.set_ylabel('Position (mm)')
    ax_pos.grid(True)

    # Plot orientation
    ax_rot.plot(timestamps, error_rot_deg, label='Original')
    ax_rot.plot(matched_timestamps, matched_error_rot_deg, label='Latency-Adjusted')
    ax_rot.set_xlabel('Time (s)')
    ax_rot.set_ylabel('Orientation (deg)')
    ax_rot.grid(True)
    
    # Legend
    legend = ax_pos.legend(
        loc="lower right",
        bbox_to_anchor=(0.995, 0.53),
        frameon=True,
        edgecolor="black",
        facecolor="white",
        labelspacing=0.2
    )
    legend.get_frame().set_linewidth(0.55)

    return error_fig

error_fig = plot_error(resampled_timestamps, matched_timestamps, error_right_pos, matched_error_right_pos, error_right_rot_deg, matched_error_right_rot_deg)

# ------------------------------------------ Save plots ------------------------------------------

# plt.show()
os.makedirs("teleop_graphs", exist_ok=True)
traj_fig.savefig(f"teleop_graphs/{log_file_name}_traj.png")
error_fig.savefig(f"teleop_graphs/{log_file_name}_error.png")
matched_traj_fig.savefig(f"teleop_graphs/{log_file_name}_matched_traj.png")
traj_detail_fig.savefig(f"teleop_graphs/{log_file_name}_traj_detail.png")
matched_traj_detail_fig.savefig(f"teleop_graphs/{log_file_name}_matched_traj_detail.png")
