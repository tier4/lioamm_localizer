#!/usr/bin/python3
import argparse
from datetime import datetime
import math
import os

from scipy.spatial import KDTree

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tier4_debug_msgs.msg import Float32Stamped
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosidl_runtime_py.utilities import get_message
import tf_transformations

plt.style.use("seaborn")
plt.rcParams["font.size"] = 12


def load_rosbag(path:str, target_topic:str):
  storage_options = StorageOptions(uri=path, storage_id="sqlite3")
  converter_options = ConverterOptions(
    input_serialization_format="cdr", output_serialization_format="cdr"
  )
  reader = SequentialReader()
  reader.open(storage_options, converter_options)

  topic_type_list = {}
  for topic_type in reader.get_all_topics_and_types():
    topic_type_list[topic_type.name] = topic_type.type

  pose = []
  while reader.has_next():
    topic, data, _ = reader.read_next()

    msg = deserialize_message(data, get_message(topic_type_list[topic]))

    if topic == target_topic:
      stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
      quat = [
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
      ]
      _, _, yaw = tf_transformations.euler_from_quaternion(quat)
      pose.append((stamp, msg.pose.position.x, msg.pose.position.y, yaw))

  return np.array(pose)

def find_nearest_poses(query_trajectory, ref_trajectory):
    correspondence = np.zeros((query_trajectory.shape[0], 4))

    tree = KDTree(ref_trajectory[:, 1:3])

    for i, query_pose in enumerate(query_trajectory):
      dist, idx = tree.query(query_pose[1:3])
      ref_pose = ref_trajectory[idx]

      correspondence[i, :] = ref_pose
    return correspondence


def sync(query, ref):
  correspondence = np.zeros((query.shape[0], 4))
  for i in range(query.shape[0]):
    match_idx = np.argmin(np.abs(query[i, 0] - ref[:, 0]))
    correspondence[i, :] = ref[match_idx, :]
  return correspondence


def frange(start, end, step):
  list = [start]
  n = start
  while n + step < end:
    n = n + step
    list.append(n)
  return list


def ced(data_list, min, max):
  result = []
  for i in frange(min, max, 0.001):
    count = 0
    for data in data_list:
      if data < i:
        count += 1
    if count == 0 or len(data_list) == 0:
      result.append(0.0)
    else:
      result.append(count / len(data_list))
  return result


def calc_euclid_error(gt_data, eval_data):
  euclid_error = []
  for idx in range(gt_data.shape[0]):
    dx = gt_data[idx, 1] - eval_data[idx, 1]
    dy = gt_data[idx, 2] - eval_data[idx, 2]
    dist = math.sqrt(dx**2 + dy**2)
    euclid_error.append(dist)
  return euclid_error


def calc_lateral_error(gt_data, eval_data):
  lateral_error = []
  for idx in range(gt_data.shape[0]):
    base = np.array([np.cos(gt_data[idx, 3]), np.sin(gt_data[idx, 3]), 0.0])
    dx = gt_data[idx, 1] - eval_data[idx, 1]
    dy = gt_data[idx, 2] - eval_data[idx, 2]
    diff_vec = np.array([dx, dy, 0.0])
    cross_vec = np.cross(base, diff_vec)
    lateral_error.append(abs(cross_vec[2]))
  return lateral_error

def calculate_errors(gt_data, eval_data):
  lateral_errors = []
  longitudinal_errors = []

  for idx in range(gt_data.shape[0]):
    base = np.array([np.cos(gt_data[idx, 3]), np.sin(gt_data[idx, 3]), 0.0])

    dx = gt_data[idx, 1] - eval_data[idx, 1]
    dy = gt_data[idx, 2] - eval_data[idx, 2]
    diff_vec = np.array([dx, dy, 0.0])

    cross_vec = np.cross(base, diff_vec)
    lateral_error = cross_vec[2]
    lateral_errors.append(lateral_error)

    longitudinal_error = np.dot(base[:2], diff_vec[:2])
    longitudinal_errors.append(longitudinal_error)

  return lateral_errors, longitudinal_errors

def plot_ced(gt_data, eval_data):
  fig = plt.figure(figsize=(10, 10))
  lateral_error_threshold = frange(0.0, 1.0, 0.001)
  euclid_error_threshold = frange(0.0, 1.0, 0.001)

  # sync pose topic
  if len(gt_data) < len(eval_data):
      sync_data = sync(gt_data, eval_data)
      eval_data = sync_data
  else:
      sync_data = sync(eval_data, gt_data)
      gt_data = sync_data

  # calculation lateral / euclid pose error
  lateral_error = calc_lateral_error(gt_data, eval_data)
  euclid_error = calc_euclid_error(gt_data, eval_data)

  # calculation ced curve
  lateral_ced_result = ced(lateral_error, 0.0, 1.0)
  euclid_ced_result = ced(euclid_error, 0.0, 1.0)

  ax1 = fig.add_subplot(1, 2, 1, aspect=1)
  ax1.plot(lateral_error_threshold, lateral_ced_result, label="lateral error")
  ax1.set_xlabel("Error [m]")
  ax1.set_ylabel("Ratio [0.0~1.0]")
  ax1.set_ylim(0.0, 1.0)
  ax1.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  ax1.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  ax1.minorticks_on()
  plt.title("CED Curve for Lateral Error")
  plt.legend()

  ax2 = fig.add_subplot(1, 2, 2, aspect=1)
  ax2.plot(euclid_error_threshold, euclid_ced_result, label="euclid error")
  ax2.set_xlabel("Error [m]")
  ax2.set_ylabel("Ratio [0.0~1.0]")
  ax2.set_ylim(0.0, 1.0)
  ax2.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  ax2.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  ax2.minorticks_on()
  plt.title("CED Curve for Euclid Error")
  plt.legend()

  # plt.savefig(file_name)
  plt.show()


def plot_pose_error(gt_data, eval_data):
  fig = plt.figure(figsize=(18, 10))

  # sync pose topic
  # if len(gt_data) < len(eval_data):
      # sync_data = sync(gt_data, eval_data)
      # eval_data = sync_data
  # else:
  sync_data = sync(eval_data, gt_data)
  gt_data = sync_data

  # calculation lateral / euclid pose error
  lateral_error = calc_lateral_error(gt_data, eval_data)
  euclid_error = calc_euclid_error(gt_data, eval_data)

  plt.subplot(1, 2, 1, aspect=1)
  plt.scatter(
      eval_data[:, 1],
      eval_data[:, 2],
      vmin=0.0,
      vmax=1.0,
      c=lateral_error,
      s=5,
      cmap=cm.jet,
      label='lateral error"',
  )
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")
  plt.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  plt.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  plt.minorticks_on()
  plt.colorbar()
  plt.title("Lateral Pose Error of evaluation data")
  plt.legend()

  plt.subplot(1, 2, 2, aspect=1)
  plt.scatter(
      eval_data[:, 1],
      eval_data[:, 2],
      vmin=0.0,
      vmax=1.0,
      c=euclid_error,
      s=5,
      cmap=cm.jet,
      label="euclid error",
  )
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")
  plt.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  plt.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  plt.minorticks_on()
  plt.colorbar()
  plt.title("Euclid Pose Error of evaluation data")
  plt.legend()

  plt.suptitle("Pose Error with color map")

  # plt.savefig(file_name)
  plt.show()

def plot_trajectory(gt_pose, eval_pose):
  plt.subplot(1,1,1,aspect=1)
  plt.scatter(gt_pose[:,1], gt_pose[:,2], label='autoware', s=5,c='red')
  plt.scatter(eval_pose[:,1], eval_pose[:,2], label='lioamm_localizer',s=5, c='blue')
  plt.legend()
  plt.show()

def plot_score(gt_pose, eval_pose, gt_score, eval_score):
  plt.figure(figsize=(18, 10))

  plt.subplot(1, 2, 1)
  plt.plot(gt_score[:, 0], gt_score[:, 1], label="Score of ground truth score")
  plt.plot(eval_score[:, 0], eval_score[:, 1], label="Score of evaluation data")
  plt.xlabel("stamp")
  plt.ylabel("score")
  plt.legend()

  plt.subplot(1, 2, 2, aspect=1)
  plt.scatter(
      eval_pose[:, 1],
      eval_pose[:, 2],
      vmin=min(eval_score[:, 1]),
      vmax=max(eval_score[:, 1]),
      c=eval_score[:, 1],
      cmap=cm.jet,
      label="Score of evaluation data with color map",
  )
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")
  plt.legend()
  plt.colorbar()

  plt.suptitle("Scan Matching Score")

  # plt.savefig(file_name)
  plt.show()

def plot_result(autoware_pose, autoware_ndt_pose, lioamm_localizer_pose):
  fig = plt.figure(figsize=(18, 10))

  sync_data = find_nearest_poses(lioamm_localizer_pose, autoware_pose)
  autoware_pose = sync_data

  lateral_error, longitudinal_error = calculate_errors(autoware_pose, lioamm_localizer_pose)

  plt.subplot(2, 2, 1, aspect=1)
  plt.scatter(
      lioamm_localizer_pose[:, 1],
      lioamm_localizer_pose[:, 2],
      vmin=0.0,
      vmax=0.5,
      c=lateral_error,
      s=5,
      cmap=cm.jet,
      label='lateral error',
  )
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")
  plt.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  plt.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  plt.minorticks_on()
  plt.colorbar()
  plt.title("Lateral Pose Error of evaluation data")
  plt.legend()

  plt.subplot(2, 2, 2, aspect=1)
  plt.scatter(
      lioamm_localizer_pose[:, 1],
      lioamm_localizer_pose[:, 2],
      vmin=0.0,
      vmax=0.5,
      c=longitudinal_error,
      s=5,
      cmap=cm.jet,
      label="longitudinal error",
  )
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")
  plt.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  plt.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  plt.minorticks_on()
  plt.colorbar()
  plt.title("Longitudinal Pose Error of evaluation data")
  plt.legend()

  plt.subplot(2,2,3)
  plt.plot(range(len(lateral_error)), lateral_error, label="lateral error")
  plt.xlabel("Error [m]")
  plt.ylabel("Ratio [0.0~1.0]")
  plt.ylim(-1.0,1.0)
  plt.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  plt.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  plt.minorticks_on()
  plt.title("Lateral Error")
  plt.legend()

  plt.subplot(2,2,4)
  plt.plot(range(len(longitudinal_error)), longitudinal_error, label="longitudinal error")
  plt.xlabel("Error [m]")
  plt.ylabel("Ratio [0.0~1.0]")
  plt.ylim(-1.0, 1.0)
  plt.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  plt.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  plt.minorticks_on()
  plt.title("Longitudinal Error")

  plt.suptitle("Autoware vs lioamm_localizer")

  fig1 = plt.figure(figsize=(18, 10))

  lateral_error_threshold = frange(0.0, 1.0, 0.001)
  longitudinal_error_threshold = frange(0.0, 1.0, 0.001)
  lateral_ced_result = ced(np.fabs(lateral_error), 0.0, 1.0)
  longitudinal_ced_result = ced(np.fabs(longitudinal_error), 0.0, 1.0)

  plt.subplot(1,2,1)
  plt.plot(lateral_error_threshold, lateral_ced_result, label="lateral error")
  plt.xlabel("Error [m]")
  plt.ylabel("Ratio [0.0~1.0]")
  plt.ylim(0.0,1.0)
  plt.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  plt.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  plt.minorticks_on()
  plt.title("CED Curve for Lateral Error")
  plt.legend()

  plt.subplot(1,2,2)
  plt.plot(longitudinal_error_threshold, longitudinal_ced_result, label="longitudinal error")
  plt.xlabel("Error [m]")
  plt.ylabel("Ratio [0.0~1.0]")
  plt.ylim(0.0, 1.0)
  plt.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  plt.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  plt.minorticks_on()
  plt.title("CED Curve for Longitudinal Error")

  fig2 = plt.figure(figsize=(18,10))

  plt.subplot(1,1,1,aspect=1)
  plt.scatter(
      lioamm_localizer_pose[:, 1],
      lioamm_localizer_pose[:, 2],
      s=8,
      c='red',
      label='lioamm localizer',
  )
  plt.scatter(
      autoware_pose[:, 1],
      autoware_pose[:, 2],
      s=8,
      c='blue',
      label='Autoware EKF',
  )
  plt.scatter(
      autoware_ndt_pose[:, 1],
      autoware_ndt_pose[:, 2],
      s=8,
      c='green',
      label='Autoware NDT',
  )
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")
  plt.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
  plt.grid(which="minor", axis="both", color="black", alpha=0.2, linestyle="--", linewidth=1)
  plt.minorticks_on()
  plt.legend()
  plt.show()


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--ground_truth_data")
  parser.add_argument("--evaluation_data")
  # parser.add_argument("--output", default=datetime.now().strftime("%y-%m-%d-%H-%M-%S"))

  args = parser.parse_args()

  autoware_ekf_pose = load_rosbag(args.ground_truth_data, "/localization/pose_twist_fusion_filter/pose")
  autoware_ndt_pose = load_rosbag(args.ground_truth_data, "/localization/pose_estimator/pose")
  lioamm_localizer_pose = load_rosbag(args.evaluation_data, "/map_pose")

  plot_result(autoware_ekf_pose, autoware_ndt_pose, lioamm_localizer_pose)

  # create directory to save for result image
  # save_directory = os.path.join("result", args.output)
  # os.makedirs(save_directory, exist_ok=True) if not os.path.isdir(save_directory) else None

  # load ground truth rosbag(without noise)
  # gt_ekf_pose, gt_ndt_pose, gt_score = load_rosbag(args.ground_truth_data)
  # load evaluation rosbag(with noise)
  # eval_ekf_pose, eval_ndt_pose, eval_score = load_rosbag(args.evaluation_data)

  # plot ced curve
  # plot_ced(gt_ekf_pose, eval_ekf_pose)
  # plot_ced(gt_ndt_pose, eval_ndt_pose)

  # plot pose error with color map
  # plot_pose_error(gt_ekf_pose, eval_ekf_pose)
  # plot_pose_error(gt_ndt_pose, eval_ndt_pose)

  # plot score
  # plot_score(gt_ndt_pose, eval_ndt_pose, gt_score, eval_score)


if __name__ == "__main__":
  main()
