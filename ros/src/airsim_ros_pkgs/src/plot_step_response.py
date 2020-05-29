import rosbag
import argparse
import numpy as np
parser = argparse.ArgumentParser()
parser.add_argument('--bag_file', type=str)

opt = parser.parse_args()

ODOM_TOPIC = '/airsim_node/PhysXCar/odom_local_ned'
CMD_TOPIC = '/airsim_node/PhysXCar/cmd_vel'

bag = rosbag.Bag(opt.bag_file)

active_cmd = None
cmd_start_time = 0
from matplotlib import pyplot as plt

command_responses = []

for topic, msg, t in bag.read_messages(topics=[ODOM_TOPIC, CMD_TOPIC]):
  if topic == CMD_TOPIC:
    active_cmd = msg
    cmd_start_time = t
    command_responses.append([])    
  elif topic == ODOM_TOPIC and active_cmd:
    x_vel = np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
    target_vel = active_cmd.linear.x
    command_responses[-1].append((t.to_sec(), x_vel, target_vel))

plt.figure()
for response_info in command_responses:
  data = np.array(response_info)
  if len(data):
    plt.scatter(data[:, 0], data[:, 1])
    plt.plot(data[:, 0], data[:, 2])
plt.show()

bag.close()
