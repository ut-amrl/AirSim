import rosbag
import argparse
import numpy as np
parser = argparse.ArgumentParser()
parser.add_argument('--bag_file', type=str)

opt = parser.parse_args()

ODOM_TOPIC = '/airsim_node/PhysXCar/odom_local_ned/'
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
    time_delta = (t - cmd_start_time).to_sec()
    x_vel = msg.twist.twist.linear.x
    target_vel = active_cmd.linear.x
    command_responses[-1].append((time_delta, target_vel - x_vel))    
  print(msg)

import pdb; pdb.set_trace()
for response_info in command_responses:
  data = np.array(response_info)
  plt.figure()
  plt.scatter(data[:, 0], data[:, 1])
  plt.show()

bag.close()
