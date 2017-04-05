import rosbag
# import csv

def poseToString(pose):
	# convert class data structure to a comma separated string
	result = ""
	result += str(pose.position.x) + ","
	result += str(pose.position.y) + ","
	result += str(pose.position.z) + ","
	result += str(pose.orientation.x) + ","
	result += str(pose.orientation.y) + ","
	result += str(pose.orientation.z) + ","
	result += str(pose.orientation.w)
	
	return result

def jointValueToString(joint_value):
	# ends with comma !!
	result = ""
	result += str(joint_value.time_from_start.secs) + ","
	for value in joint_value.positions:
		result += str(value) + ","
	
	return result


bag = rosbag.Bag('i-profile_with_cost.bag')

# Read joint values (and corresponding time)
joint_values = []
for topic, msg, t in bag.read_messages(topics=['trajectory']):
    joint_values = msg.points

# read end effector poses
ee_poses = []
for topic, msg, t in bag.read_messages(topics=['robotPoses']):
	ee_poses =  msg.poses

# read trajectory poses
traj_poses = []
for topic, msg, t in bag.read_messages(topics=['trajectoryPoses']):
	traj_poses =  msg.poses

bag.close()

# create header for data file
header = "time,j1,j2,j3,j4,j5,j6,"
header += "eepx,eepy,eepz,eeox,eeoy,eeoz,eeow,"
header += "trajpx,trajpy,trajpz,trajox,trajoy,trajoz,trajow"
header += "\n"

#for topic, msg, t in bag.read_messages(topics=['robotPoses', 'trajectory', 'trajectoryPoses']):
nlines = len(traj_poses)
with open('test2.csv', 'wb') as file:
	file.write(header)
	for i in range(0, nlines):
		line = jointValueToString(joint_values[0])
		line += poseToString(ee_poses[0]) + ","
		line += poseToString(traj_poses[0]) + "\n"
		file.write(line) 
    
   
