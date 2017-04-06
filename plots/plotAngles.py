import rosbag
import matplotlib.pyplot as plt

# read bag cylinder A
# ===================
bag = rosbag.Bag('test.bag')
points = []
for topic, msg, t in bag.read_messages(topics=['trajectory']):
    points = msg.points
bag.close()

tA = []
joint_positions = []
for point in points:
	tA.append(point.time_from_start.secs + point.time_from_start.nsecs / 1e9)
	joint_positions.append(point.positions)

jA = [[],[],[],[],[],[]]
for joints in joint_positions:
	for i in range(0, 6):
		jA[i].append(joints[i])

# read bag l profile B
# ====================
bag = rosbag.Bag('L_profile_without_weldingcost.bag')
points = []
for topic, msg, t in bag.read_messages(topics=['trajectory']):
    points = msg.points
bag.close()

tA = []
joint_positions = []
for point in points:
	tA.append(point.time_from_start.secs + point.time_from_start.nsecs / 1e9)
	joint_positions.append(point.positions)

jA = [[],[],[],[],[],[]]
for joints in joint_positions:
	for i in range(0, 6):
		jA[i].append(joints[i])

# create plot
# ===========
plt.figure()
plt.subplot(121)
for i in range(0, 6):
	plt.plot(tA, jA[i])
plt.title("Case A: cylinder")
plt.ylabel('Joint angles [rad]')
plt.xlabel('Time [s]')
#~ plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'])

plt.subplot(122)
for i in range(0, 6):
	plt.plot(tA, jA[i])
plt.title("Case B: l-profile")
plt.xlabel('Time [s]')
plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'])

plt.savefig("joint_angles_AB.png")
#~ plt.show()
