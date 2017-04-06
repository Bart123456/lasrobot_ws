import rosbag
import matplotlib.pyplot as plt

# read bag files
bag = rosbag.Bag('L_profile_without_weldingcost.bag')
points = []
for topic, msg, t in bag.read_messages(topics=['trajectory']):
    points = msg.points
bag.close()

time = []
joint_positions = []
for point in points:
	time.append(point.time_from_start.secs + point.time_from_start.nsecs / 1e9)
	joint_positions.append(point.positions)

joint1 = []
joint2 = []
joint3 = []
joint4 = []
joint5 = []
joint6 = []
for joints in joint_positions:
	joint1.append(joints[0])
	joint2.append(joints[1])
	joint3.append(joints[2])
	joint4.append(joints[3])
	joint5.append(joints[4])
	joint6.append(joints[5])

#~ import matplotlib.pyplot as plt
#~ plt.plot(time, joint1, 'b', time, joint2, 'g',time, joint3, 'r',time, joint4, 'c',time, joint5, 'm',time, joint6, 'y')
#~ plt.ylabel('Joint angles [rad]')
#~ plt.xlabel('Time [s]')
#~ plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'])
#~ plt.show()

jwithout = [[],[],[],[],[],[]]
for joints in joint_positions:
	for i in range(0, 6):
		jwithout[i].append(joints[i])

plt.figure()
for i in range(0, 6):
	plt.plot(time, jwithout[i])
plt.show()
