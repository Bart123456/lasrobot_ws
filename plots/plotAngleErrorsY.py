import rosbag
bag = rosbag.Bag('L_ITTOL_Cost.bag')
errorsX = []
errorsY = []
points = []
for topic, msg, t in bag.read_messages(topics=['angleErrorsX','angleErrorsY','trajectory']):
	if topic == 'angleErrorsX':
		errorsX = msg.data
    	
	if topic == 'angleErrorsY':
		errorsY = msg.data
    	
	if topic == 'trajectory':
		points = msg.points
bag.close()

bag2 = rosbag.Bag('L_ITTOL_noCost.bag')
errorsYnocost = []
for topic, msg, t in bag2.read_messages(topics=['angleErrorsY']):
	if topic == 'angleErrorsY':
		errorsYnocost = msg.data
bag2.close()

time = []
joint_positions = []
for point in points:
	time.append(point.time_from_start.secs + point.time_from_start.nsecs / 1e9)
	joint_positions.append(point.positions)

#remove 'NaN' values from lists (supposed to be 0)
count = 0
for error in errorsX:
	if error == 'nan':
		errorsX[count] = 0.0
	count += 1
	
count = 0
for error in errorsY:
	if error == 'nan':
		errorsY[count] = 0.0
	count += 1
	
count = 0
for error in errorsYnocost:
	if error == 'nan':
		errorsYnocost[count] = 0.0
	count += 1
	
import matplotlib.pyplot as plt
plt.plot(time, errorsY, 'k', time, errorsYnocost, 'r', linewidth=2.0)
plt.ylabel('Angle Y Error [rad]')
plt.xlabel('Time [s]')
#plt.axis([0, 9, -0.5, 1])
plt.grid('on')
plt.legend(['With cost function', 'Without cost function'])
#~ plt.show()
plt.savefig("L_ITTOL_angleErrorsY_Cost.png")
#~ plt.savefig("compare_cost_functions.png", fontsize=18)
