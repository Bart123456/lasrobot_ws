import rosbag
import matplotlib.pyplot as plt

# read error of simulation with custom cost function
#~ bag = rosbag.Bag('L_profile_with_weldingcost.bag')
bag = rosbag.Bag('L_profile_costweight_01.bag')
#~ bag = rosbag.Bag('circle.bag')
errorsX = []
errorsY = []
points1 = []
for topic, msg, t in bag.read_messages(topics=['angleErrorsX','angleErrorsY','trajectory']):
	if topic == 'angleErrorsX':
		errorsX = msg.data
    	
	if topic == 'angleErrorsY':
		errorsY = msg.data
    	
	if topic == 'trajectory':
		points1 = msg.points
		
time1 = []
for point in points1:
	time1.append(point.time_from_start.secs + point.time_from_start.nsecs / 1e9)
	
bag.close()

# read error of simulation with standard cost function
bag2 = rosbag.Bag('L_profile_without_weldingcost.bag')
errorsYnocost = []
points2 = []
for topic, msg, t in bag2.read_messages(topics=['angleErrorsY', 'trajectory']):
	if topic == 'angleErrorsY':
		errorsYnocost = msg.data
		
	if topic == 'trajectory':
		points2 = msg.points
		
time2 = []
for point in points2:
	time2.append(point.time_from_start.secs + point.time_from_start.nsecs / 1e9)
		
bag2.close()

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
	
#~ from matplotlib import rc
#~ rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
#~ ## for Palatino and other serif fonts use:
#~ #rc('font',**{'family':'serif','serif':['Palatino']})
#~ rc('text', usetex=True)

# use same font as in latex http://matplotlib.org/users/usetex.html
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
# create plot
plt.plot(time1, errorsY, 'k', time2, errorsYnocost, 'k--', linewidth=2.0)
plt.ylabel(r'Deviation $|\gamma_p - \gamma_{ee}|$ [rad]', fontsize=20)
plt.xlabel('Time [s]', fontsize=18)
plt.axis([0, 40, 0, 1])
plt.legend(['Improved cost function', 'Standard cost function'], fontsize=18)
plt.show()
#~ plt.savefig("compare_cost_functions.png", fontsize=18)
