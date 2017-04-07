import rosbag
import matplotlib.pyplot as plt

# read error of simulation
bag = rosbag.Bag('L_profile_glitch_without_cost.bag')
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


#remove 'NaN' values from lists (supposed to be 0)
# add sign again, because absolute value is given in bag file
errorsX = list(errorsX)
for i in range(0, len(errorsX)):
	if errorsX[i] == 'nan':
		errorsX[i] = 0.0
		
	if i < 223 and i > 204:
		errorsX[i] = -errorsX[i]
	
count = 0
for error in errorsY:
	if error == 'nan':
		errorsY[count] = 0.0
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
plt.plot(time1, errorsX, 'k.')
plt.ylabel(r'Tolerance angle [rad]', fontsize=20)
plt.xlabel('Time [s]', fontsize=18)
plt.axis([0, 40, -1, 1])
#~ plt.legend(['Improved cost function', 'Standard cost function'], fontsize=18)
#~ plt.show()
plt.savefig("cost_function_glitch.png", fontsize=18)
