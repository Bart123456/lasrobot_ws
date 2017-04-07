import rosbag
import matplotlib.pyplot as plt

# read error of simulation
bag = rosbag.Bag('L_profile_glitch_test.bag')
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
	
#~ from matplotlib import rc
#~ rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
#~ ## for Palatino and other serif fonts use:
#~ #rc('font',**{'family':'serif','serif':['Palatino']})
#~ rc('text', usetex=True)

# use same font as in latex http://matplotlib.org/users/usetex.html
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
# create plot
plt.plot(time1, errorsX, 'k')
plt.ylabel(r'Deviation $|\gamma_p - \gamma_{ee}|$ [rad]', fontsize=20)
plt.xlabel('Time [s]', fontsize=18)
plt.axis([0, 40, 0, 1])
#~ plt.legend(['Improved cost function', 'Standard cost function'], fontsize=18)
plt.show()
#~ plt.savefig("cost_function_glitch.png", fontsize=18)
