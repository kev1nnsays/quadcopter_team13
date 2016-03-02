import lcm
import sys

sys.path.insert(0,'/home/student/Documents/ROB550LAB5/quadrotor_team13/include/lcmtypes/lcmtypes_py')

from pose_t import pose_t
from fsm_state_t import fsm_state_t
from pid_status_t import pid_status_t

pose_cnt = 0
thrust_cnt = 0
roll_cnt = 0
yaw_cnt = 0
pitch_cnt = 0
fsm_cnt = 0

readme = open("testData/readme.txt", 'w')
readme.write("try2_test1.log\n")
readme.write("Pose: count, channels 1-8 \n")
readme.write("Pitch, roll, yaw, thrust: count, error, xref, output \n")
readme.write("FSM: time, time_since_reached, state_name, setpoint, pose,stable,reached \n")
readme.close()

print ("Converting...")
pose_data = open("testData/pose_data.txt", 'w')
pitch_data = open("testData/pitch_data.txt", 'w') 
roll_data = open("testData/roll_data.txt", 'w') 
yaw_data = open("testData/yaw_data.txt", 'w')
thrust_data = open("testData/thrust_data.txt", 'w')
fsm_data = open("testData/fsm_data.txt", 'w')

def pose_handler (channel,data): #count, channels 1-8 
	global pose_cnt
	msg = pose_t.decode(data)
	message = str(pose_cnt)+','+','.join([str(x) for x in msg.channels])
	pose_data.write(message + '\n') 
	pose_cnt += 1

def pid_pitch_handler (channel,data): #count, error, ref, output 
	global pitch_cnt
	msg = pid_status_t.decode(data)
	message = str(pitch_cnt)+','+str(msg.error)+','+str(msg.xref)+','+str(msg.output)
	pitch_data.write(message + '\n') 
	pitch_cnt+=1

def pid_roll_handler (channel,data): #count, error, ref, output 
	global roll_cnt
	msg = pid_status_t.decode(data)
	message = str(roll_cnt)+','+str(msg.error)+','+str(msg.xref)+','+str(msg.output)
	roll_data.write(message + '\n') 
	roll_cnt+=1

def pid_yaw_handler (channel,data): #count, error, ref, output 
	global yaw_cnt
	msg = pid_status_t.decode(data)
	message = str(yaw_cnt)+','+str(msg.error)+','+str(msg.xref)+','+str(msg.output)
	yaw_data.write(message + '\n') 
	yaw_cnt+=1

def pid_thrust_handler (channel,data): #count, error, ref, output 
	global thrust_cnt
	msg = pid_status_t.decode(data)
	message = str(thrust_cnt)+','+str(msg.error)+','+str(msg.xref)+','+str(msg.output)
	thrust_data.write(message + '\n') 
	thrust_cnt+=1


def fsm_handler (channel,data):
	msg = fsm_state_t.decode(data)
	if (str(msg.auto_state) == 'False'):
		auto = 0
	else: 
		auto = 1
	if (str(msg.state_name) == 'MANUAL MODE'):
		state = 0
	else: 
		state = 1
	if (str(msg.stable) == 'False'):
		stable = 0
	else: 
		stable = 1
	if (str(msg.reached) == 'False'):
		reached = 0
	else: 
		reached = 1

	print(auto)

	message = str(auto)+','+str(msg.time_since_reached)+','+str(state)+ ','+ \
			','.join([str(x) for x in msg.setpoint])+ ','+\
			','.join([str(x) for x in msg.pose]) + ',' + \
			str(stable)+','+str(reached)
	fsm_data.write(message + '\n') 



lc = lcm.LCM()
subs_pose = lc.subscribe("POSE",pose_handler)
subs_pid_pitch = lc.subscribe("Pitch",pid_pitch_handler)
subs_pid_roll = lc.subscribe("Roll",pid_roll_handler)
subs_pid_thrust = lc.subscribe("Thrust",pid_thrust_handler)
subs_pid_yaw = lc.subscribe("Yaw",pid_yaw_handler)
subs_fsm = lc.subscribe("FSM",fsm_handler)

while(1):
	lc.handle()

pose_data.close()
pitch_data.close()
roll_data.close()
yaw_data.close()
thrust_data.close()
fsm_data.close()

print ("Done converting.")
