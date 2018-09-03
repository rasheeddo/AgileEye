import numpy
import math
import time
import pygame
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

######################################## Pygame #############################################
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
#print 'Initialized Joystick : %s' % j.get_name()
"""
Returns a vector of the following form:
[LThumbstickX, LThumbstickY, Unknown Coupled Axis???, 
RThumbstickX, RThumbstickY, 
Button 1/X, Button 2/A, Button 3/B, Button 4/Y, 
Left Bumper, Right Bumper, Left Trigger, Right Triller,
Select, Start, Left Thumb Press, Right Thumb Press]
Note:
No D-Pad.
Triggers are switches, not variable. 
Your controller may be different
"""

def rc_map(x,in_min,in_max,out_min,out_max):
    result = (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min
    return result

def getButton():
    #Read input from the two joysticks
    pygame.event.pump()
    #unknown1 = j.get_axis(0)
    #unknown2 = j.get_axis(1)
    #throttle = j.get_axis(2)
    #roll = j.get_axis(4)
    #pitch = j.get_axis(3)
    #yaw = j.get_axis(5)
    button0 = j.get_button(0)
    button1 = j.get_button(1)
    button2 = j.get_button(2)
    button3 = j.get_button(3)
    button4 = j.get_button(4)
    button5 = j.get_button(5)
    button6 = j.get_button(6)
    button7 = j.get_button(7)
    button8 = j.get_button(8)
    button9 = j.get_button(9)
    button10 = j.get_button(10)
    joy_button = [button0, button1, button2, button3, button4, button5, button6, button7,button8, button9, button10]
    
    return joy_button

def getAxis():
    #Read input from the two joysticks
    pygame.event.pump()
    axis0 = j.get_axis(0)
    axis1 = j.get_axis(1)
    axis2 = j.get_axis(2)
    axis3 = j.get_axis(4)
    axis4 = j.get_axis(3)
    axis5 = j.get_axis(5)
    joy_axis = [axis0, axis1, axis2, axis3, axis4, axis5]
    return joy_axis

def getHat():
    pygame.event.pump()
    hat0 = j.get_hat(0)
    
    joy_hat = hat0
    return joy_hat

def map(val, in_min, in_max, out_min, out_max):

	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def INV(r,p,y):
		################################# Euler rotation ########################################
	psi = y
	theta = p
	phi = r

	E11 = math.cos(psi)*math.cos(theta)
	E12 = math.cos(psi)*math.sin(phi)*math.sin(theta)-math.cos(phi)*math.sin(psi)
	E13 = math.sin(phi)*math.sin(psi)+math.cos(phi)*math.cos(psi)*math.sin(theta)

	E21 = math.cos(theta)*math.sin(psi)
	E22 = math.cos(phi)*math.cos(psi) + math.sin(phi)*math.sin(psi)*math.sin(theta)
	E23 = math.cos(phi)*math.sin(psi)*math.sin(theta)-math.cos(psi)*math.sin(phi)

	E31 = -math.sin(theta)
	E32 = math.cos(theta)*math.sin(phi)
	E33 = math.cos(phi)*math.cos(theta)

	Eur_Rot = numpy.matrix([[E11,E12,E13],[E21,E22,E23],[E31,E32,E33]])
	#Eur_Rot = Eur_Rot.transpose()

	PVV1 = Eur_Rot*pv1
	PVV2 = Eur_Rot*pv2
	PVV3 = Eur_Rot*pv3

	magV1 = math.sqrt(PVV1[0]**2 + PVV1[1]**2 + PVV1[2]**2)
	magV2 = math.sqrt(PVV2[0]**2 + PVV2[1]**2 + PVV2[2]**2)
	magV3 = math.sqrt(PVV3[0]**2 + PVV3[1]**2 + PVV3[2]**2)

	v1=PVV1/magV1
	v2=PVV2/magV2
	v3=PVV3/magV3

	################################# Inverse Kinematics ########################################

	A1 = -(-math.sin(eta1)*math.sin(gam)*math.cos(alp1) + math.sin(eta1)*math.cos(gam)*math.sin(alp1))*v1[0] + (math.cos(eta1)*math.sin(gam)*math.cos(alp1) - math.cos(eta1)*math.cos(gam)*math.sin(alp1))*v1[1] + (math.cos(gam)*math.cos(alp1) - math.sin(gam)*math.sin(alp1))*v1[2] - math.cos(alp2);
	B1 = -(math.cos(eta1)*math.sin(alp1))*v1[0] + (math.sin(eta1)*math.sin(alp1))*v1[1];
	C1 = -(-math.sin(eta1)*math.sin(gam)*math.cos(alp1) - math.sin(eta1)*math.cos(gam)*math.sin(alp1))*v1[0] + (math.cos(eta1)*math.sin(gam)*math.cos(alp1) + math.cos(eta1)*math.cos(gam)*math.sin(alp1))*v1[1] + (-math.cos(gam)*math.cos(alp1) + math.sin(gam)*math.sin(alp1))*v1[2] - math.cos(alp2);

	A2 = -(-math.sin(eta2)*math.sin(gam)*math.cos(alp1) + math.sin(eta2)*math.cos(gam)*math.sin(alp1))*v2[0] + (math.cos(eta2)*math.sin(gam)*math.cos(alp1) - math.cos(eta2)*math.cos(gam)*math.sin(alp1))*v2[1] + (math.cos(gam)*math.cos(alp1) - math.sin(gam)*math.sin(alp1))*v2[2] - math.cos(alp2);
	B2 = -(math.cos(eta2)*math.sin(alp1))*v2[0] + (math.sin(eta2)*math.sin(alp1))*v2[1];
	C2 = -(-math.sin(eta2)*math.sin(gam)*math.cos(alp1) - math.sin(eta2)*math.cos(gam)*math.sin(alp1))*v2[0] + (math.cos(eta2)*math.sin(gam)*math.cos(alp1) + math.cos(eta2)*math.cos(gam)*math.sin(alp1))*v2[1] + (-math.cos(gam)*math.cos(alp1) + math.sin(gam)*math.sin(alp1))*v2[2] - math.cos(alp2);

	A3 = -(-math.sin(eta3)*math.sin(gam)*math.cos(alp1) + math.sin(eta3)*math.cos(gam)*math.sin(alp1))*v3[0] + (math.cos(eta3)*math.sin(gam)*math.cos(alp1) - math.cos(eta3)*math.cos(gam)*math.sin(alp1))*v3[1] + (math.cos(gam)*math.cos(alp1) - math.sin(gam)*math.sin(alp1))*v3[2] - math.cos(alp2);
	B3 = -(math.cos(eta3)*math.sin(alp1))*v3[0] + (math.sin(eta3)*math.sin(alp1))*v3[1];
	C3 = -(-math.sin(eta3)*math.sin(gam)*math.cos(alp1) - math.sin(eta3)*math.cos(gam)*math.sin(alp1))*v3[0] + (math.cos(eta3)*math.sin(gam)*math.cos(alp1) + math.cos(eta3)*math.cos(gam)*math.sin(alp1))*v3[1] + (-math.cos(gam)*math.cos(alp1) + math.sin(gam)*math.sin(alp1))*v3[2] - math.cos(alp2);

	T1 = [None]*2
	T2 = [None]*2
	T3 = [None]*2
	ang1 = [None]*2
	ang2 = [None]*2
	ang3 = [None]*2

	T1[0] = (-B1+math.sqrt(B1**2 - A1*C1))/A1
	T1[1] = (-B1-math.sqrt(B1**2 - A1*C1))/A1
	T2[0] = (-B2+math.sqrt(B2**2 - A2*C2))/A2
	T2[1] = (-B2-math.sqrt(B2**2 - A2*C2))/A2
	T3[0] = (-B3+math.sqrt(B3**2 - A3*C3))/A3
	T3[1] = (-B3-math.sqrt(B3**2 - A3*C3))/A3

	ang1[0] = math.atan(T1[0])*2*rad2deg
	ang2[0] = math.atan(T2[0])*2*rad2deg
	ang3[0] = math.atan(T3[0])*2*rad2deg
	ang1[1] = math.atan(T1[1])*2*rad2deg
	ang2[1] = math.atan(T2[1])*2*rad2deg
	ang3[1] = math.atan(T3[1])*2*rad2deg

	deg1 = ang1[1]
	deg2 = ang2[1]
	deg3 = ang3[1]

	return deg1, deg2, deg3

def RPYOutRange(roll,pitch,yaw):

    if ((roll >= 1) or (pitch >= 1) or (roll<=-1) or (pitch<=-1) or (yaw>= 1) or (yaw<= -1)):
        print("---------------------------------------")
        print("---------------------------------------")
        print("------Jogging out of safety range!-----")
        print("---------------------------------------")
        print("---------------------------------------")
        WarningFlag = True
    
    else:
        WarningFlag = False

    return WarningFlag

def ServoDrive(DEG1,DEG2,DEG3):
	servo_ang1 = map(DEG1, 0.0, 360.0, 0, 4095)
	servo_ang2 = map(DEG2, 0.0, 360.0, 0, 4095)
	servo_ang3 = map(DEG3, 0.0, 360.0, 0, 4095)

	dxl1_goal_position = servo_ang1
	dxl2_goal_position = servo_ang2
	dxl3_goal_position = servo_ang3

	dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
	dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
	dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))

def GoHome():
	Home = 135
	servo_ang1 = map(Home, 0.0, 360.0, 0, 4095)
	servo_ang2 = map(Home, 0.0, 360.0, 0, 4095)
	servo_ang3 = map(Home, 0.0, 360.0, 0, 4095)

	dxl1_goal_position = servo_ang1  
	dxl2_goal_position = servo_ang2
	dxl3_goal_position = servo_ang3

	dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
	dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
	dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))

####################################################### Set Servo Configuration #############################################################

						############################## Control table address ##############################
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

ADDR_PRO_CURRENT_LIMIT      = 38
ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_PRESENT_CURRENT    = 126 

ADDR_PRO_OPERATING_MODE     = 11

ADDR_PRO_GOAL_VELOCITY      = 104

ADDR_PRO_ACCELERATION_LIMIT = 40
ADDR_PRO_VELOCITY_LIMIT     = 44
ADDR_PRO_PROFILE_ACCELERATION  = 108
ADDR_PRO_PROFILE_VELOCITY   = 112

ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84

ADDR_PRO_MOVING             = 122

CURRENT_CONTROL                     = 0
POSITION_CONTROL                    = 3 # Default
CURRENT_BASED_POSITION_CONTROL      = 5
# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                             # Dynamixel ID: 1
DXL2_ID                      = 2                             # Dynamixel ID: 2
DXL3_ID                      = 3                             # Dynamixel ID: 3
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
												# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
	print("Succeeded to open the port")
else:
	print("Failed to open the port")
	print("Press any key to terminate...")
	getch()
	quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
	print("Succeeded to change the baudrate")
else:
	print("Failed to change the baudrate")
	print("Press any key to terminate...")
	getch()
	quit()

######################### Disable Torque Before Changing Mode  ##############################
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
print("Disable Toruqe...")

######################### Check Operating Mode  ##############################

						############### Choose Mode ########################
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
if dxl_comm_result != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error))
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
if dxl_comm_result != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error))
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
if dxl_comm_result != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error))

						############### Check Present Mode ########################
present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE)
present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE)
present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE)
if present_mode == 0:
	# Current (Torque) Control Mode
	print("Now Operating Mode is Torque Control")
elif present_mode == 3:
	# Position Control Mode
	print("Now Operating Mode is Position Control")
elif present_mode == 5:
	# Current-based Position Control Mode
	print("Now Operating Mode is Current-based Position Control")
else:
	print("In other Mode that didn't set!")

# Enable Dynamixel Torque 1
dxl_comm_result1, dxl_error1 = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result1 != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
elif dxl_error1 != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error1))
else:
	print("Dynamixel 1 has been successfully connected")

# Enable Dynamixel Torque 2
dxl_comm_result2, dxl_error2 = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result2 != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
elif dxl_error2 != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error2))
else:
	print("Dynamixel 2 has been successfully connected")

# Enable Dynamixel Torque 3
dxl_comm_result3, dxl_error3 = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result3 != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result3))
elif dxl_error3 != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error3))
else:
	print("Dynamixel 3 has been successfully connected")

######################### Set Goal Current  ##############################
SetCur = 30
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
if dxl_comm_result != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
	print("Goal Current is set")


######################### Set Velocity / Acceleration Profile  ##############################
set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
set_V_Limit = 350       # 350 Default                  [0.229RPM]

final_pos = 90.0          # deg
t3 = 3.0                  # second
t1 = t3/3              # second
t2 = 2*t1               # second

#dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
#start_pos = map(dxl_present_position,0.0,4095.0,0.0,360.0)
#start_pos = 0
#delta_pos = final_pos - start_pos       # deg.
#delta_pos_rev = delta_pos/360.0           # Rev
#set_V_PRFL = (64.0*delta_pos)/(t2*100)     # Rev/Min
#set_A_PRFL = (64.0*set_V_PRFL)/(t1*100)    # Rev/Min^2


set_A_PRFL = 200     # between 0 ~ set_A_limit      [214.577 rev/min^2]
set_V_PRFL = 200       # between 0 ~ set_V_Limit      [0.229RPM]

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT)
velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT)
acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT)
velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT)
acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_ACCELERATION_LIMIT)
velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_LIMIT)
#print("Initial Position: %f" %start_pos)
#print("Final Position: %f" %final_pos)
#print("Travel time: %d" %t3)
print("V PRFL: %f" %set_V_PRFL)
print("A PRFL: %f" %set_A_PRFL)
print("Acceleration Limited: %d" %acceleration_limit)
print("Velocity Limited: %d" %velocity_limit)
print("--------------------------------")

######################### Set PID Gain Position Loop  ##############################
set_P_Gain = 800    #800 default
set_I_Gain = 30     #0 default
set_D_Gain = 2000   #4700 default

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
print("PID's Gain are set")

position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN)
position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN)
position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN)
position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN)
position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN)
position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN)
position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D_GAIN)
position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I_GAIN)
position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P_GAIN)

print("Position D Gain: %d" %position_D_gain)
print("Position I Gain: %d" %position_I_gain)
print("Position P Gain: %d" %position_P_gain)
print("--------------------------------")


####################################################### Read Initial positions #############################################################
# Read present position before it moves
dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
if dxl_comm_result1 != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
elif dxl_error1 != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error1))
dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
if dxl_comm_result2 != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
elif dxl_error2 != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error2))
dxl_present_position3, dxl_comm_result3, dxl_error3 = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION)
if dxl_comm_result3 != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result3))
elif dxl_error3 != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error3))

pos1_now = dxl_present_position1
ang1_now = map(pos1_now,0,4095,0.0,360.0)
pos2_now = dxl_present_position2
ang2_now = map(pos2_now,0,4095,0.0,360.0)
pos3_now = dxl_present_position3
ang3_now = map(pos3_now,0,4095,0.0,360.0)

print("Present angle1: %f" %ang1_now)
print("Present angle2: %f" %ang2_now)
print("Present angle3: %f" %ang3_now)
print("----------------------------")
time.sleep(0.1)



################################# Constant ########################################
rad2deg = 180/math.pi
deg2rad = math.pi/180
root2 = math.sqrt(2)
root3 = math.sqrt(3)

################################# Parameters ########################################
t1 = time.time()

eta1 = 0
eta2 = 120.0*deg2rad
eta3 = 240.0*deg2rad
gam = 54.75*deg2rad
bet = gam
alp1 = 90.0*deg2rad
alp2 = alp1

l = 55
s = l*root2
h = math.sqrt(l**2 - (l*root2*root3/3)**2)
b = s/2
c = s*math.sqrt(3)/3
a = math.sqrt(c**2 - b**2)
v1x = -b
v1y = a
v1z = h
v2x = b
v2y = a
v2z = h
v3x = 0
v3y = -c
v3z = h
vxc = 0
vyc = 0
vzc = h

pv1 = numpy.matrix([[v1x], [v1y], [v1z]])
pv2 = numpy.matrix([[v2x], [v2y], [v2z]])
pv3 = numpy.matrix([[v3x], [v3y], [v3z]])


#################################################################

################################################## Get Joy Stick ##############################################

Hats = getHat()
JogDir = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1 

Buttons = getButton()
Cir_Btn = Buttons[0] #A
P_Btn = Buttons[1] #B
R_Btn = Buttons[2] #X
Y_Btn = Buttons[3] #Y
BackHome_Btn = Buttons[6] #Back
Start_Btn = Buttons[7] #Start
Exit_Btn = Buttons[8] #Logiccool

Axes = getAxis()
Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
Ax2 = Axes[2]       # LT unpressed = -1, LT pressed = 1
Ax3 = Axes[3]       #Analog right push down = +1, Analog right push up = -1
Ax4 = Axes[4]       #Analog right push right = +1, Analog right push left = -1
Ax5 = Axes[5]       # RT unpressed = -1, RT pressed = 1

LinearIncrement = 1
Rmin = -55
Rmax = 55
Pmin = -55
Pmax = 55
Ymin = -55
Ymax = 55

################## Go to stand by position before starting  ###########################

GoHome()

#############################################################################################

waitForStart = True
print("----------------------------------------------------------------------------------")
print("-----------------------Press Start Button to Jog!---------------------------------")
print("----------------------------------------------------------------------------------")

while waitForStart:

    Buttons = getButton()
    Start_Btn = Buttons[7] #Start

    if Start_Btn == 1:
        waitForStart = False
        startJog = True

    time.sleep(0.1)

time.sleep(1)
print("...The Agile Eye is ready to move...")

while startJog:
	
	############ Receive Value From Joy Stick All The Time ###############
	Buttons = getButton()
	Cir_Btn = Buttons[0] #A
	P_Btn = Buttons[1] #B
	R_Btn = Buttons[2] #X
	Y_Btn = Buttons[3] #Y
	Back_Btn = Buttons[6] #Back
	Start_Btn = Buttons[7] #Start
	Exit_Btn = Buttons[8] #Logiccool
	
	if Back_Btn == 1:
		print("Agile Eye is going home...")
		GoHome()

	if Exit_Btn == 1:
		GoHome()
		print("----------------------------------------------------------------------------------")
		print("---------------------------Exit the Agile Eye Jog Mode---------------------------")
		print("----------------------------------------------------------------------------------")
		break

	while R_Btn == 1:
		Buttons = getButton()
		R_Btn = Buttons[2] #X
		Axes = getAxis()
		Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1

		Rcom = map(Ax1,-1,1,Rmin,Rmax)
		Rcom = Rcom*deg2rad
		Pcom = 0.0*deg2rad
		Ycom = 0.0*deg2rad
		RPYOutRange(Rcom,Pcom,Ycom)
		DEG = INV(Rcom,Pcom,Ycom)
		DEG1 = DEG[0]
		DEG2 = DEG[1]
		DEG3 = DEG[2]
		ServoDrive(DEG1,DEG2,DEG3)

	while P_Btn == 1:
		Buttons = getButton()
		P_Btn = Buttons[1] #B
		Axes = getAxis()
		Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1

		Pcom = map(Ax0,-1,1,Pmin,Pmax)
		Pcom = Pcom*deg2rad
		Rcom = 0.0*deg2rad
		Ycom = 0.0*deg2rad
		RPYOutRange(Rcom,Pcom,Ycom)
		DEG = INV(Rcom,Pcom,Ycom)
		DEG1 = DEG[0]
		DEG2 = DEG[1]
		DEG3 = DEG[2]
		ServoDrive(DEG1,DEG2,DEG3)

	while Y_Btn == 1:
		Buttons = getButton()
		Y_Btn = Buttons[3] #Y
		Axes = getAxis()
		Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1

		Ycom = map(Ax0,-1,1,Ymin,Ymax)
		Ycom = Ycom*deg2rad
		Rcom = 0.0*deg2rad
		Pcom = 0.0*deg2rad
		RPYOutRange(Rcom,Pcom,Ycom)
		DEG = INV(Rcom,Pcom,Ycom)
		DEG1 = DEG[0]
		DEG2 = DEG[1]
		DEG3 = DEG[2]
		ServoDrive(DEG1,DEG2,DEG3)

	while Cir_Btn == 1:
		Buttons = getButton()
		Cir_Btn = Buttons[0] #A
		Axes = getAxis()
		Ax0 = Axes[0]
		Ax1 = Axes[1]
		Rcom = map(Ax1,-1,1,-45,45)
		Pcom = map(Ax0,-1,1,-45,45)
		print("R angle: %f" %Rcom)
		print("P angle: %f" %Pcom)
		print("-------------------------")
		Rcom = Rcom*deg2rad
		Pcom = Pcom*deg2rad
		Ycom = 0.0*deg2rad
		RPYOutRange(Rcom,Pcom,Ycom)
		DEG = INV(Rcom,Pcom,Ycom)
		DEG1 = DEG[0]
		DEG2 = DEG[1]
		DEG3 = DEG[2]
		ServoDrive(DEG1,DEG2,DEG3)

	GoHome()

	


'''
r = 0.0*deg2rad
p = 0.0*deg2rad
y = 0.0*deg2rad

if r >= 1 or p >=1 or y >= 0.9:
	r = 0
	p = 0
	y = 0
	print("RPY Input out of range!")
else:
	print("RPY Inputs are valid")

DEG = INV(r,p,y)

DEG1 = DEG[0]
DEG2 = DEG[1]
DEG3 = DEG[2]

servoDrive(DEG1,DEG2,DEG3)
time.sleep(1)
GoHome()
time.sleep(1)
'''
