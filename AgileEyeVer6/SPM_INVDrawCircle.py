import numpy
import math
import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

def map(val, in_min, in_max, out_min, out_max):

	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

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

########################### Draw circular path ############################
r = 1					
zeta = [None]*360
X = [None]*360
Y1 = [None]*180
Y2 = [None]*180
Y = [None]*360

for i in range(1,361):
    zeta[i-1]=i*deg2rad
    X[i-1] = r*math.cos(zeta[i-1])

for i in range(1,(len(X)/2)+1):
    Y1[i-1] = math.sqrt(r**2 - X[i-1]**2)

j = 0
for i in range((len(X)/2)+1,len(X)+1):
    Y2[j] = -math.sqrt(r**2 - X[j]**2)
    j = j+1

Y = numpy.concatenate((Y1,Y2))

################################################################################
radius = 12.
delaySpeed = 0
increment = 5
i = 0

try:
	while i < 360:

		r = radius*Y[i]*deg2rad
		p = radius*X[i]*deg2rad
		y = 0.0*deg2rad

		if r >= 1 or p >=1 or y >= 0.55:
			r = 0
			p = 0
			y = 0
			print("RPY Input out of range!")
			break

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

		################################# Servo Drives ########################################

		servo_ang1 = map(deg1, 0.0, 360.0, 0, 4095)
		servo_ang2 = map(deg2, 0.0, 360.0, 0, 4095)
		servo_ang3 = map(deg3, 0.0, 360.0, 0, 4095)

		dxl1_goal_position = servo_ang1
		dxl2_goal_position = servo_ang2
		dxl3_goal_position = servo_ang3

		Moving1, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MOVING)
		Moving2, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MOVING)
		Moving3, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_MOVING)


		dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
		dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
		dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))

		i = i + increment

		time.sleep(delaySpeed)

except (KeyboardInterrupt, SystemExit):
	
	# Disable Dynamixel Torque 1
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	# Disable Dynamixel Torque 2
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	# Disable Dynamixel Torque 3
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	
	# Close port
	portHandler.closePort()


time.sleep(0.5)

################################## Go Home ########################################################
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