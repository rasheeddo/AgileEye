import numpy
import math
import time
import smbus
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

def map(val, in_min, in_max, out_min, out_max):

	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


def INV(r,p,y):
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

def RunServo123(DEG1,DEG2,DEG3):
	servo_ang1 = map(DEG1, 0.0, 360.0, 0, 4095)
	servo_ang2 = map(DEG2, 0.0, 360.0, 0, 4095)
	servo_ang3 = map(DEG3, 0.0, 360.0, 0, 4095)

	dxl1_goal_position = servo_ang1
	dxl2_goal_position = servo_ang2
	dxl3_goal_position = servo_ang3

	dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
	dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
	dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))

def RunServo456(DEG4,DEG5,DEG6):
	servo_ang4 = map(DEG4, 0.0, 360.0, 0, 4095)
	servo_ang5 = map(DEG5, 0.0, 360.0, 0, 4095)
	servo_ang6 = map(DEG6, 0.0, 360.0, 0, 4095)

	dxl4_goal_position = servo_ang4
	dxl5_goal_position = servo_ang5
	dxl6_goal_position = servo_ang6

	dxl_comm_result4, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, int(dxl4_goal_position))
	dxl_comm_result5, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_POSITION, int(dxl5_goal_position))
	dxl_comm_result6, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_GOAL_POSITION, int(dxl6_goal_position))

def RunServo1(inputDeg1):
    pos1 = inputDeg1
    servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
    dxl1_goal_position = int(servo_com1)
    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
    if dxl_comm_result1 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error1))

def RunServo2(inputDeg2):
    pos2 = inputDeg2
    servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
    dxl2_goal_position = int(servo_com2)
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
    if dxl_comm_result2 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
    elif dxl_error2 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error2))

def RunServo3(inputDeg3):
    pos3 = inputDeg3
    servo_com3 = map(pos3,0.0,360.0,0.0,4095.0)
    dxl3_goal_position = int(servo_com3)
    dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
    if dxl_comm_result3 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result3))
    elif dxl_error3 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error3))

def RunServo4(inputDeg4):
    pos4 = inputDeg4
    servo_com4 = map(pos4,0.0,360.0,0.0,4095.0)
    dxl4_goal_position = int(servo_com4)
    dxl_comm_result4, dxl_error4 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
    if dxl_comm_result4 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result4))
    elif dxl_error4 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error4))

def RunServo5(inputDeg5):
    pos5 = inputDeg5
    servo_com5 = map(pos5,0.0,360.0,0.0,4095.0)
    dxl5_goal_position = int(servo_com5)
    dxl_comm_result5, dxl_error5 = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_POSITION, dxl5_goal_position)
    if dxl_comm_result5 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result5))
    elif dxl_error5 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error5))

def RunServo6(inputDeg6):
    pos6 = inputDeg6
    servo_com6 = map(pos6,0.0,360.0,0.0,4095.0)
    dxl6_goal_position = int(servo_com6)
    dxl_comm_result6, dxl_error6 = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_GOAL_POSITION, dxl6_goal_position)
    if dxl_comm_result6 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result6))
    elif dxl_error6 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error6))

def ReadAngle123():
    dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION)
    
    pre_pos1 = map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
    pre_pos2 = map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)
    pre_pos3 = map(dxl_present_position3, 0.0, 4095.0, 0.0, 360.0)

    return pre_pos1,pre_pos2,pre_pos3

def ReadAngle456():
	dxl_present_position4, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_PRESENT_POSITION)
	dxl_present_position5, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_PRESENT_POSITION)
	dxl_present_position6, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_PRESENT_POSITION)

	pre_pos1 = map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
	pre_pos2 = map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)
	pre_pos3 = map(dxl_present_position3, 0.0, 4095.0, 0.0, 360.0)
	pre_pos4 = map(dxl_present_position4, 0.0, 4095.0, 0.0, 360.0)
	pre_pos5 = map(dxl_present_position5, 0.0, 4095.0, 0.0, 360.0)
	pre_pos6 = map(dxl_present_position6, 0.0, 4095.0, 0.0, 360.0)

	return pre_pos4,pre_pos5,pre_pos6

def GoHome123():
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

def GoHome456():
	Home = 135
	servo_ang1 = map(Home, 0.0, 360.0, 0, 4095)
	servo_ang2 = map(Home, 0.0, 360.0, 0, 4095)
	servo_ang3 = map(Home, 0.0, 360.0, 0, 4095)

	dxl1_goal_position = servo_ang1  
	dxl2_goal_position = servo_ang2
	dxl3_goal_position = servo_ang3

	dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
	dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
	dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))

def SetOperationMode(MODE):
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, MODE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, MODE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, MODE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_OPERATING_MODE, MODE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_OPERATING_MODE, MODE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_OPERATING_MODE, MODE)

def CheckOperationMode():
	present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE)
	present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE)
	present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE)
	present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_OPERATING_MODE)
	present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_OPERATING_MODE)
	present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_OPERATING_MODE)
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

def TorqueOn():
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	print("Enable Toruqe...")

def TorqueOff():
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	print("Disable Toruqe...")
	
def SetGoalCurrent(SetCur):
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
		print("Goal Current is set")

def SetProfile1(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 500       # 350 Default                  [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 1: %d" %set_V_PRFL)
    print("A PRFL 1: %d" %set_A_PRFL)
    print("--------------------------------")

def SetProfile2(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 500       # 350 Default                  [0.229RPM]

    #set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
    #set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 2: %d" %set_V_PRFL)
    print("A PRFL 2: %d" %set_A_PRFL)
    print("--------------------------------") 

def SetProfile3(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 500       # 350 Default                  [0.229RPM]

    #set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
    #set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 3: %d" %set_V_PRFL)
    print("A PRFL 3: %d" %set_A_PRFL)
    print("--------------------------------")    

def SetProfile4(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 500       # 350 Default                  [0.229RPM]

    #set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
    #set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 4: %d" %set_V_PRFL)
    print("A PRFL 4: %d" %set_A_PRFL)
    print("--------------------------------")

def SetProfile5(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 500       # 350 Default                  [0.229RPM]

    #set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
    #set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 5: %d" %set_V_PRFL)
    print("A PRFL 5: %d" %set_A_PRFL)
    print("--------------------------------")    

def SetProfile6(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 500       # 350 Default                  [0.229RPM]

    #set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
    #set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 6: %d" %set_V_PRFL)
    print("A PRFL 6: %d" %set_A_PRFL)
    print("--------------------------------")

def SetPID1(set_P_Gain,set_I_Gain,set_D_Gain):
    
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
    
    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 1: %d" %position_P_gain)
    print("Position I Gain 1: %d" %position_I_gain)
    print("Position D Gain 1: %d" %position_D_gain)
    print("------------------------------")

def SetPID2(set_P_Gain,set_I_Gain,set_D_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 2: %d" %position_P_gain)
    print("Position I Gain 2: %d" %position_I_gain)
    print("Position D Gain 2: %d" %position_D_gain)
    print("------------------------------")

def SetPID3(set_P_Gain,set_I_Gain,set_D_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 3: %d" %position_P_gain)
    print("Position I Gain 3: %d" %position_I_gain)
    print("Position D Gain 3: %d" %position_D_gain)
    print("------------------------------")

def SetPID4(set_P_Gain,set_I_Gain,set_D_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)    

    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 4: %d" %position_P_gain)
    print("Position I Gain 4: %d" %position_I_gain)
    print("Position D Gain 4: %d" %position_D_gain)
    print("------------------------------")

def SetPID5(set_P_Gain,set_I_Gain,set_D_Gain):

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 5: %d" %position_P_gain)
    print("Position I Gain 5: %d" %position_I_gain)
    print("Position D Gain 5: %d" %position_D_gain)
    print("------------------------------")

def SetPID6(set_P_Gain,set_I_Gain,set_D_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 6: %d" %position_P_gain)
    print("Position I Gain 6: %d" %position_I_gain)
    print("Position D Gain 6: %d" %position_D_gain)
    print("------------------------------")

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
ADDR_PRO_FEEDFORWARD_2nd_GAIN = 88
ADDR_PRO_FEEDFORWARD_1st_GAIN = 90

ADDR_PRO_MOVING             = 122
ADDR_PRO_MOVING_STATUS       = 123

CURRENT_CONTROL                     = 0
POSITION_CONTROL                    = 3 # Default
CURRENT_BASED_POSITION_CONTROL      = 5
# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                             # Dynamixel ID: 1
DXL2_ID                      = 2                             # Dynamixel ID: 2
DXL3_ID                      = 3                             # Dynamixel ID: 3
DXL4_ID                      = 4
DXL5_ID                      = 5
DXL6_ID                      = 6

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

######################################### MPU setting #######################################

rad2deg = 180.0/(math.pi)
Total_Ang_X = 0
Total_Ang_Y = 0
period = 0 # initial value of period

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

######################### Setting ##############################

#TorqueOff()
#SetOperationMode(POSITION_CONTROL)

TorqueOn()

######################### Set Velocity / Acceleration Profile  ##############################
SetProfile4(250,250)
SetProfile5(250,250)
SetProfile6(250,250)

######################### Set PID Gain Position Loop  ##############################
SetPID4(1200,100,2000)
SetPID5(1200,100,2000)
SetPID6(1200,100,2000)

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

GoHome456()
time.sleep(2)
DegX_pre = 0
DegY_pre = 0

r = 0
p = 0
y = 0

while True:
	startTime = time.time()


	#Read Accelerometer raw value
	acc_x = read_raw_data(ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_YOUT_H)
	acc_z = read_raw_data(ACCEL_ZOUT_H)

	#Read Gyroscope raw value
	gyro_x = read_raw_data(GYRO_XOUT_H)
	gyro_y = read_raw_data(GYRO_YOUT_H)
	gyro_z = read_raw_data(GYRO_ZOUT_H)

	#Full scale range +/- 250 degree/C as per sensitivity scale factor
	Ax = acc_x/16384.0
	Ay = acc_y/16384.0
	Az = acc_z/16384.0

	Gx = gyro_x/131.0
	Gy = gyro_y/131.0
	Gz = gyro_z/131.0

	Acc_Ang_X = math.atan(Ay/math.sqrt(math.pow(Ax,2) + math.pow(Az,2))) * rad2deg
	Acc_Ang_Y = math.atan(-1*Ax/math.sqrt(math.pow(Ay,2) + math.pow(Az,2))) * rad2deg
	Total_Ang_X = 0.98*(Total_Ang_X + Gx*period) + 0.02*Acc_Ang_X
	Total_Ang_Y = 0.98*(Total_Ang_Y + Gy*period) + 0.02*Acc_Ang_Y


	OffSetX = -0.4
	OffSetY = 7.7
	Deg_X = Total_Ang_X + OffSetX
	Deg_Y = Total_Ang_Y + OffSetY

	r = -int(Deg_X)*deg2rad
	p = -int(Deg_Y)*deg2rad
	y = 0.0*deg2rad

	if r >= 0.8 or p >= 0.8 or y >= 0.9:
		r = 0
		p = 0
		y = 0
		print("RPY Input out of range!")
	else:
		#print("RPY Inputs are valid")
		del_DegX = abs(DegX_pre - Deg_X)
		del_DegY = abs(DegY_pre - Deg_Y)
		print("del_DegX: %f" %del_DegX)
		print("del_DegY: %f" %del_DegY)


		#DEG = INV(r,p,y)

		#DEG4 = DEG[0]
		#DEG5 = DEG[1]
		#DEG6 = DEG[2]
		#print("RunServo456")
		#print("Deg4: %f" %DEG4)
		#print("Deg5: %f" %DEG5)
		#print("Deg6: %f" %DEG6)
		#RunServo456(DEG4,DEG5,DEG6)

	DegX_pre = Deg_X
	DegY_pre = Deg_Y





	#time.sleep(0.05)
	endTime = time.time()
	period = endTime - startTime

	print("AngleX:%f" %Deg_X, "AngleY:%f" %Deg_Y)
	print("Period: %f" %period)
	print("-------------------------------------------------")


#################################################################
'''
r = 0.0*deg2rad
p = P*deg2rad
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
#################################################################
r = R*deg2rad
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
#################################################################
r = -R*deg2rad
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
#################################################################
r = 0.0*deg2rad
p = 0.0*deg2rad
y = Y*deg2rad

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
#################################################################
r = 0.0*deg2rad
p = 0.0*deg2rad
y = -Y*deg2rad

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

t2 = time.time()
period = t2 - t1

'''
