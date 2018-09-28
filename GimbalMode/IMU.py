import time
#import signal
#import sys
import math
import smbus
#import PID
import RPi.GPIO as GPIO


###################################### Motor Setting##########################################
'''
ena1 = 13
IN1 = 19
IN2 = 26
ena2 = 16
IN3 = 20
IN4 = 21

Freq = 50
stop = 0
minDC = 10
maxDC = 100

GPIO.setmode(GPIO.BCM)

GPIO.setup(ena1, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

GPIO.setup(ena2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

pwm1 = GPIO.PWM(ena1, Freq)
pwm2 = GPIO.PWM(ena2, Freq)
#pwmL = GPIO.PWM(motorL, Freq)

pwm1.start(0)
pwm2.start(0)
'''
def map(x, inmin, inmax, outmin, outmax):
    return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin

'''
######################################## PID parameters #######################################
kp = 1
ki = 0.000
kd = 0.000
setpoint = 0  # Desired angle 0deg
maxLim = 20
minLim = -20
Input = 0
output =0
pid = PID.PID(kp,ki,kd)
pid.SetPoint=0.0
pid.setSampleTime(0.00001)
'''
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


bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")
################################################################################

################################################################################
##########################         MAIN           ############################################
try:
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


        OffSetX = -1.4
        OffSetY = -3.8
        Deg_X = Total_Ang_X + OffSetX
        Deg_Y = Total_Ang_Y + OffSetY

        ################## MAIN code start from here #####################
        '''
        offSet = 1.1
        Input = Total_Ang_X + offSet
        pid.update(Input)
        output = pid.output

        if abs(Input) < 0.4:
                
            GPIO.output(IN1, True)
            GPIO.output(IN2, True)
            GPIO.output(IN3, True)
            GPIO.output(IN4, True)
            drive1 = stop
            drive2 = stop

            pwm1.ChangeDutyCycle(drive1)
            pwm2.ChangeDutyCycle(drive2)

        else:
            if output < 0:
                GPIO.output(IN1, True)
                GPIO.output(IN2, False)
                GPIO.output(IN3, False)
                GPIO.output(IN4, True)
                # map = (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin
                x = output
                output_map = (x - (0)) * (maxDC - minDC) / (-10 - (0)) + minDC
                drive1 = output_map
                drive2 = output_map
            
            else:
                GPIO.output(IN1, False)
                GPIO.output(IN2, True)
                GPIO.output(IN3, True)
                GPIO.output(IN4, False)
                # map = (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin
                x = output
                output_map = (x - (0)) * (maxDC - minDC) / (10 - (0)) + minDC
                drive1 = output_map
                drive2 = output_map
               
            if drive1 < minDC or drive2 < minDC:
                drive1 = minDC
                drive2 = minDC
            elif drive1 > maxDC or drive2 > maxDC:
                drive1 = maxDC
                drive2 = maxDC

            pwm1.ChangeDutyCycle(drive1)
            pwm2.ChangeDutyCycle(drive2)
        '''
        endTime = time.time()
        period = endTime - startTime
        
        print("AngleX:%f" %Deg_X, "AngleY:%f" %Deg_Y)
        print("Period: %f" %period)
        print("-------------------------------------------------")
            
except KeyboardInterrupt:
    GPIO.cleanup()
GPIO.cleanup()