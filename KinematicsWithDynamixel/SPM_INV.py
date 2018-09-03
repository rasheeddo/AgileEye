import numpy
import math
import maestro
import time

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


rad2deg = 180/math.pi
deg2rad = math.pi/180

################################# Parameters ########################################
eta1 = 0
eta2 = 120.0*deg2rad
eta3 = 240.0*deg2rad
gam = 54.75*deg2rad
bet = gam
alp1 = 90.0*deg2rad
alp2 = alp1

l = 42.426
h = 17.321
b = l/2
c = 24.4947
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

r = 0.0*deg2rad
p = 0.0*deg2rad
y = 0.0*deg2rad

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
Eur_Rot = Eur_Rot.transpose()

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

deg1 = 180.0 - ang1[1]
deg2 = 180.0 - ang2[1]
deg3 = 180.0 - ang3[1]

################################# Servo Drives ########################################

servo = maestro.Controller()
pos0 = servo.getPosition(0)
pre_pos = map(pos0,4000, 8000, 0, 90)

print("pos0=%f" %pos0)
print("pre_pos=%f" %pre_pos)

pos0 = servo.getPosition(0)

acc = 4
servo.setAccel(0,acc)      #set servo 0 acceleration to 4
servo.setAccel(1,acc)      #set servo 0 acceleration to 4
servo.setAccel(2,acc)      #set servo 0 acceleration to 4
'''
speed = 200
servo.setSpeed(0,speed)
servo.setSpeed(1,speed)
servo.setSpeed(2,speed)
'''
#pos = 70
offset1 = 0  #2
offset2 = 0  #3
offset3 = 0  #3
pos1 = deg1+offset1
servo_pos1 = map(pos1, 0.0, 90.0, 4000.0, 8000.0)
pos2 = deg2+offset2
servo_pos2 = map(pos2, 0.0, 90.0, 4000.0, 8000.0)
pos3 = deg3+offset3
servo_pos3 = map(pos3, 0.0, 90.0, 4000.0, 8000.0)
t1 = time.time()
servo.setTarget(0,int(servo_pos1))  #set servo to move to center position  min=3000  mid=6000  max=9000
servo.setTarget(1,int(servo_pos2))
servo.setTarget(2,int(servo_pos3))

t2 = time.time()
period = t2 - t1
print("time=%f" %period)
print("deg1=%f" %deg1)
print("deg2=%f" %deg2)
print("deg3=%f" %deg3)
print("ang1_1=%f" %ang1[0])
print("ang1_2=%f" %ang1[1])
print("ang2_1=%f" %ang2[0])
print("ang2_2=%f" %ang2[1])
print("ang3_1=%f" %ang3[0])
print("ang3_2=%f" %ang3[1])
print("-----------------------------")
servo.close

