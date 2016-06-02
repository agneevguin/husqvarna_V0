import math
import tf

MAX_ROT = 0.5
MAX_VEL = 1.0
MAX_ACC = 2.

def rotateScan(pos, orientation, target, rotateSpeed, temp1, temp2):
	rotateSpeed = 0.
	euler = tf.transformations.euler_from_quaternion(orientation)

	if (target[0]-pos[0]) != 0:
		theta = math.atan2((target[1]-pos[1]),(target[0]-pos[0]))
	else:
		theta = math.pi/2

	theta = fix_angle_360(theta)
	omega = theta - euler[2]
	omega = fix_angle_180(omega)
	
	if((temp1 == False and temp2 == False) or (temp1 == True and temp2 == False)): #rotate 360 degrees
		Vx = 0.04
		Vy = 0.04
		rotateSpeed = MAX_ROT
		if((omega) > 2.8 or (omega) < -2.8): #update temp1 for completion of 180 degrees
			temp1 = True
		if ((math.fabs(omega) < 0.3) and (temp1 == True)):
			temp2 = True
			#theta = 0.
			#Vx = math.cos(theta) * MAX_VEL
			#Vy = math.sin(theta) * MAX_VEL
			
	print 'Pos : ', pos, 'Temp1 :', temp1, 'Temp2 :', temp2
	#position = math.sqrt(math.pow(Vx,2) + math.pow(Vy,2))
	return rotateSpeed, temp1, temp2
	
def fix_angle_360(angle):
    if angle > math.pi*2.0:
        angle -= math.pi*2.0
    elif angle < 0.:
        angle += math.pi*2.0
    return angle

def fix_angle_180(angle):
    if angle > math.pi:
        angle -= math.pi*2.0
    elif angle < -math.pi:
        angle += math.pi*2.0
    return angle