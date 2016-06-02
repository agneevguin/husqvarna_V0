import math
import numpy
import tf

MAX_ROT = 0.4
MAX_VEL = 5.
MAX_ACC = 2.

def updateMotion(pos, vel, orientation, target, index, prev_error, prev_error_theta, dt):
	velocity = 0.
	delta_error = 0.
	error_theta = 0.
	delta_error_theta = 0.
	rotateSpeed = 0.
	euler = tf.transformations.euler_from_quaternion(orientation)
	#roll = euler[0]
	#pitch = euler[1]
	#yaw = euler[2]

	if math.sqrt(math.pow((target[1]-pos[1]),2) + math.pow((target[0]-pos[0]),2)) < 0.5:
		position = 0
		rotateSpeed = 0
		prev_error = 0
		prev_error_theta = 0
		index += 1
	else:
	
		if (target[0]-pos[0]) != 0:
			theta = math.atan2((target[1]-pos[1]),(target[0]-pos[0]))
		else:
			theta = (90*math.pi)/180
			
		aligned = (euler[2] > theta - 0.1 and euler[2] < theta + 0.1)
		
		if (aligned):
			
			Vx = math.cos(theta) * MAX_VEL;
			Vy = math.sin(theta) * MAX_VEL;
			print 'theta', theta
			print 'target-pos', target[0]-pos[0]
			if (target[0]-pos[0]) < 0:
				Vx = -Vx
		else:
			Vx = 0.
			if(euler[2] - theta > 0):
				rotateSpeed -= math.pi*MAX_ROT
			else:
				rotateSpeed += math.pi*MAX_ROT
				
		'''	
		#ROTATIONAL MOTION
		Kp = 5
		Kd = 2
	
		if (theta > math.pi):
			theta = -(theta - math.pi)
		elif (theta < -math.pi):
			theta = (theta + math.pi)
		
		error_theta = theta - euler[2]
		print 'error_theta', error_theta
		print 'euler[0]', euler[0]
		print 'euler[1]', euler[1]
		print 'euler[2]', euler[2]
		print 'theta', theta
		if (error_theta > math.pi):
			error_theta = -(error_theta - math.pi)
		elif (error_theta < -math.pi):
			error_theta = (error_theta + math.pi)
		
		delta_error_theta = prev_error_theta - error_theta

		rotateSpeed = Kp * error_theta - Kd * delta_error_theta
			
		if (rotateSpeed > MAX_ROT):
			rotateSpeed = MAX_ROT
		elif (rotateSpeed < -MAX_ROT):
			rotateSpeed = -MAX_ROT
		
		print 'delta_error_theta', delta_error_theta
		print 'rotateSpeed', rotateSpeed
		
		if (error_theta > -0.05 or error_theta < 0.05):
			#TRANSLATIONAL MOTION
			Kp = 5
			Kd = 2

			error = math.sqrt(math.pow((target[1]-pos[1]),2) + math.pow((target[0]-pos[0]),2))

			delta_error = error - prev_error
			at = Kp * error - Kd * delta_error
			if (at > MAX_ACC):
				at = MAX_ACC
			elif (at < (-MAX_ACC)):
				at = (-MAX_ACC)
			velocity = velocity + at;
			if (velocity > MAX_VEL):
				velocity = MAX_VEL
			elif (velocity < (-MAX_VEL)):
				velocity = (-MAX_VEL)
			
			theta = math.atan2((pos[1]-target[1]),(pos[0]-target[0]))
			vx = (velocity * math.cos(theta))
			ax = (at * math.cos(theta))
			position = (vx)
			prev_error = error
		prev_error_theta = error_theta
		'''
		position = Vx
	return position, rotateSpeed, prev_error, prev_error_theta, index
