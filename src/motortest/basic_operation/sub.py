import time, motor1, motor2


wannatime = time.time() + 5

while True:
	motor1.cw('go')
	motor2.cw('go')
	
	if time.time() > wannatime:
		break
	
	motor1.cw('stop')
	motor2.cw('stop')
