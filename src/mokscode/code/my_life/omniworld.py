import motor1, motor2, motor3, time

print("============w(straignt)===========")
print("======a(left)       d(right)======")
print("============s(back)===============")


while True:

	
	button = raw_input("Press the direction : ")

	if button == 'w':

		wannatime = time.time() + 5

		while True:
			motor2.ccw('go')
			motor3.cw('go')
	
			if time.time() > wannatime:
				break
		
		motor2.ccw('stop')
		motor3.cw('stop')


	elif button == 's':

		wannatime = time.time() + 5

		while True:
			motor2.cw('go')
			motor3.ccw('go')
	
			if time.time() > wannatime:
				break

		motor2.cw('stop')
		motor3.ccw('stop')

	elif button == 'a':
		wannatime = time.time() + 5

		while True:
			motor1.cw('go')
			motor2.ccw('go')
			motor3.ccw('go')
				
			if time.time() > wannatime:
				break

		motor1.cw('stop')
		motor2.ccw('stop')
		motor3.ccw('stop')

	elif button == 'd':
		wannatime = time.time() + 5

		while True:
			motor1.ccw('go')
			motor2.cw('go')
			motor3.cw('go')
			
			if time.time() > wannatime:
				break

		motor1.ccw('stop')
		motor2.cw('stop')
		motor3.cw('stop')


                      
	else:

		print("Wrong")

