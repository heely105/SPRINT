import motor1, motor2, motor3, time


print("============w(straignt)===========")
print("======a(left)       d(right)======")
print("============s(back)===============")



while True:

	
	button = raw_input("Press the direction : ")

	if button == 'w':

		motor1.cw('stop')
		motor2.ccw('go')
		motor3.cw('go')

	

	elif button == 's':
		
		motor1.ccw('stop')
		motor2.cw('go')
		motor3.ccw('go')


	elif button == 'a':
	
		motor1.cw('go')
		motor2.ccw('go')
		motor3.ccw('go')

	elif button == 'd':

		motor1.ccw('go')
		motor2.cw('go')
		motor3.cw('go')

	elif button == 'p':

		motor1.cw('stop')
		motor2.cw('stop')
		motor3.cw('stop')
                      
	else:

		print("Wrong")

