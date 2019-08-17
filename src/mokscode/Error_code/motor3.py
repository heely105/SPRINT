

def ccw(num):

	from dynamixel_helper import DxlHelper
	helper = DxlHelper("preset3.json", verbosity="detailed")
	motor = helper.get_motor(3)
	dxl_unit, res = motor.get_present_position()
	motor.set_torque(False)
	motor.set_operating_mode(1)
	motor.set_torque(True)

	if num == 'stop':
		motor.set_torque(False)
	elif num == 'go':
		print(motor.set_goal_velocity(200))


def cw(num):

	from dynamixel_helper import DxlHelper
	helper = DxlHelper("preset3.json", verbosity="detailed")
	motor = helper.get_motor(3)
	dxl_unit, res = motor.get_present_position()
	motor.set_torque(False)
	motor.set_operating_mode(1)
	motor.set_torque(True)

	if num == 'stop':
		motor.set_torque(False)
	elif num == 'go':
		print(motor.set_goal_velocity(-200))
