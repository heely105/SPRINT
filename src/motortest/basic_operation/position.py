from dynamixel_helper import DxlHelper

helper = DxlHelper("my_preset.json")

motor_id = 1
motor = helper.get_motor(motor_id)

motor.set_torque(False)
motor.set_operating_mode(3)

motor.set_torque(True)

dxl_unit, res = motor.get_present_position()
motor.set_goal_position((dxl_unit + 2000) % 4096)

