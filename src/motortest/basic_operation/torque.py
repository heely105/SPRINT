from dynamixel_helper import DxlHelper

helper = DxlHelper("my_preset.json")

motor_id = 1
motor = helper.get_motor(motor_id)
motor.set_torque(False)