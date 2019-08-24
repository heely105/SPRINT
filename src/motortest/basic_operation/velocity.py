from dynamixel_helper import DxlHelper

helper = DxlHelper("my_preset.json")

motor = helper.get_motor(1)

dxl_unit, res = motor.get_present_position()

motor.set_torque(False)

motor.set_operating_mode(1)

motor.set_torque(False)

print(motor.set_goal_velocity(200))
