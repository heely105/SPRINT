# teleop_keyboard_omni3
Generic Keyboard Teleop for 3 Wheeled Omnidirectional robots

- For package details : [ROS Wiki]

- Motion Analysis of 3 wheeled omnidirectional robot (Theory involved in writing the code): [Blog post]

## Installation
1. `cd ~/catkin_ws/src`
2. `git clone https://github.com/YugAjmera/teleop_keyboard_omni3`
3. `cd ~/catkin_ws`
4. `catkin_make`
5. `source ~/catkin_ws/devel/setup.bash`
6. `source ~/.bashrc`

Change the topic names in this [scipt](teleop_keyboard_omni3.py) according to your robot.

## Launch
Run.
```
rosrun teleop_keyboard_omni3 teleop_keyboard_omni3.py 
```

## Usage

"""
Control Your OmniBot!
---------------------------
Moving around:
        w
   a    s    d           o    p
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22)
a/d : increase/decrease angular velocity (Burger : ~ 2.84)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

