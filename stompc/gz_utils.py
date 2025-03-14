import os
import sys
import signal
from subprocess import Popen, PIPE

xrce_process = None
gz_process = None
launch_process = None
xrce_cmd = 'MicroXRCEAgent udp4 -p 8888'
room_A_default = "PX4_GZ_MODEL_POSE='-4.0,2.0,0.24'"
room_B_tetris_room = 'PX4_GZ_MODEL_POSE="-3.5,-2.0,0.24"'
room_C_big_room = 'PX4_GZ_MODEL_POSE="6.0,-7.0,0.24"'
room_D_cylinder_map = 'PX4_GZ_MODEL_POSE="-1.0,3.0,0.24"'
launch_file = 'bridges_and_nodes_launch.py'

# SHOULD NOT BE USED, KEPT IN BECAUSE IT MIGHT BE FIXED LATER!!
def run_launch_file(LAUNCH_PATH: str):
    global launch_process
    print('Launching slam toolbox, PointCloud2Laserscan and all bridges')
    launch_process = Popen('ros2 launch {}/{}'.format(LAUNCH_PATH,launch_file),
                           shell=True,
                           stdout=PIPE,
                           stderr=PIPE,
                           )
    print("launch file pid:",launch_process.pid)

def run_gz(GZ_PATH: str, room_name:str):
    global gz_process
    print('starting gz')

    match room_name:
        case "Tetris":
            gz_cmd = 'PX4_SYS_AUTOSTART=4002 HEADLESS=1 ' + room_B_tetris_room + ' make px4_sitl gz_x500_depth_tetrisRoom'
        case "Large":
            gz_cmd = 'PX4_SYS_AUTOSTART=4002 HEADLESS=1 ' + room_C_big_room + ' make px4_sitl gz_x500_depth_largeRoom'
        case "Cylinder":
            gz_cmd = 'PX4_SYS_AUTOSTART=4002 HEADLESS=1 ' + room_D_cylinder_map + ' make px4_sitl gz_x500_depth_cylinderRoom'
        case "Default" | _:
            gz_cmd = 'PX4_SYS_AUTOSTART=4002 HEADLESS=1 ' + room_A_default + ' make px4_sitl gz_x500_depth'

    gz_process = Popen('cd {} && {}'.format(GZ_PATH, gz_cmd),
                       shell=True,
                       stdout=PIPE,
                       stderr=PIPE,
                       )
    print('gazebo pid:', gz_process.pid)

def run_xrce_agent():
    global xrce_process
    print('Starting micro agent')
    xrce_process = Popen(xrce_cmd,
                        shell=True,
                        stdout=PIPE,
                        stderr=PIPE,
                        )
    print('xrce pid:', xrce_process.pid)