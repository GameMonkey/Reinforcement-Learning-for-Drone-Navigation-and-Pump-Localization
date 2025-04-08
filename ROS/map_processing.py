import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.srv import GetMap

import numpy as np
import threading
import math
import os
import sys

sys.path.insert(0, '../')
from stompc.classes import State, MapConfig
from stompc.utils import get_map_index_of_pump

class MapServiceCaller(Node):
    def __init__(self) -> None:
        super().__init__('map_listener')
        self.cli = self.create_client(GetMap, '/slam_toolbox/dynamic_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMap.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        


def process_map_data(drone_x: float, drone_y: float, map_config: MapConfig) -> State:
    msg = None
    map_service_instance = MapServiceCaller()
    map_service_instance.send_request()
    while rclpy.ok():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(map_service_instance)
        executor.spin_once()
        if map_service_instance.future.done():
            try:
                response = map_service_instance.future.result()
            except Exception as e:
                map_service_instance.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
               msg = response.map
            break

    #print("map service called")
    width = msg.info.width
    height = msg.info.height
    granularity = round(msg.info.resolution,2)
    matrix = []
    row = []
    data = msg.data

    all_pumps = map_config.fake_pumps + map_config.pumps

    x_offset = abs(math.floor((msg.info.origin.position.x / granularity)))
    y_offset = abs(math.floor((msg.info.origin.position.y / granularity)))

    x_index = math.floor((drone_x) / granularity) + x_offset
    y_index = math.floor(((drone_y * -1)) / granularity) + y_offset

    for i in range(0, len(data), width):
        row = data[i:i + width]
        matrix.append(row)

    state = State(matrix, x_index, y_index, width, height, granularity, x_offset, y_offset)
    for pump in all_pumps:
        pump_x_index, pump_y_index = get_map_index_of_pump(state,pump)
        #print(pump_x_index, pump_y_index)
        if (pump_x_index >= state.map_width or pump_y_index >= state.map_height 
            or pump_x_index < 0 or pump_y_index < 0)\
                or matrix[pump_y_index][pump_x_index] == -1:
            continue
        elif pump.has_been_discovered == False:
            matrix[pump_y_index][pump_x_index] = 2
        elif pump.has_been_discovered == True and pump in map_config.fake_pumps:
            matrix[pump_y_index][pump_x_index] = 0
        elif pump.has_been_discovered == True and pump in map_config.pumps:
            matrix[pump_y_index][pump_x_index] = 3


    x = 0
    y = 0
    with open('matrix.txt', 'w') as testfile:
        for row in matrix:
            x=x+1
            string_row = []
            for a in row:
                y=y+1
                if x == y_offset and y == x_offset:
                    string_row.append("M")
                if x == y_index and y == x_index:
                    string_row.append("*")
                elif a == -1:
                    string_row.append("?")
                elif a == 0:
                    string_row.append("+")
                elif a == 100:
                    string_row.append("-")
                elif a == 2:
                    string_row.append("!")
            testfile.write(', '.join(string_row) + '\n')
            y = 0
            
    return state