import math

from classes import Pump,MapConfig


###########################################
### BASELINE MAP WITH ONE PUMP - ROOM A ###
###########################################
def get_baseline_one_pump_config(granularity=0.05):
    pump = Pump(-4.0,-7.2)
    fake_pump = Pump(0,-8.5)
    config = MapConfig(pumps=[pump], fake_pumps=[fake_pump], n_cells_in_area=math.ceil((6/granularity)*(10.2/granularity)))
    return config


##################################
### CYLINEDER MAP WITH 3 PUMPS ###
##################################
def get_baseline_cylinder_room_config():
    pump1 = Pump(0,-3.5)
    pump2 = Pump(-4.5,2.3)
    pump3 = Pump(-5.5,-5.2)

    config = MapConfig(pumps=[pump1,pump2,pump3], fake_pumps=[], n_cells_in_area=31415)
    return config

##################################
### BASELINE MAP WITH ONE PUMP ###
##################################
def get_baseline_big_room_config():
    pump = Pump(0,14)
    pump2 = Pump(11.5, 14)
    pump3 = Pump(11.5, 0)
    fake_pump = Pump(5.5,14)
    
    config = MapConfig(pumps=[pump, pump2, pump3], fake_pumps=[fake_pump], n_cells_in_area=144400)
    return config


def get_baseline_tetris_room_config(granularity=0.05):
    pump = Pump(4,-4)
    fake_pump = Pump(0,-7)
    #Rough calculations viewing the room as a square. 
    config = MapConfig(pumps=[pump], fake_pumps=[fake_pump], n_cells_in_area=math.round((8.20488**2)/(granularity**2))
    return config


###################################
### BASELINE MAP WITH TWO PUMPS ###
###################################
def get_baseline_two_pumps_config():
    pump = Pump(3.6, 4.2)
    pump2 = Pump(2.2, 4.4)
    config = MapConfig(pumps=[pump, pump2])
    return config
