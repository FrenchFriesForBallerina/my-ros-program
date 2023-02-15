import re

from helper_functions import int_to_bitblock
from statistics import mean
import time


def cruise_control(error, last_error, read, target, pid_controller, car):
    bits_block, indices = int_to_bitblock(read)
    #print('bits_block is', bits_block)
    branching_off_ahead(bits_block)

    if len(indices) != 0:
        last_error = error
        error = target - mean(indices)
        pid_controller.apply_controller(car, error, last_error)

    else:
        car.speed_left_wheel = 0
        car.speed_right_wheel = 0


def branching_off_ahead(binary):  # '00110011'
    roadsign_first_detection
    roadsign_confirmed

    m = re.search('^1+1?0+0?1+1?0+$', binary)
    if m:
        if roadsign_first_detection == False:
            roadsign_first_detection = True
            print('match:', m[0])
        else:
            roadsign_confirmed = True

    else:
        print('nope')

    # reads two zeros between ones

    # if branching off ahead, remember what side the sign was on
    # to define, what side it was on, see which line continues and which doesn't
    # the one that doesn't is the direction of the turn ahead
    # remember the side to turn to - global variable like a switch True

    # need another function that determines if branching is detected

    # if branching is detected, turn to the according side (left/right)
    # after turning, switch the variable back to False
