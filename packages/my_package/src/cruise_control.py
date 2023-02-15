from helper_functions import int_to_bitblock
from statistics import mean
from Pid_controller import PID_Controller


def cruise_control(error, last_error, read, target, pid_controller, car):
    bits_block, indices = int_to_bitblock(read)
    if len(indices) != 0:
        last_error = error
        error = target - mean(indices)
        print('bits_block is', bits_block)
        pid_controller.apply_controller(car, error, last_error)

    else:
        car.speed_left_wheel = 0
        car.speed_right_wheel = 0
