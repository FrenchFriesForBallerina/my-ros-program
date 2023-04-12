from helper_functions import int_to_bitblock, timer
import statistics


def cruise_control(error, last_error, read, target, pid_controller, car):
    bits_block, indices = int_to_bitblock(read)

    if branching_off_ahead(bits_block):
        print('BRANCHING -------------------------------------------------------')
        car.branching_off_first_detection = True
        car.turn_at_next_left = True
        return

    if len(indices) != 0:
        last_error = error
        error = target - statistics.mean(indices)
        pid_controller.apply_controller(car, error, last_error)

    else:
        car.speed_left_wheel = 0
        car.speed_right_wheel = 0


def branching_off_ahead(binary):
    print('binary is ', binary)
    left_turn = ['00110110', '00110100', '01100010', '01100100', '01101100',
                 '11100110', '11101100', '11000100', '11001100', '11011000']
    if binary in left_turn:
        print('I SEE ROADSIGN')
        print('im AWAKE')
        print("I WILL STOP")
        return True
