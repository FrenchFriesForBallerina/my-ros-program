import time
import re


def int_to_bitblock(read):
    assert 0 <= read <= 255, "Value error while reading Sparkfun Line Follower Array"
    # detect_atypical_road_conditions(read)
    bits = bin(read)[2:].zfill(8)
    binary_values = (128, 64, 32, 16, 8, 4, 2, 1)
    indices = []
    temp = read

    for i in range(8):
        if temp - binary_values[i] >= 0:
            temp -= binary_values[i]
            indices.append(i + 1)
    return bits, indices


class Weird_timer():
    def __init__(self):
        self.initial = 0
        self.final = 0

    def start(self):
        self.initial = time.time()
        print('start')

    def stop(self):
        self.final = time.time()
        print('stop')

    def result(self):
        print(self.initial)
        print(self.final)
        res = self.final - self.initial
        print('result', round(res, 2))
        return (round(self.final - self.initial, 2))

    def reset(self):
        self.initial = 0
        self.final = 0
        print('reset')


def detect_atypical_road_conditions(read):
    bits = bin(read)[2:].zfill(8)
    m = re.search('^0+((?:1{2})|(?:1{1}))0+$', bits)
    if not m:
        return True
    else:
        return False
