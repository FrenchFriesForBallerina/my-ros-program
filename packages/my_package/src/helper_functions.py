import time
import re


def int_to_bitblock(read):
    assert 0 <= read <= 255, "Value error while reading Sparkfun Line Follower Array"

    bits = bin(read)[2:].zfill(8)
    detect_atypical_road_conditions(bits)
    binary_values = (128, 64, 32, 16, 8, 4, 2, 1)
    indices = []
    temp = read

    for i in range(8):
        if temp - binary_values[i] >= 0:
            temp -= binary_values[i]
            indices.append(i + 1)
    return bits, indices


def timer():
    def __init__(self):
        self.start = 0
        self.end = 0

    def start(self):
        start = time.time()
        return start

    def stop(self):
        end = time.time()
        return end

    def time_elapsed(self):
        print(self.end - self.start)
        return (self.end - self.start)


def detect_atypical_road_conditions(bits):
    m = re.search('^0+((?:1{2})|(?:1{1}))0+$', bits)
    if not m:
        print('################## ATYPICAL ROAD CONDITIONS #####################')
    else:
        print('...')
        pass
