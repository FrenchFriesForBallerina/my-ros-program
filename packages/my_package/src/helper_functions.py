def int_to_bitblock(read):
    assert 0 <= read <= 255, "Value error while reading Sparkfun Line Follower Array"

    bits = bin(read)[2:].zfill(8)
    binary_values = (128, 64, 32, 16, 8, 4, 2, 1)
    indices = []
    temp = read

    for i in range(8):
        if temp - binary_values[i] >= 0:
            temp -= binary_values[i]
            indices.append(i + 1)
    return bits  # , indices
