def int_to_bitblock(read):
    bits = bin(read)[2:].zfill(8)
    indices = []

    for idx, value in enumerate(bits):
        if value == '1':
            indices.append(idx + 1)

    return bits, indices

# def check_branching(bits):
