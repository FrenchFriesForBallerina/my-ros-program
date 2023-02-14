def int_to_bitblock(read):
    bits_block = bin(read)[2:]
    leading_zeros = 8 - len(bits_block)
    bits = leading_zeros*'0' + bits_block
    indices = []

    for idx, value in enumerate(bits):
        if value == '1':
            indices.append(idx + 1)

    return bits, indices
