from cruise_control import *


def test1():
    print("Sanity check passed" if 1 == 1 else "Failed")


def test_branch_detection(read):
    print("branch case 1 detected OK" if branching_off_ahead(
        read) == True else "Failed")


def tests():
    test1()
