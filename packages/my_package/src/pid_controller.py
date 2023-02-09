""" def apply_controller(err, last_error, rospy_rate, vehicle_speed, Kp, Ki, Kd):
    global I
    P = Kp * err
    I = I + (rospy_rate * err * Ki)
    D = Kd * ((err - last_error)/rospy_rate)
    PID = P + I + D
    right_wheel_speed = vehicle_speed + PID
    left_wheel_speed = vehicle_speed - PID
    return right_wheel_speed, left_wheel_speed  """