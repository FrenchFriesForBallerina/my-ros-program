# https://github.com/duckietown/mooc-exercises/blob/c476a67fea9836beab99eed23cfc1a83c77af7bb/modcon/solution/05-PID-Control/SOLUTION-PID_controller.ipynb
# plot drawing

class PID_Controller():
    def __init__(self, Kp, Ki, Kd, I, rospy_rate):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.I = I
        self.delta_t = rospy_rate

    def apply_controller(self, car, err, last_error):
        P = self.Kp * err
        self.I = self.I + (self.delta_t * err * self.Ki)
        self.I = max(min(self.I, 0.2), -0.2)  # 2, -2
        D = self.Kd * ((err - last_error)/self.delta_t)
        PID = P + self.I + D
        car.speed_right_wheel = car.velocity + PID
        car.speed_left_wheel = car.velocity - PID
        # return car.speed_right_wheel, car.speed_left_wheel
