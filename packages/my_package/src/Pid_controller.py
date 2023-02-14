# https://github.com/duckietown/mooc-exercises/blob/c476a67fea9836beab99eed23cfc1a83c77af7bb/modcon/solution/05-PID-Control/SOLUTION-PID_controller.ipynb
# plot drawing

class PID_Controller():
    def __init__(self, Kp, Ki, Kd, I, rospy_rate):
        self.Kp = Kp
        print("Kp is", self.Kp)
        self.Ki = Ki
        self.Kd = Kd
        self.I = I
        self.delta_t = rospy_rate

    def apply_controller(self, car, err, last_error):
        if err == 0:
            car.forward()
        else:
            P = err
            self.I = self.I + err
            self.I = max(min(self.I, 0.2), -0.2)  # 2, -2
            D = err - last_error
            PID = (self.Kp * P) + (self.Ki * self.I * self.delta_t) + \
                (self.Kd * D)
            print("PID:", PID)
            print("P * self.Kp is:", P * self.Kp)
            print("self.Ki * self.I * self.delta_t is:",
                  self.Ki * self.I * self.delta_t)
            #print("self.Kd * D / self.delta_t is:", self.Kd * D / self.delta_t)
            print("self.Kd * D is:", self.Kd * D)

            if car.velocity + PID >= 0:
                car.speed_right_wheel = car.velocity + PID
            else:
                car.speed_right_wheel = 0
            if car.velocity - PID >= 0:
                car.speed_left_wheel = car.velocity - PID
            else:
                car.speed_left_wheel = 0
