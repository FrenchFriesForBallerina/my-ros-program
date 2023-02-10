class Car():
    def __init__(self, velocity):
        self.velocity = velocity
        self.speed_right_wheel = velocity
        self.speed_left_wheel = velocity

    def turn_left(self):
        print('turn left')
        self.speed_left_wheel = 0.01 * self.velocity
        self.speed_right_wheel = self.velocity

    def turn_right(self):
        print('turn right')
        self.speed_right_wheel = 0.01 * self.velocity
        self.speed_left_wheel = self.velocity

    def forward(self):
        print('go forward')
        self.speed_left_wheel = self.velocity
        self.speed_right_wheel = self.velocity
