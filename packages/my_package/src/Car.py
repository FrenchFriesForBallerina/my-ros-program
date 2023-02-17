class Car():
    def __init__(self, velocity):
        self.velocity = velocity
        self.speed_right_wheel = velocity
        self.speed_left_wheel = velocity
        self.branching_off_first_detection = False
        self.branching_off_confirmed = False
        self.turn_at_next_left = False
        self.turn_at_next_right = False

    def turn_left(self):
        print('turn left')
        self.speed_left_wheel = 0.01 * self.velocity
        self.speed_right_wheel = self.velocity

    def turn_right(self):
        print('turn right')
        self.speed_right_wheel = 0.01 * self.velocity
        self.speed_left_wheel = self.velocity

    def sharp_left(self):
        print('sharp left')
        self.speed_left_wheel = 0
        self.speed_right_wheel = self.velocity * 0.8

    def sharp_right(self):
        print('sharp right')
        self.speed_right_wheel = 0
        self.speed_left_wheel = self.velocity * 0.8

    def forward(self):
        self.speed_left_wheel = self.velocity
        self.speed_right_wheel = self.speed_left_wheel
