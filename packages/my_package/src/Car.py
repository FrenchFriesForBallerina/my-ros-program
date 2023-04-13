class Car():
    def __init__(self, velocity):
        self.velocity = velocity
        self.speed_right_wheel = velocity
        self.speed_left_wheel = velocity
        self.branching_off_first_detection = False
        self.branching_off_confirmed = False
        self.turn_at_next_left = False
        self.turn_at_next_right = False

    def forward(self):
        self.speed_left_wheel = self.velocity
        self.speed_right_wheel = self.speed_left_wheel
