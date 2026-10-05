from rocket_sim.control import Command


class ConstController:
    def __init__(self, y: float = 0.0):
        self.y = y

    def reset(self, ctx):
        pass

    def update(self, inp):
        return Command(self.y, 0.0)
