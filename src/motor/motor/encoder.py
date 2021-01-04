from gpiozero import DigitalInputDevice


class Encoder:

    def __init__(self, node, a, b, precision, read_interval=0.05, reverse=False):
        self.a = DigitalInputDevice(a, pull_up=None, active_state=True)
        self.b = DigitalInputDevice(b, pull_up=None, active_state=True)

        self.a.pin.edges = 'falling'
        self.a.pin.when_changed = self._on_falling_edge

        self.read_interval = read_interval
        self.reverse = -1 if reverse else 1
        self.precision = precision
        self.timer = node.create_timer(read_interval, self._read_vel)

        self.pulses = 0
        self.read_rpm = 0

    def close(self):
        self.a.pin.when_changed = None

    def _on_falling_edge(self, ticks, state):
        if self.b.value == 1:
            self.pulses += 1
        else:
            self.pulses -= 1

    def _read_vel(self):
        self.read_rpm = self.reverse * (self.pulses * 60 / (self.precision * self.read_interval))
        self.pulses = 0
