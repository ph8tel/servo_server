class EscFilter:
    def __init__(self, accel_rate=300, decel_rate=800):
        # pwm units per second
        self.accel_rate = accel_rate
        self.decel_rate = decel_rate
        self.filtered = 1500

    def update(self, target, dt):
        neutral = 1500
        cur = self.filtered

        accelerating = (
            (target > cur and cur >= neutral) or
            (target < cur and cur <= neutral)
        )

        rate = self.accel_rate if accelerating else self.decel_rate
        max_step = rate * dt

        delta = target - cur
        if abs(delta) <= max_step:
            self.filtered = target
        else:
            self.filtered = cur + max_step * (1 if delta > 0 else -1)

        return self.filtered
