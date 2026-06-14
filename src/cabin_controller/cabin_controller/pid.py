import time

class PIDController:
    def __init__(
        self,
        kp=0.0, 
        ki=0.0, 
        kd=0.0,
        max_out=100.0, 
        min_out=-100.0,
        max_i=50.0, 
        min_i=-50.0,
        error_tolerance=0.0
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.max_out = max_out
        self.min_out = min_out
        
        self.max_i = max_i
        self.min_i = min_i
        
        self.error_tolerance = error_tolerance

        self.last_error = 0.0
        self.integral = 0.0
        self.target = 0.0
        self.last_time = 0.0

    def set_target(self, target):
        # 目标航向角约束到0~360区间，防止外部传入超范围角度
        target = target % 360.0
        self.target = target

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = 0.0

    def compute(self, current, dt=None):
        now = time.time()
        
        if dt is None:
            if self.last_time == 0:
                self.last_time = now
                return 0.0
            dt = now - self.last_time

        dt = max(dt, 1e-6)
        # 1. 当前角度先取模约束至0~360，避免传感器输出负数/超360
        current_angle = current % 360.0
        # 2. 原始线性误差
        raw_error = self.target - current_angle
        # 3. 将误差映射到 [-180, 180]，实现最短转向，不会绕360度大圈
        error = raw_error
        if error > 180.0:
            error -= 360.0
        elif error < -180.0:
            error += 360.0

        if abs(error) < self.error_tolerance:
            self.reset()
            return 0.0

        p_term = self.kp * error

        # 角度跳变时冻结积分，防止积分饱和炸输出
        # 误差绝对值接近180度说明需要大掉头，此时停止积分累加
        if abs(error) < 175.0:
            self.integral += error * dt
        self.integral = max(min(self.integral, self.max_i), self.min_i)
        i_term = self.ki * self.integral

        d_term = self.kd * (error - self.last_error) / dt
        output = p_term + i_term + d_term
        output = max(min(output, self.max_out), self.min_out)

        self.last_error = error
        self.last_time = now

        return output
