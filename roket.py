import numpy as np
from cfg import RocketCFG, Simcfg


class RocketPhysics:
    def __init__(self):
        self.current_mass = RocketCFG.mass_full
        self.fuel_burned = 0.0
        self.prev_thrust = 0.0

    def calculate_thrust(self, t: float, height: float, velocity: float) -> np.ndarray:

        g = abs(Simcfg.gravity[2])
        m = self.current_mass
        mdot = RocketCFG.fuel_consumption_rate


        # плавное уменьшение скорости к земле
        if height > 100:
            v_target = -50
        elif height > 50:
            v_target = -25
        elif height > 20:
            v_target = -10
        elif height > 10:
            v_target = -5
        else:
            v_target = -2

        # PD-регулятор пропорционально-дифференцирующий
        Kp = 1.6
        Kd = 0.9

        error_v = v_target - velocity
        
        # предсказание торможения
        a_req = Kp * error_v - Kd * velocity + g

        # ограничиваем адекватно
        a_req = np.clip(a_req, -18.0, 18.0)

        thrust = m * (g + a_req)

        # учёт остатка топлива 
        fuel_left = RocketCFG.mass_fuel - self.fuel_burned

        if fuel_left <= 0:
            thrust = 0.0
        else:
            fuel_ratio = fuel_left / RocketCFG.mass_fuel
            thrust *= fuel_ratio

        # мягкое удержание у земли
        if height < 10:
            a_req *= 1.05


        # финальный дожим энергии падения
        if height < 8:
            required_decel = (velocity ** 2) / (2 * max(height, 0.1))
            thrust = max(thrust, m * (g + required_decel))
        
        
        
        # ОГРАНИЧЕНИЯ
        
        thrust = max(0, min(thrust, RocketCFG.max_thrust))

        # перед землёй
        if height < 15 and velocity < -3:
            thrust = RocketCFG.max_thrust * 0.8

        
        # РАСХОД ТОПЛИВА
        
        # расход пропорционален тяге
        throttle = thrust / RocketCFG.max_thrust
        fuel_used = mdot * throttle * Simcfg.step

        self.fuel_burned += fuel_used

        self.current_mass = max(
            RocketCFG.mass_empty,
            RocketCFG.mass_full - self.fuel_burned
        )

        print(f"[{t:.2f}s] h={height:.1f} | v={velocity:.2f} | T={thrust:.0f}")

       
        # СГЛАЖИВАНИЕ
        
        alpha = 0.5
        
        if height < 20:
            alpha = 0.6
        else:
            alpha = 0.35

        thrust = alpha * thrust + (1 - alpha) * self.prev_thrust
        self.prev_thrust = thrust

        return np.array([0, 0, thrust])

    def calculate_drag(self, velocity: np.ndarray, height: float) -> np.ndarray:

        v = velocity
        speed = np.linalg.norm(v)

        if speed < 0.01:
            return np.zeros(3)

        rho = RocketCFG.air_density * np.exp(-height / 8000.0)

        # аэродинамическое сопротивление
        drag_direction = -v / speed  # против скорости

        drag_magnitude = (
            0.5
            * rho
            * RocketCFG.drag_coefficient
            * RocketCFG.effective_area
            * speed ** 2
        )

        drag = drag_direction * drag_magnitude

        return drag

    def calculate_gravity(self) -> np.ndarray:
        """Расчет силы тяжести"""
        return np.array([0, 0, self.current_mass * Simcfg.gravity[2]])

    def total_force(self, t: float, position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """Суммарная сила, действующая на ракету"""
        height = position[2]

        thrust = self.calculate_thrust(t, height, velocity[2])
        drag = self.calculate_drag(velocity, height)
        gravity = self.calculate_gravity()

        return thrust + drag + gravity

    def get_current_mass(self) -> float:
        return self.current_mass