# pybulletSIM.py
import pybullet as p
import pybullet_data
import numpy as np
from load_rocket import RocketLoader
from cfg import Simcfg, RocketCFG
import roket


class PyBulletSimulation:
    def __init__(self, gui=True):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        self.rocket_id = None
        self.time = 0
        self.history = {'time': [], 'height': [], 'velocity': [], 'thrust': []}
        roket.reset_state()

    def setup_environment(self, urdf_path="rocket.urdf"):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(*Simcfg.gravity)
        p.loadURDF("plane.urdf", useFixedBase=True)

        # НАСТРОЙКА КАМЕРЫ - ПРИБЛИЖЕННАЯ
        p.resetDebugVisualizerCamera(
            cameraDistance=Simcfg.camera_distance,  # 80 метров
            cameraYaw=Simcfg.camera_yaw,  # 0° - вид прямо
            cameraPitch=Simcfg.camera_pitch,  # 0° - горизонтально
            cameraTargetPosition=Simcfg.camera_target  # [0, 0, 150]
        )

        # Включаем визуализацию
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

        # Добавляем подсветку
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 0)

        # Маркеры высот для ориентации
        for h in [300, 250, 200, 150, 100, 50, 20, 10, 5]:
            p.addUserDebugText(f"{h}м", [0, 0, h], [0.7, 0.7, 0.7], 0.8)

        # Яркий маркер места посадки
        p.addUserDebugText("🏁 ПОСАДКА", [0, 0, 0.5], [1, 1, 0], 1.5)
        p.addUserDebugLine([-3, 0, 0], [3, 0, 0], [1, 0, 0], 3)
        p.addUserDebugLine([0, -3, 0], [0, 3, 0], [1, 0, 0], 3)

        # Добавляем круг посадки
        for angle in np.linspace(0, 2 * np.pi, 36):
            x = 2 * np.cos(angle)
            y = 2 * np.sin(angle)
            p.addUserDebugLine([x, y, 0.01], [2 * np.cos(angle + 0.17), 2 * np.sin(angle + 0.17), 0.01], [0, 1, 0], 1)

        # Загрузка ракеты
        loader = RocketLoader(self.client)
        self.rocket_id = loader.load(urdf_path)

        if self.rocket_id is None:
            raise RuntimeError("Не удалось загрузить ракету")

        print(f"\n✅ СИМУЛЯЦИЯ ГОТОВА")
        print(f"   Камера: дистанция={Simcfg.camera_distance}м, вид сбоку")

    def step(self):
        if self.rocket_id is None:
            return

        pos, _ = p.getBasePositionAndOrientation(self.rocket_id)
        vel, _ = p.getBaseVelocity(self.rocket_id)

        height = pos[2]
        velocity = vel[2]

        # Стабилизация каждые 10 шагов
        if int(self.time * 240) % 10 == 0 and height > 0.5:
            p.resetBasePositionAndOrientation(self.rocket_id, pos, [0, 0, 0, 1])
            p.resetBaseVelocity(self.rocket_id, [0, 0, vel[2]], [0, 0, 0])

        # Расчет сил
        mass = roket.get_current_mass()
        force = roket.total_force(self.time, vel, mass, height)

        # Применение силы
        p.applyExternalForce(self.rocket_id, -1, force, [0, 0, 0], p.LINK_FRAME)

        # История
        self.history['time'].append(self.time)
        self.history['height'].append(height)
        self.history['velocity'].append(velocity)
        self.history['thrust'].append(force[2])

        p.stepSimulation()
        self.time += Simcfg.step

    def run_simulation(self, duration=None):
        if duration is None:
            duration = Simcfg.duration

        steps = int(duration / Simcfg.step)

        print("\n" + "=" * 60)
        print("🚀 ПОСАДКА РАКЕТЫ")
        print("=" * 60)
        print(f"{'Время':6} | {'Высота':8} | {'Скорость':9} | {'Тяга':8} | {'Топливо':6}")
        print("-" * 55)

        for i in range(steps):
            self.step()

            if i % 120 == 0 and len(self.history['time']) > 0:
                fuel = roket.get_fuel()
                print(f"{self.time:5.1f} | {self.history['height'][-1]:8.1f} | "
                      f"{self.history['velocity'][-1]:8.2f} | {self.history['thrust'][-1]:7.0f} | {fuel:6.1f}")

            if len(self.history['height']) > 0 and self.history['height'][-1] <= 0.3:
                speed = abs(self.history['velocity'][-1])
                print("-" * 55)
                print(f"\n🪂 ПОСАДКА!")
                print(f"   Скорость касания: {speed:.2f} м/с")
                if speed < 2:
                    print("   ✅ ИДЕАЛЬНАЯ МЯГКАЯ ПОСАДКА!")
                elif speed < 5:
                    print("   ✅ ХОРОШАЯ ПОСАДКА")
                else:
                    print("   ⚠️ ЖЕСТКАЯ ПОСАДКА")
                break

        return self.history

    def dispose(self):
        try:
            p.disconnect()
        except:
            pass
