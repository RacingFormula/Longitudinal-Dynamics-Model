import numpy as np
import matplotlib.pyplot as plt

class LongitudinalDynamicsModel:
    def __init__(self, mass, drag_coefficient, frontal_area, rolling_resistance, engine_force, brake_force):
        self.mass = mass
        self.drag_coefficient = drag_coefficient
        self.frontal_area = frontal_area
        self.rolling_resistance = rolling_resistance
        self.engine_force = engine_force
        self.brake_force = brake_force

    def calculate_drag_force(self, velocity):
        air_density = 1.225  # kg/m^3 (standard at sea level)
        return 0.5 * self.drag_coefficient * self.frontal_area * air_density * velocity**2

    def calculate_rolling_resistance_force(self):
        return self.rolling_resistance * self.mass * 9.81  # g = 9.81 m/s^2

    def simulate_motion(self, time_steps, initial_velocity, throttle_input, brake_input):
        velocities = [initial_velocity]

        for i in range(1, len(time_steps)):
            dt = time_steps[i] - time_steps[i - 1]
            velocity = velocities[-1]

            # Calculate forces
            engine_force = self.engine_force * throttle_input[i]
            braking_force = self.brake_force * brake_input[i]
            drag_force = self.calculate_drag_force(velocity)
            rolling_resistance_force = self.calculate_rolling_resistance_force()

            # Net force
            net_force = engine_force - braking_force - drag_force - rolling_resistance_force

            # Acceleration (F = ma)
            acceleration = net_force / self.mass

            # Update velocity
            new_velocity = max(velocity + acceleration * dt, 0)  # Ensure velocity doesn't go negative
            velocities.append(new_velocity)

        return np.array(velocities)

if __name__ == "__main__":
    # Define vehicle parameters
    model = LongitudinalDynamicsModel(
        mass=1500,  # kg
        drag_coefficient=0.32,  # dimensionless
        frontal_area=2.2,  # m^2
        rolling_resistance=0.015,  # dimensionless
        engine_force=8000,  # N
        brake_force=10000,  # N
    )

    # Simulation parameters
    total_time = 20  # seconds
    time_steps = np.linspace(0, total_time, 201)  # 201 time steps
    initial_velocity = 0  # m/s

    # Define throttle and brake inputs
    throttle_input = np.clip(np.linspace(1, 0, len(time_steps)), 0, 1)  # Linearly decrease throttle
    brake_input = np.clip(np.linspace(0, 1, len(time_steps)), 0, 1)  # Linearly increase braking

    # Simulate motion
    velocities = model.simulate_motion(time_steps, initial_velocity, throttle_input, brake_input)

    # Plot results
    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, velocities, label="Velocity (m/s)", linewidth=2)
    plt.title("Longitudinal Dynamics Simulation", fontsize=14)
    plt.xlabel("Time (s)", fontsize=12)
    plt.ylabel("Velocity (m/s)", fontsize=12)
    plt.grid(True)
    plt.legend()
    plt.show()
