import math


def create_drivetrain(
    mass,
    motor,
    num_motors,
    gear_ratio,
    wheel_diameter,
    resistance_bat
):
    pass
    # combined_motor = Motor(
    #         num_motors * motor.torque_const,
    #         motor.back_emf_const,


class Drivetrain:
    g = 9.81  # m/s

    def __init__(
        self,
        mass,
        motor,
        gear_ratio,
        wheel_diameter,
        voltage_bat,
        resistance_bat,
        wheel_friction_coef=None,
        current_limit=None
    ):

        self._mass = mass
        self.motor = motor
        self.gear_ratio = gear_ratio
        self.wheel_radius = wheel_diameter
        self.voltage_bat = voltage_bat
        self.resistance_bat - resistance_bat
        self.current_limit = current_limit
        self._wheel_friction_coef = wheel_friction_coef

        self._update_slip_force()

    @property
    def mass(self):
        return self._mass

    @mass.setter
    def mass(self, mass):
        self._mass = mass
        self._update_slip_force()

    @property
    def wheel_friction_coef(self):
        return self._wheel_friction_coef

    @wheel_friction_coef.setter
    def wheel_friction_coef(self, wheel_friction_coef):
        self._wheel_friction_coef = wheel_friction_coef
        self._update_slip_force()

    def _update_slip_force(self):
        if self.wheel_friction_coef is not None:
            self._slip_force = self.mass * self.g * self.wheel_friction_coef
        else:
            self._slip_force = None

    def forward_ode(self, t, y):
        """
        Defines the system of ODEs for a forward moving drivetrain

        :param t: A scalar of the current time step
        :param y: A vector of the system variables
        :returns: A vector of the time derivatives of the system variables
        """

        (velocity, position) = y

        omega_motor = velocity / self.wheel_radius  # rad/s

        # TODO: include motor impedance effects
        motor_current = self._motor_current(omega_motor, 0)
        if self.current_limit is not None:
            motor_current = min((motor_current, self.current_limit))

        force = self._motor_current(omega_motor, 0) * \
            self.motor.torque_const * self.gear_ratio / self.wheel_radius
        if self._slip_force is not None:
            force = min((force, self._slip_force))

        return (force / self.mass, velocity)

    def _motor_current(self, omega_motor, motor_current_dot):
        return (self.voltage_bat
                - (self.motor.impedance * motor_current_dot)
                - (self.motor.back_emf_const * omega_motor)) \
                / (self.resistance_bat + self.motor.resistance)


class MotorFactory:
    motor_list = {
            'cim': {
                'voltage': 12,  # volts
                'free_speed': 5330,  # RPM
                'free_current': 2.7,  # amps
                'stall_torque': 2.41,  # Nm
                'stall_current': 131,  # amps
                'impedance': 0
            }
        }

    @classmethod
    def create(cls, motor_name):
        specs = cls.motor_list[motor_name]
        free_speed = specs['free_speed'] * 2 * math.pi / 60  # rad/s
        voltage = specs['voltage']
        stall_current = specs['stall_current']

        resistance = voltage / stall_current
        torque_const = specs['stall_torque'] / stall_current
        back_emf_const = (voltage - (resistance * specs['free_current'])) \
            / free_speed

        return Motor(
                torque_const, back_emf_const, resistance, specs['impedance'])


class Motor:
    def __init__(self, torque_const, back_emf_const, resistance, impedance):
        self.torque_const = torque_const
        self.back_emf_const = back_emf_const
        self.resistance = resistance
        self.impedance = impedance
