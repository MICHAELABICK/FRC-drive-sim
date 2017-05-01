clc

motor = createMotor(18700, 0.7, 0.71, 134);

x = linspace(0, 18700);
y = getTorque(motor, x, 100)

plot(x, y);