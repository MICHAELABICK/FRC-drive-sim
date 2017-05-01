clc

close

motor = createMotor(18700, 0.7, 0.71, 134);

time = 3;
vi = 0;

dia1 = 4 * .0254;
ratio1 = 16;
numMotors1 = 4;
currLim1 = 1500;
mass1 = 60;

dia2 = 4 * .0254;
ratio2 = 12;
numMotors2 = 8;
currLim2 = 500;
mass2 = 60;

[t1, x1, v1, a1] = lineSim(time, motor, numMotors1, currLim1, mass1, 1.1, vi, dia1, ratio1);
[t2, x2, v2, a2] = lineSim(time, motor, numMotors2, currLim2, mass2, 1.1, vi, dia2, ratio2);

% Convert to feet
x1 = x1 .* 3.28;
x2 = x2 .* 3.28;
v1 = v1 .* 3.28;
v2 = v2 .* 3.28;

subplot(3, 1, 1);
plot(t1, x1);
hold on;
plot(t2, x2);
hold off

subplot(3, 1, 2);
plot(t1, v1);
hold on;
plot(t2, v2);
hold off

subplot(3, 1, 3);
plot(t1, a1);
hold on;
plot(t2, a2);
hold off