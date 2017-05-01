function [t, x, v, a] = lineSim(time, motor, numMotors, currLim, mass, fric, vi, dia, ratio)
% A function to simulate the maximum performance of an electic vehicle
% along a line. USES METRIC UNITS

eqMotor = motor;
eqMotor.freeCurr = numMotors .* eqMotor.freeCurr;
eqMotor.stallTorque = numMotors .* eqMotor.stallTorque;
eqMotor.stallCurr = numMotors .* eqMotor.stallCurr;

circum = 3.14159 .* dia;
fmax = fric .* mass .* 9.8;

t = linspace(0, time);
dt = t(2) - t(1);
x = 0;
v = vi;
f = [];
a = [];
w = [];
torque = [];

for i = 1:length(t)
  if i ~= 1
    v(i) = v(i - 1) + (dt .* a(i - 1));
    x(i) = x(i - 1) + (dt .* v(i - 1));
  end

  w(i) = v(i) .* ratio .* 60 ./ circum;
  torque(i) = getTorque(eqMotor, w(i), currLim);
  f(i) = torque(i) .* ratio .* 2 ./ dia;

  if f(i) > fmax
    f(i) = fmax;
  elseif f(i) < -fmax
    f(i) = -fmax;
  end

  a(i) = f(i) ./ mass;
end

end
