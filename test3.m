clc;

close all

points = [0, 0; 5, 3; 13, 1; 14, 5];
x = points(:, 1);
y = points(:, 2);

[xq, yq, l, curv, B] = createPath(x, y, 100);

% set vi & vf
vi = 0;
vf = 0;

% Lets pretend we know some info about the vehicle
mass = 60;
fric = 1.1;
trackwidth = .7;

% Calculate the traction limit for each side,
% remembering that each side only holds half the weight
% of the vehicle
iFric = fric * mass * 9.8 / 2;
oFric = iFric;

dia = 4 * .0254;
circum = dia * 3.14159;
ratio = 22;
numMotors = 4; % Motors per side
currLim = 700; % Current limit for vehicle
mass = 60;

motor = createMotor(18700, 0.7, 0.71, 134);
eqMotor = motor;
eqMotor.freeCurr = numMotors .* eqMotor.freeCurr;
eqMotor.stallTorque = numMotors .* eqMotor.stallTorque;
eqMotor.stallCurr = numMotors .* eqMotor.stallCurr;

vmax = sqrt(fric * 9.8 ./ curv);
% limit vmax for the sake of graphing
vgraph = min(vmax, 10);

% Calculate the velocities for each side of the vehicle,
% then limit based on traction and current
vaccel = vi;
r = 1 ./ curv;
rAdd = trackwidth / 2;
% Use forward kinematics to calculate accel
for i = 1:length(r) - 1
  % Calculate the acceleration on the outside wheel,
  % as this will be the one moving faster and therefore
  % have a more limited force
  vi = (r(i) - rAdd) .* vaccel(i) ./ r(i);
  vo = (r(i) + rAdd) .* vaccel(i) ./ r(i);

  wi = calcRotVel(vi, ratio, circum);
  wo = calcRotVel(vo, ratio, circum);
  
  % Allocate more current to the side that needs to
  % generate more force
  oCurrLim = currLim .* (r(i) + rAdd) ./ (2 * r(i));
  iCurrLim = currLim - oCurrLim;
  
  ji = getTorque(eqMotor, wi, iCurrLim);
  jo = getTorque(eqMotor, wo, oCurrLim);
  
  % Seperate fric for eventual accounting of CG
  fi = getForce(ji, ratio, dia, iFric);
  fo = getForce(jo, ratio, dia, oFric);
  
  a(i) = curv(i) .* ((fi * (r(i) - rAdd)) + (fo * (r(i) + rAdd))) ./ mass;
  
  % dv(x) = a(x)dx
  dv = a(i) * (l(i + 1) - l(i));
  vaccel(i + 1) = vaccel(i) + dv;
  
  vaccel(i + 1) = min(vmax(i + 1), vaccel(i + 1));
  
  % Turn left AKA vr > vl
  %if B(3,i) > 0
  % Turn right AKA vl > vr
  %else
  %end
end

% Calculate the velocities for each side of the vehicle,
% then limit based on traction and current
vdecel(length(r)) = vf;
iter = -1;
% Use forward kinematics to calculate decel
for i = length(r):iter:2
  % Calculate the acceleration on the outside wheel,
  % as this will be the one moving faster and therefore
  % have a more limited force
  vi = (r(i) - rAdd) .* vdecel(i) ./ r(i);
  vo = (r(i) + rAdd) .* vdecel(i) ./ r(i);

  wi = calcRotVel(vi, ratio, circum);
  wo = calcRotVel(vo, ratio, circum);
  
  % Allocate more current to the side that needs to
  % generate more force
  oCurrLim = currLim .* (r(i) + rAdd) ./ (2 * r(i));
  iCurrLim = currLim - oCurrLim;
  
  ji = getTorque(eqMotor, wi, iCurrLim);
  jo = getTorque(eqMotor, wo, oCurrLim);
  
  % Seperate fric for eventual accounting of CG
  fi = getForce(ji, ratio, dia, iFric);
  fo = getForce(jo, ratio, dia, oFric);
  
  % FROM NOW ON DIFFERENT THAN DECEL
  a(i) = curv(i) .* ((fi * (r(i) - rAdd)) + (fo * (r(i) + rAdd))) ./ mass;
  
  % dv(x) = a(x)dx
  % TODO: figure out a better way of generalizing this than taking abs
  dv = a(i) * abs(l(i + iter) - l(i));
  vdecel(i + iter) = vdecel(i) + dv;
  
  vdecel(i + iter) = min(vmax(i + iter), vdecel(i + iter));
  
  % Turn left AKA vr > vl
  %if B(3,i) > 0
  % Turn right AKA vl > vr
  %else
  %end
end

figure
plot(xq, yq);
grid on
%pbaspect([1 1 1])
daspect([1 1 1])

figure;
%x = 0:1/100:length(points);
x = l(1:length(curv));
subplot(2, 1, 1);
plot(x, curv);

% Convert to feet
x = x .* 3.28;
vgraph = vgraph .* 3.28;
vaccel = vaccel .* 3.28;
vdecel = vdecel .* 3.28;
% Plot
subplot(2, 1, 2);
plot(x, vgraph);
hold on
plot(x, vaccel(1:length(x)));
hold off
hold on
plot(x, vdecel(1:length(x)));
hold off

% dt = dx/v(x)
v = min(vmax, vaccel);
%v = min(v, vdecel);
dt = diff(l) ./ v;
isInf = dt == inf;
dt(isInf) = 0;
t = cumsum(dt)

figure
subplot(2, 1, 1);
plot(t, x);

subplot(2, 1, 2);
plot(t, v);