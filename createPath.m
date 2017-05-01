function [xq, yq, l, curv, B] = createPath(x, y, steps)
% A function to return a spline path given waypoint coordinates and
% the amount of steps wanted between waypoints

% define waypoints
t = 0:length(x) - 1;

% calculate spline for way points
dt = 1 / steps;
tq = 0:dt:length(x) - 1;
xq = interp1(t,x,tq,'spline');
yq = interp1(t,y,tq,'spline');

vx = diff(xq) ./ dt;
vy = diff(yq) ./ dt;
s = sqrt((vx .^ 2) + (vy .^2));
l = cumtrapz(tq(1:length(s)), s);

% Find the unit tangent vector
Tx = vx ./ s;
Ty = vy ./ s;

dTx = diff(Tx) ./ dt;
dTy = diff(Ty) ./ dt;
magdT = sqrt((dTx .^ 2) + (dTy .^ 2));

% Find the normal vector
Nx = dTx ./ magdT;
Ny = dTy ./ magdT;

% Find the curvature
curv = magdT ./ s(1:end - 1);

% Find the binomial vector
T = [Tx; Ty; zeros(1, length(Tx))];
N = [Nx; Ny; zeros(1, length(Nx))];
B = cross(T(:, 1:length(Nx)), N);
