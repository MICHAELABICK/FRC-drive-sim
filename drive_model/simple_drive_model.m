trackwidth = 22; % inches
wheelbase = 18; % inches
x = trackwidth / 2;
y = wheelbase / 2;
fric_coef = 1.1;
mass = 50;

g = 9.8; % m/s^2

center = [0, 0];
wheel1 = 0.0254 * [x, y];
wheel2 = 0.0254 * [x, -y];
wheel3 = 0.0254 * [-x, y];
wheel4 = 0.0254 * [-x,-y];
wheels = [wheel1; wheel2; wheel3; wheel4];
[num_wheels, ~] = size(wheels);

is_left = wheels(:, 1) < 0;
left_wheels = wheels(is_left, :);
right_wheels = wheels(~is_left, :);

Fs_max = fric_coef * mass* g / num_wheels; % static friction force on each wheel, assuming weight spread evenly
