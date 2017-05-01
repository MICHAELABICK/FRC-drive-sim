function f = getForce(torque, ratio, dia, fmax)

f = torque .* ratio .* 2 ./ dia;

if f > fmax
  f = fmax;
elseif f < -fmax
  f = -fmax;
end

end
