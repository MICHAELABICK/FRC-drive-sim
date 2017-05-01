function torque = getTorque(motor, speed, currLim)
% A function to determine the max amoount of torque a given electric
% motor can produce given the current rpms and current limit
% ASSUMES FREE CURRENT = 0A

torque = motor.stallTorque .* (1 - (speed ./ motor.freeSpeed));

curr = motor.stallCurr .* (1 - (speed ./ motor.freeSpeed));

% Account for current limiting
% If at the current limit, scale torque proportionately
cAdj = currLim ./ curr;
torqueAdj = torque .* cAdj;

isLim = curr > currLim;
isNegLim = curr < -currLim;
torque(isLim) = torqueAdj(isLim);
torque(isNegLim) = torqueAdj(isNegLim);

end
