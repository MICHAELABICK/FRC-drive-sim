function w = calcRotVel(v, ratio, circum)

w = v .* ratio .* 60 ./ circum;

end
