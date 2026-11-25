function angle = wrap_angle(angle)
%WRAP_ANGLE Normalize angles in radians to [-pi, pi).

angle = mod(angle + pi, 2 * pi) - pi;
end
