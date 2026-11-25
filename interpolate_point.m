function point = interpolate_point(start_point, end_point, fraction)
%INTERPOLATE_POINT Quintic interpolation between Cartesian points.

start_point = start_point(:).';
end_point = end_point(:).';
if numel(start_point) ~= 2 || numel(end_point) ~= 2
    error('interpolate_point:InvalidPoint', 'Points must be two-element [x y] vectors.');
end
blend = quintic_blend(fraction);
point = start_point + (end_point - start_point) .* blend;
end
