function J = robot_jacobian(angles, lengths)
%ROBOT_JACOBIAN Return the 2-by-N Cartesian position Jacobian.

angles = angles(:).';
lengths = lengths(:).';
if numel(angles) ~= numel(lengths)
    error('robot_jacobian:DimensionMismatch', ...
        'Angles and link lengths must have the same number of elements.');
end

headings = cumsum(angles);
J = zeros(2, numel(angles));
for joint = 1:numel(angles)
    downstream = joint:numel(angles);
    J(1, joint) = -sum(lengths(downstream) .* sin(headings(downstream)));
    J(2, joint) = sum(lengths(downstream) .* cos(headings(downstream)));
end
end
