function result = inverse_kinematics(target, start_angles, lengths, options)
%INVERSE_KINEMATICS Solve a planar arm target with damped least squares.
%   TARGET is [x y] in millimeters. Joint angles are returned in radians.

if nargin < 4
    options = struct();
end
max_iterations = option_value(options, 'maxIterations', 240);
tolerance = option_value(options, 'tolerance', 0.5);
damping = option_value(options, 'damping', 0.001);
angles = wrap_angle(start_angles(:).');
lengths = lengths(:).';
target = target(:).';

if numel(target) ~= 2
    error('inverse_kinematics:InvalidTarget', 'Target must be a two-element [x y] vector.');
end
if numel(angles) ~= numel(lengths)
    error('inverse_kinematics:DimensionMismatch', ...
        'Start angles and link lengths must have the same number of elements.');
end
if max_iterations < 1 || max_iterations ~= floor(max_iterations) || tolerance <= 0 || damping < 0
    error('inverse_kinematics:InvalidOptions', ...
        'maxIterations must be a positive integer; tolerance must be positive and damping nonnegative.');
end

reached = false;
iterations = 0;
for iteration = 1:max_iterations
    tool = forward_kinematics(angles, lengths);
    error_vector = target - tool;
    error_mm = norm(error_vector);
    if error_mm < tolerance
        reached = true;
        iterations = iteration - 1;
        break;
    end

    J = robot_jacobian(angles, lengths);
    lambda = (J * J.' + damping * eye(2)) \ error_vector.';
    angles = wrap_angle(angles + (J.' * lambda).');
    limit = 170 * pi / 180;
    angles = max(-limit, min(limit, angles));
    iterations = iteration;
end

tool = forward_kinematics(angles, lengths);
error_mm = norm(target - tool);
result = struct('angles', angles, 'error', error_mm, ...
    'iterations', iterations, 'reached', reached || error_mm < tolerance);
end

function value = option_value(options, name, default_value)
if isfield(options, name)
    value = options.(name);
else
    value = default_value;
end
end
