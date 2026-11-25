function [tool, heading, points] = forward_kinematics(angles, lengths)
%FORWARD_KINEMATICS Calculate link endpoints for a planar serial arm.
%   ANGLES are relative joint angles in radians; LENGTHS are in millimeters.

angles = angles(:).';
lengths = lengths(:).';
if numel(angles) ~= numel(lengths)
    error('forward_kinematics:DimensionMismatch', ...
        'Angles and link lengths must have the same number of elements.');
end

joint_headings = cumsum(angles);
heading = sum(angles);
increments = [lengths .* cos(joint_headings); lengths .* sin(joint_headings)];
points = [[0; 0], cumsum(increments, 2)];
tool = points(:, end).';
end
