function tests = testRobotArmKinematics
tests = functiontests(localfunctions);
end

function testStraightArmForwardKinematics(testCase)
[tool, heading, points] = forward_kinematics([0 0 0], [220 220 220]);
verifyEqual(testCase, tool, [660 0], 'AbsTol', 1e-10);
verifyEqual(testCase, heading, 0);
verifyEqual(testCase, size(points), [2 4]);
end

function testJacobianMatchesFiniteDifferences(testCase)
angles = [-0.4 0.6 0.2];
lengths = [220 220 220];
J = robot_jacobian(angles, lengths);
step = 1e-6;
numerical = zeros(2, numel(angles));
for joint = 1:numel(angles)
    offset = zeros(size(angles));
    offset(joint) = step;
    plus = forward_kinematics(angles + offset, lengths);
    minus = forward_kinematics(angles - offset, lengths);
    numerical(:, joint) = ((plus - minus) / (2 * step)).';
end
verifyEqual(testCase, J, numerical, 'AbsTol', 1e-5);
end

function testInverseKinematicsReachesRepresentativeTarget(testCase)
result = inverse_kinematics([420 260], [-25 35 20] * pi / 180, ...
    [220 220 220]);
verifyTrue(testCase, result.reached);
verifyLessThan(testCase, result.error, 0.5);
end

function testQuinticBlendHasZeroEndpointVelocities(testCase)
verifyEqual(testCase, quintic_blend(0), 0);
verifyEqual(testCase, quintic_blend(1), 1);
verifyEqual(testCase, quintic_blend([-1 0 1 2]), [0 0 1 1]);
end

function testInterpolationReachesExactTarget(testCase)
verifyEqual(testCase, interpolate_point([10 20], [110 220], 1), ...
    [110 220], 'AbsTol', 1e-12);
end
