function robot_arm_kinematics_control
%ROBOT_ARM_KINEMATICS_CONTROL Launch the interactive 3-link arm simulation.

lengths = [220 220 220];
joints = [-25 35 20] * pi / 180;
velocities = zeros(1, 3);
initial_target = [420 260];

fig = figure('Name', '3-Link Robot Arm Kinematics & Control', ...
    'NumberTitle', 'off', 'Color', [0.035 0.067 0.11], ...
    'Position', [100 100 1120 700], 'MenuBar', 'none', ...
    'CloseRequestFcn', @close_figure);
ax = axes('Parent', fig, 'Units', 'normalized', ...
    'Position', [0.055 0.12 0.59 0.8], 'Color', [0.035 0.067 0.11], ...
    'XColor', [0.72 0.82 0.88], 'YColor', [0.72 0.82 0.88]);
hold(ax, 'on');
axis(ax, 'equal');
xlim(ax, [-500 500]);
ylim(ax, [-130 620]);
grid(ax, 'on');
ax.GridColor = [0.3 0.42 0.5];
ax.GridAlpha = 0.25;
xlabel(ax, 'X (mm)', 'Color', [0.85 0.91 0.95]);
ylabel(ax, 'Y (mm)', 'Color', [0.85 0.91 0.95]);
title(ax, 'Planar 3R Robot Arm', 'Color', [0.88 0.95 0.98]);

theta = linspace(0, pi, 180);
for radius = lengths(1):lengths(1):sum(lengths)
    plot(ax, radius .* cos(theta), radius .* sin(theta), ...
        'Color', [0.18 0.28 0.35], 'LineStyle', ':');
end
plot(ax, [-500 500], [0 0], 'Color', [0.28 0.4 0.48]);
arm_line = plot(ax, NaN, NaN, '-', 'Color', [0.35 0.84 0.9], 'LineWidth', 7);
joints_plot = plot(ax, NaN, NaN, 'o', 'MarkerSize', 10, ...
    'MarkerFaceColor', [0.04 0.13 0.2], 'MarkerEdgeColor', [0.65 0.9 0.95]);
tip_plot = plot(ax, NaN, NaN, 'o', 'MarkerSize', 9, ...
    'MarkerFaceColor', [0.35 0.84 0.9], 'MarkerEdgeColor', 'w');
target_plot = plot(ax, initial_target(1), initial_target(2), 'o', ...
    'MarkerSize', 11, 'MarkerFaceColor', [1 0.62 0.26], ...
    'MarkerEdgeColor', 'w');
path_plot = plot(ax, NaN, NaN, '--', 'Color', [0.68 0.76 0.8], 'LineWidth', 1.4);

status = make_label('Simulation ready', [0.69 0.88 0.25 0.05], 11, true);
metric = make_label(sprintf('Tip error: -- mm\nIK solution: --'), ...
    [0.69 0.78 0.28 0.08], 10, false);
make_label('Target X (mm)', [0.69 0.72 0.25 0.035], 10, false);
x_slider = make_slider(-430, 430, initial_target(1), [0.69 0.68 0.27 0.04]);
make_label('Target Y (mm)', [0.69 0.62 0.25 0.035], 10, false);
y_slider = make_slider(-100, 520, initial_target(2), [0.69 0.58 0.27 0.04]);
make_label('Trajectory duration (s)', [0.69 0.52 0.25 0.035], 10, false);
duration_slider = make_slider(1, 8, 3, [0.69 0.48 0.27 0.04]);
make_label('Kp', [0.69 0.42 0.25 0.035], 10, false);
kp_slider = make_slider(2, 35, 18, [0.69 0.38 0.27 0.04]);
make_label('Kd', [0.69 0.32 0.25 0.035], 10, false);
kd_slider = make_slider(0, 8, 2.2, [0.69 0.28 0.27 0.04]);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Plan & Run', ...
    'Units', 'normalized', 'Position', [0.69 0.17 0.13 0.065], ...
    'FontSize', 11, 'FontWeight', 'bold', 'Callback', @run_plan);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Stop', ...
    'Units', 'normalized', 'Position', [0.84 0.17 0.12 0.065], ...
    'FontSize', 11, 'Callback', @stop_plan);
setappdata(fig, 'StopRequested', false);
setappdata(fig, 'IKStatus', '--');
draw_arm();

    function handle = make_label(text, position, font_size, bold)
        handle = uicontrol(fig, 'Style', 'text', 'String', text, ...
            'Units', 'normalized', 'Position', position, ...
            'BackgroundColor', get(fig, 'Color'), ...
            'ForegroundColor', [0.85 0.91 0.95], ...
            'HorizontalAlignment', 'left', 'FontSize', font_size);
        if bold
            handle.FontWeight = 'bold';
        end
    end

    function handle = make_slider(minimum, maximum, value, position)
        handle = uicontrol(fig, 'Style', 'slider', 'Min', minimum, ...
            'Max', maximum, 'Value', value, 'Units', 'normalized', ...
            'Position', position, 'Callback', @update_target);
    end

    function update_target(~, ~)
        target_plot.XData = x_slider.Value;
        target_plot.YData = y_slider.Value;
        drawnow limitrate;
    end

    function draw_arm()
        [~, ~, points] = forward_kinematics(joints, lengths);
        arm_line.XData = points(1, :);
        arm_line.YData = points(2, :);
        joints_plot.XData = points(1, 1:end-1);
        joints_plot.YData = points(2, 1:end-1);
        tip_plot.XData = points(1, end);
        tip_plot.YData = points(2, end);
        current_target = [x_slider.Value y_slider.Value];
        target_plot.XData = current_target(1);
        target_plot.YData = current_target(2);
        tip_error = norm(current_target - points(:, end).');
        metric.String = sprintf('Tip error: %.1f mm\nIK solution: %s', ...
            tip_error, getappdata(fig, 'IKStatus'));
        drawnow limitrate;
    end

    function run_plan(~, ~)
        if ~isgraphics(fig)
            return;
        end
        start = forward_kinematics(joints, lengths);
        start_angles = joints;
        target = [x_slider.Value y_slider.Value];
        duration = duration_slider.Value;
        ik = inverse_kinematics(target, joints, lengths, ...
            struct('tolerance', 1.2, 'damping', 0.0008));
        target_angles = ik.angles;
        if ik.reached
            ik_status = sprintf('Reached (%d iterations)', ik.iterations);
        else
            ik_status = sprintf('Unreached (%.1f mm)', ik.error);
        end
        setappdata(fig, 'IKStatus', ik_status);
        status.String = 'Trajectory active';
        path_fraction = linspace(0, 1, 81);
        path_points = zeros(2, numel(path_fraction));
        for index = 1:numel(path_fraction)
            path_points(:, index) = interpolate_point(start, target, path_fraction(index)).';
        end
        path_plot.XData = path_points(1, :);
        path_plot.YData = path_points(2, :);
        setappdata(fig, 'StopRequested', false);

        dt = 0.02;
        steps = max(1, ceil(duration / dt));
        for index = 0:steps
            if ~isgraphics(fig) || getappdata(fig, 'StopRequested')
                break;
            end
            fraction = min(index / steps, 1);
            blend = quintic_blend(fraction);
            kp = kp_slider.Value;
            kd = kd_slider.Value;
            for joint = 1:numel(joints)
                desired = wrap_angle(start_angles(joint) + ...
                    wrap_angle(target_angles(joint) - start_angles(joint)) * blend);
                error_angle = wrap_angle(desired - joints(joint));
                acceleration = kp * error_angle - kd * velocities(joint);
                velocities(joint) = (velocities(joint) + acceleration * dt) * 0.995;
                joints(joint) = wrap_angle(joints(joint) + velocities(joint) * dt);
            end
            draw_arm();
            pause(dt);
        end
        if isgraphics(fig)
            if getappdata(fig, 'StopRequested')
                path_plot.XData = NaN;
                path_plot.YData = NaN;
                status.String = 'Stopped';
            else
                status.String = 'Simulation ready';
            end
        end
    end

    function stop_plan(~, ~)
        if isgraphics(fig)
            setappdata(fig, 'StopRequested', true);
            status.String = 'Stopped';
        end
    end

    function close_figure(~, ~)
        if isgraphics(fig)
            delete(fig);
        end
    end
end
