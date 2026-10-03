%% LIVE_MONITOR.M
% Real-Time Flight Telemetry & 2D Animated Map Monitor for MATLAB
% Displays dynamic drone movement along A* routes during Webots simulation
% ------------------------------------------------------------------------

function live_monitor()
    clc;
    fprintf('=======================================================\n');
    fprintf('   LIVE TELEMETRY MONITOR & REAL-TIME A* TRACKER      \n');
    fprintf('   (Watching Webots flight simulation in real-time)   \n');
    fprintf('=======================================================\n\n');

    % Check if reference paths exist
    if ~exist('astar_paths.mat', 'file')
        error('Please run "astar_planner" or "run_demo" first to calculate A* paths!');
    end
    load('astar_paths.mat', 'path1_webots', 'path2_webots');

    % Initialize Figure
    fig = figure('Name', 'Live Drone Fleet A* Telemetry Monitor', ...
                 'NumberTitle', 'off', 'Color', 'w', 'Position', [80, 80, 1250, 620]);

    %% Subplot 1: 2D Live Moving Map
    subplot(1, 2, 1);
    hold on;
    grid on;
    axis equal;
    box on;

    % Draw Apartment Boundary and Obstacles
    rectangle('Position', [-10, -7, 10, 7], 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 2);
    plot([-3.3, -3.3], [-3.5, 0], 'k-', 'LineWidth', 4, 'DisplayName', 'Wall Partition');
    rectangle('Position', [-8.5, -3.5, 2.0, 2.0], 'FaceColor', [0.88 0.88 0.88], 'EdgeColor', [0.5 0.5 0.5]);
    rectangle('Position', [-3.0, -1.2, 2.5, 1.0], 'FaceColor', [0.88 0.88 0.88], 'EdgeColor', [0.5 0.5 0.5]);
    rectangle('Position', [-2.0, -5.8, 1.5, 1.6], 'FaceColor', [0.88 0.88 0.88], 'EdgeColor', [0.5 0.5 0.5]);
    patch(nan, nan, [0.88 0.88 0.88], 'EdgeColor', [0.5 0.5 0.5], 'DisplayName', 'Furniture Obstacles');

    % Static A* Reference Paths
    plot(path1_webots(:, 1), path1_webots(:, 2), 'b:', 'LineWidth', 1.8, 'DisplayName', 'Drone 1 (A* Path)');
    plot(path2_webots(:, 1), path2_webots(:, 2), 'm:', 'LineWidth', 1.8, 'DisplayName', 'Drone 2 (A* Path)');

    % Animated Flight Trail Lines
    h_trail1 = plot(nan, nan, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Drone 1 Trail');
    h_trail2 = plot(nan, nan, 'm-', 'LineWidth', 2.5, 'DisplayName', 'Drone 2 Trail');

    % Animated Drone Current Position Markers
    h_drone1 = plot(nan, nan, 'bp', 'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'Drone 1 (Live)');
    h_drone2 = plot(nan, nan, 'mp', 'MarkerSize', 14, 'MarkerFaceColor', 'm', 'DisplayName', 'Drone 2 (Live)');

    xlim([-10.5, 0.5]);
    ylim([-7.5, 0.5]);
    xlabel('X Position [m]');
    ylabel('Y Position [m]');
    title('Real-Time 2D Position Tracker (Live Physics Map)');
    legend('Location', 'southoutside', 'NumColumns', 3);

    % Live status text overlay
    h_status1 = text(-9.5, -0.6, 'Drone 1: Waiting...', 'Color', 'b', 'FontSize', 9, 'FontWeight', 'bold');
    h_status2 = text(-4.8, -0.6, 'Drone 2: Waiting...', 'Color', 'm', 'FontSize', 9, 'FontWeight', 'bold');

    %% Subplot 2: Real-Time Altitude & Speed Strip Chart
    subplot(1, 2, 2);
    hold on;
    grid on;
    box on;

    h_alt1 = plot(nan, nan, 'b-', 'LineWidth', 2, 'DisplayName', 'Drone 1 Altitude');
    h_alt2 = plot(nan, nan, 'm-', 'LineWidth', 2, 'DisplayName', 'Drone 2 Altitude');
    yline(1.0, 'r--', 'Target Altitude (1.0m)', 'LineWidth', 1.5, 'DisplayName', 'Setpoint (1.0m)');

    xlim([0, 30]);
    ylim([0, 1.8]);
    xlabel('Time Elapsed [s]');
    ylabel('Altitude Z [m]');
    title('Real-Time Altitude Controller Response');
    legend('Location', 'northeast');

    fprintf('Live monitor running. Start the Webots simulation to watch real-time flight!\n');
    fprintf('(Close the figure window to stop monitoring).\n');

    %% Real-Time Telemetry Polling Loop
    while ishandle(fig)
        % Read live logs from disk
        d1 = read_telemetry('trajectory_drone_1.csv');
        d2 = read_telemetry('trajectory_drone_2.csv');

        % Update Drone 1
        if ~isempty(d1) && size(d1, 1) >= 1
            set(h_trail1, 'XData', d1(:, 2), 'YData', d1(:, 3));
            set(h_drone1, 'XData', d1(end, 2), 'YData', d1(end, 3));
            set(h_alt1, 'XData', d1(:, 1), 'YData', d1(:, 4));
            set(h_status1, 'String', sprintf('Drone 1: X=%.2fm, Y=%.2fm, Z=%.2fm', d1(end, 2), d1(end, 3), d1(end, 4)));
        end

        % Update Drone 2
        if ~isempty(d2) && size(d2, 1) >= 1
            set(h_trail2, 'XData', d2(:, 2), 'YData', d2(:, 3));
            set(h_drone2, 'XData', d2(end, 2), 'YData', d2(end, 3));
            set(h_alt2, 'XData', d2(:, 1), 'YData', d2(:, 4));
            set(h_status2, 'String', sprintf('Drone 2: X=%.2fm, Y=%.2fm, Z=%.2fm', d2(end, 2), d2(end, 3), d2(end, 4)));
        end

        % Adjust Time Window
        max_t = 10;
        if ~isempty(d1), max_t = max(max_t, d1(end, 1) + 2); end
        if ~isempty(d2), max_t = max(max_t, d2(end, 1) + 2); end
        subplot(1, 2, 2);
        xlim([0, max_t]);

        drawnow limitrate;
        pause(0.08); % 12 Hz refresh rate
    end
end

function data = read_telemetry(filename)
    data = [];
    if exist(filename, 'file')
        try
            data = readmatrix(filename);
        catch
            data = [];
        end
    end
end
