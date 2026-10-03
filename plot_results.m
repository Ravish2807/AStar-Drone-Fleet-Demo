%% PLOT_RESULTS.M
% Evaluates and compares MATLAB A* Planned Path vs. Webots Real Flight Trajectory
% -------------------------------------------------------------------------------

function plot_results()
    clc;
    fprintf('=== Generating A* Path vs. Webots Flight Comparison ===\n');

    % Check if A* paths exist
    if ~exist('astar_paths.mat', 'file')
        error('Please run "astar_planner.m" first to generate the reference paths!');
    end
    load('astar_paths.mat', 'path1_webots', 'path2_webots');

    % Helper to load trajectory from CSV or MAT
    d1_log = load_traj('trajectory_drone_1');
    d2_log = load_traj('trajectory_drone_2');

    has_drone1 = ~isempty(d1_log);
    has_drone2 = ~isempty(d2_log);

    % Create Figure
    fig = figure('Name', 'A* Global Path vs. Webots Controller Tracking', ...
                 'NumberTitle', 'off', 'Color', 'w', 'Position', [100, 100, 1200, 600]);

    %% Subplot 1: 2D World Trajectory Comparison (A* vs. Actual)
    subplot(1, 2, 1);
    hold on;
    grid on;
    axis equal;
    box on;

    % Draw Apartment Boundary and Key Obstacles in Webots Frame
    rectangle('Position', [-10, -7, 10, 7], 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 2);
    % Center Wall Partition: X = -3.3, Y from -3.5 to 0 (Doorway open at Y from -7 to -3.5)
    plot([-3.3, -3.3], [-3.5, 0], 'k-', 'LineWidth', 4, 'DisplayName', 'Wall Partition');
    
    % Furniture Obstacles
    rectangle('Position', [-8.5, -3.5, 2.0, 2.0], 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', [0.5 0.5 0.5]);
    rectangle('Position', [-3.0, -1.2, 2.5, 1.0], 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', [0.5 0.5 0.5]);
    rectangle('Position', [-2.0, -5.8, 1.5, 1.6], 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', [0.5 0.5 0.5]);

    % Dummy patch for Obstacles in legend
    patch(nan, nan, [0.85 0.85 0.85], 'EdgeColor', [0.5 0.5 0.5], 'DisplayName', 'Obstacles (Furniture)');

    % Plot A* Planned Paths
    plot(path1_webots(:, 1), path1_webots(:, 2), 'b--', 'LineWidth', 2, 'DisplayName', 'Drone 1 (A* Planned)');
    plot(path2_webots(:, 1), path2_webots(:, 2), 'm--', 'LineWidth', 2, 'DisplayName', 'Drone 2 (A* Planned)');

    % Plot Webots Actual Trajectories
    if has_drone1
        plot(d1_log(:, 2), d1_log(:, 3), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Drone 1 (Webots Actual)');
        plot(d1_log(1, 2), d1_log(1, 3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Drone 1 Start');
    end

    if has_drone2
        plot(d2_log(:, 2), d2_log(:, 3), 'm-', 'LineWidth', 2.5, 'DisplayName', 'Drone 2 (Webots Actual)');
        plot(d2_log(1, 2), d2_log(1, 3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Drone 2 Start');
    end

    xlabel('X Position [m] (Webots Frame)');
    ylabel('Y Position [m] (Webots Frame)');
    title('2D Flight Trajectories (Planned vs Flown)');
    legend('Location', 'southoutside', 'NumColumns', 2);

    %% Subplot 2: Altitude & Controller Performance
    subplot(1, 2, 2);
    hold on;
    grid on;
    box on;

    if has_drone1
        plot(d1_log(:, 1), d1_log(:, 4), 'b-', 'LineWidth', 2, 'DisplayName', 'Drone 1 Altitude');
    end

    if has_drone2
        plot(d2_log(:, 1), d2_log(:, 4), 'm-', 'LineWidth', 2, 'DisplayName', 'Drone 2 Altitude');
    end

    yline(1.0, 'r--', 'Target Altitude (1.0 m)', 'LineWidth', 1.5, 'DisplayName', 'Setpoint (1.0m)');
    xlabel('Time [s]');
    ylabel('Altitude Z [m]');
    title('Drone Altitude Controller Response');
    legend('Location', 'northeast');

    % Print quantitative metrics to console
    if has_drone1
        fprintf('[Drone 1] Total Flown Duration: %.2f s | Final Position: [%.2f, %.2f, %.2f]\n', ...
            d1_log(end, 1), d1_log(end, 2), d1_log(end, 3), d1_log(end, 4));
    end
    if has_drone2
        fprintf('[Drone 2] Total Flown Duration: %.2f s | Final Position: [%.2f, %.2f, %.2f]\n', ...
            d2_log(end, 1), d2_log(end, 2), d2_log(end, 3), d2_log(end, 4));
    end
    fprintf('Comparison plot successfully generated.\n');
end

function traj = load_traj(basename)
    traj = [];
    csv_file = [basename, '.csv'];
    mat_file = [basename, '.mat'];
    if exist(csv_file, 'file')
        traj = readmatrix(csv_file);
    elseif exist(mat_file, 'file')
        d = load(mat_file);
        if isfield(d, 'trajectory_log')
            traj = d.trajectory_log;
        end
    end
end
