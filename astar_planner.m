%% ASTAR_PLANNER.M
% A* Multi-Agent Path Planner for Drone Fleet (MATLAB & Webots)
% Demonstrates global 2D grid search (f(n) = g(n) + h(n))
% -------------------------------------------------------------

function [path1, path2] = astar_planner()
    clc;
    fprintf('=== [MATLAB] Running A* Fleet Path Planner ===\n');

    %% 1. Define World Boundaries & Resolution
    % Apartment dimensions: X: [0 to 10] m, Y: [0 to 7] m
    mapWidth = 10;   % X size (meters)
    mapHeight = 7;   % Y size (meters)
    resolution = 5;  % 5 cells per meter (0.2m grid cell)
    
    map = binaryOccupancyMap(mapWidth, mapHeight, resolution);
    map.GridOriginInLocal = [0, 0];

    %% 2. Insert Obstacles (Apartment Layout)
    % Center Wall partition with doorway (X = 6.7 in grid -> X = -3.3 in Webots)
    % Wall from Y=3.5 to Y=6.9 (Doorway open at Y=0.5 to 3.5)
    [wallX, wallY] = meshgrid(linspace(6.5, 6.9, 10), linspace(3.5, 6.9, 25));
    setOccupancy(map, [wallX(:), wallY(:)], 1);

    % Kitchen Island / Counters (Grid X: 7.2 to 9.5, Y: 5.8 to 6.8)
    [kX, kY] = meshgrid(linspace(7.2, 9.5, 15), linspace(5.8, 6.8, 10));
    setOccupancy(map, [kX(:), kY(:)], 1);

    % Dining Table (Grid X: 8.6 to 9.6, Y: 1.0 to 2.2)
    [tX, tY] = meshgrid(linspace(8.6, 9.6, 10), linspace(1.0, 2.2, 10));
    setOccupancy(map, [tX(:), tY(:)], 1);

    % Living Room Sofas & Coffee Table (Grid X: 1.5 to 3.2, Y: 3.5 to 4.5)
    [sX, sY] = meshgrid(linspace(1.5, 3.2, 15), linspace(3.5, 4.5, 10));
    setOccupancy(map, [sX(:), sY(:)], 1);

    % Desk / Armchair (Grid X: 4.2 to 5.5, Y: 5.8 to 6.8)
    [dX, dY] = meshgrid(linspace(4.2, 5.5, 10), linspace(5.8, 6.8, 10));
    setOccupancy(map, [dX(:), dY(:)], 1);

    % Inflate obstacles to account for drone body + safety margin (0.25m)
    inflate(map, 0.25);

    %% 3. Initialize MATLAB A* Grid Planner
    planner = plannerAStarGrid(map);
    planner.TieBreaker = 'on';

    %% 4. Define Start and Goal for Drone 1 & Drone 2 (World Coordinates)
    % Drone 1 (Living Room -> Kitchen)
    start1_world = [2.5, 2.0];  % Corresponds to Webots [-7.5, -5.0]
    goal1_world  = [8.5, 4.5];  % Corresponds to Webots [-1.5, -2.5]

    % Drone 2 (Kitchen Open Area -> Living Room / Desk)
    start2_world = [8.0, 3.2];  % Corresponds to Webots [-2.0, -3.8] (Clear open floor)
    goal2_world  = [2.5, 5.8];  % Corresponds to Webots [-7.5, -1.2]

    % Convert Continuous World Coordinates to Discrete Grid Indices (Row, Col)
    start1_grid = world2grid(map, start1_world);
    goal1_grid  = world2grid(map, goal1_world);
    start2_grid = world2grid(map, start2_world);
    goal2_grid  = world2grid(map, goal2_world);

    %% 5. Compute A* Path for Drone 1
    tic;
    path1_grid_idx = plan(planner, start1_grid, goal1_grid);
    t1 = toc;
    path1_world = grid2world(map, path1_grid_idx);
    fprintf('[Drone 1] A* Path planned in %.4f seconds (%d waypoints)\n', t1, size(path1_world, 1));

    %% 6. Compute A* Path for Drone 2
    tic;
    path2_grid_idx = plan(planner, start2_grid, goal2_grid);
    t2 = toc;
    path2_world = grid2world(map, path2_grid_idx);
    fprintf('[Drone 2] A* Path planned in %.4f seconds (%d waypoints)\n', t2, size(path2_world, 1));

    %% 7. Convert to Webots World Coordinates
    % Conversion: Webots_X = World_X - 10, Webots_Y = World_Y - 7
    path1_webots = [path1_world(:, 1) - 10, path1_world(:, 2) - 7];
    path2_webots = [path2_world(:, 1) - 10, path2_world(:, 2) - 7];

    % Save computed paths for Webots controller (.mat and .csv)
    save('astar_paths.mat', 'path1_webots', 'path2_webots', 'path1_world', 'path2_world');
    writematrix(path1_webots, 'astar_path1.csv');
    writematrix(path2_webots, 'astar_path2.csv');
    fprintf('Saved waypoints to "astar_paths.mat" and CSV files\n');

    %% 8. Visualize A* Path Planning Results
    figure('Name', 'A* Multi-Agent Fleet Path Planner', 'NumberTitle', 'off', 'Color', 'w');
    show(map);
    hold on;
    
    % Plot Drone 1 Path
    plot(path1_world(:, 1), path1_world(:, 2), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Drone 1 (A* Path)');
    plot(start1_world(1), start1_world(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Drone 1 Start');
    plot(goal1_world(1), goal1_world(2), 'g*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Drone 1 Goal');

    % Plot Drone 2 Path
    plot(path2_world(:, 1), path2_world(:, 2), 'm--', 'LineWidth', 2.5, 'DisplayName', 'Drone 2 (A* Path)');
    plot(start2_world(1), start2_world(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Drone 2 Start');
    plot(goal2_world(1), goal2_world(2), 'r*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Drone 2 Goal');

    title('MATLAB A* Global Path Planning (Occupancy Grid)');
    xlabel('X [World Coordinates (m)]');
    ylabel('Y [World Coordinates (m)]');
    legend('Location', 'northeastoutside');
    grid on;
    axis equal;

    path1 = path1_webots;
    path2 = path2_webots;
end
