%% RUN_DEMO.M
% Master runner script for the A* Multi-Drone Fleet Demonstration
% -------------------------------------------------------------

clc; clear; close all;
fprintf('=======================================================\n');
fprintf('  A* MULTI-DRONE FLEET PATH PLANNING & SIMULATION DEMO \n');
fprintf('=======================================================\n\n');

%% Step 1: Run A* Global Planner in MATLAB
fprintf('Step 1: Calculating optimal collision-free paths using A*...\n');
[path1, path2] = astar_planner();

%% Step 2: Instructions to Launch Webots
fprintf('\n-------------------------------------------------------\n');
fprintf('Step 2: Start Webots Simulation\n');
fprintf('1. Open Webots R2025a.\n');
fprintf('2. Open the world: "c:\\AStar-Demo\\worlds\\astar_drone_apartment.wbt"\n');
fprintf('3. Press "Play" (Ctrl+2) in Webots.\n');
fprintf('-------------------------------------------------------\n\n');

%% Step 3: Real-Time Live Monitoring Dashboards in MATLAB
fprintf('Step 3: Live Visualizations (Run during flight in MATLAB):\n');
fprintf('   a) "live_camera_view" -> 1x2 Dual FPV Drone Cockpit Cameras + Target HUD\n');
fprintf('   b) "live_monitor"     -> Real-Time 2D Moving Map & Altitude Strip Chart\n\n');

%% Step 4: Post-Simulation Analysis
fprintf('Step 4: After simulation completes, run "plot_results" for final trajectory comparison.\n');
