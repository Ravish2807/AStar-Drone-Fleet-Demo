%% LIVE_CAMERA_VIEW.M
% Real-Time 1x2 Dual FPV Drone Camera Dashboard in MATLAB
% Displays live camera feed + Start, Current & Goal Target coordinates for both drones
% ----------------------------------------------------------------------------------

function live_camera_view()
    clc;
    fprintf('=======================================================\n');
    fprintf('   1x2 DUAL DRONE FPV CAMERA & TARGET MISSION VIEWER   \n');
    fprintf('=======================================================\n\n');

    % Create Dashboard Window
    fig = figure('Name', 'Dual Drone FPV Cockpit & Target HUD (1x2 Live View)', ...
                 'NumberTitle', 'off', 'Color', [0.1 0.1 0.12], 'Position', [60, 80, 1300, 620]);

    % Default placeholder frame (320x240 dark gradient)
    placeholder = uint8(zeros(240, 320, 3));
    placeholder(:, :, 1) = 25; placeholder(:, :, 2) = 30; placeholder(:, :, 3) = 45;

    %% Left Subplot: Drone 1 FPV Camera
    subplot(1, 2, 1);
    h_img1 = imshow(placeholder);
    title('DRONE 1 — FPV On-Board Camera (Living Room \rightarrow Kitchen)', ...
          'Color', [0.3 0.8 1.0], 'FontSize', 12, 'FontWeight', 'bold');
    
    % Mission Target Info Panel (Drone 1)
    text_info1 = sprintf(['\\bf\\color{white}MISSION PROFILE: Living Room to Kitchen\n' ...
                          '\\color{green}Start Station:\\rm\\color{white} [-7.50, -4.50, 0.0] m\n' ...
                          '\\color{yellow}Final Goal:\\rm\\color{white}    [-1.50, -2.50, 1.0] m\n' ...
                          '\\color{cyan}Current Pose:\\rm\\color{white}  Initializing...\n' ...
                          '\\color{magenta}Active Target:\\rm\\color{white} Initializing...\n' ...
                          '\\color{white}Progress:\\rm\\color{yellow}      Waiting for Webots takeoff...']);
    h_txt1 = xlabel(text_info1, 'Color', 'w', 'FontSize', 10, 'HorizontalAlignment', 'center');

    %% Right Subplot: Drone 2 FPV Camera
    subplot(1, 2, 2);
    h_img2 = imshow(placeholder);
    title('DRONE 2 — FPV On-Board Camera (Kitchen \rightarrow Living Room Desk)', ...
          'Color', [1.0 0.5 0.9], 'FontSize', 12, 'FontWeight', 'bold');

    % Mission Target Info Panel (Drone 2)
    text_info2 = sprintf(['\\bf\\color{white}MISSION PROFILE: Kitchen to Living Room Desk\n' ...
                          '\\color{green}Start Station:\\rm\\color{white} [-2.00, -3.80, 0.0] m\n' ...
                          '\\color{yellow}Final Goal:\\rm\\color{white}    [-7.50, -1.20, 1.0] m\n' ...
                          '\\color{cyan}Current Pose:\\rm\\color{white}  Initializing...\n' ...
                          '\\color{magenta}Active Target:\\rm\\color{white} Initializing...\n' ...
                          '\\color{white}Progress:\\rm\\color{yellow}      Waiting for Webots takeoff...']);
    h_txt2 = xlabel(text_info2, 'Color', 'w', 'FontSize', 10, 'HorizontalAlignment', 'center');

    fprintf('1x2 Live Camera View running. Press Play in Webots to stream camera & targets!\n');
    fprintf('(Close this figure window to stop the viewer).\n');

    %% Continuous Video & HUD Refresh Loop
    while ishandle(fig)
        % 1. Update Drone 1 Feed & Targets
        if exist('camera_drone_1.png', 'file')
            try
                img1 = imread('camera_drone_1.png');
                set(h_img1, 'CData', img1);
            catch
                % pass if file is being written
            end
        end

        s1 = read_status('status_drone_1.csv');
        if ~isempty(s1)
            t_cur = s1(1); x_act = s1(2); y_act = s1(3); z_act = s1(4);
            tx = s1(5); ty = s1(6); wp_curr = s1(7); wp_tot = s1(8);
            pct = (wp_curr / max(1, wp_tot)) * 100;
            
            set(h_txt1, 'String', sprintf(['\\bf\\color{white}MISSION: Living Room \\rightarrow Kitchen\n' ...
                                           '\\color{green}Start:\\rm\\color{white} [-7.50, -4.50] m  |  \\color{yellow}Goal:\\rm\\color{white} [-1.50, -2.50] m\n' ...
                                           '\\color{cyan}Pose:\\rm\\color{white}  X: %+.2fm, Y: %+.2fm, Z: %.2fm (t=%.1fs)\n' ...
                                           '\\color{magenta}Next Waypoint:\\rm\\color{white} [%+.2f, %+.2f] m\n' ...
                                           '\\color{white}Progress:\\rm\\color{yellow} WP %d of %d (%.0f%%)'], ...
                                           x_act, y_act, z_act, t_cur, tx, ty, wp_curr, wp_tot, pct));
        end

        % 2. Update Drone 2 Feed & Targets
        if exist('camera_drone_2.png', 'file')
            try
                img2 = imread('camera_drone_2.png');
                set(h_img2, 'CData', img2);
            catch
                % pass
            end
        end

        s2 = read_status('status_drone_2.csv');
        if ~isempty(s2)
            t_cur = s2(1); x_act = s2(2); y_act = s2(3); z_act = s2(4);
            tx = s2(5); ty = s2(6); wp_curr = s2(7); wp_tot = s2(8);
            pct = (wp_curr / max(1, wp_tot)) * 100;
            
            set(h_txt2, 'String', sprintf(['\\bf\\color{white}MISSION: Kitchen \\rightarrow Living Room Desk\n' ...
                                           '\\color{green}Start:\\rm\\color{white} [-2.00, -3.80] m  |  \\color{yellow}Goal:\\rm\\color{white} [-7.50, -1.20] m\n' ...
                                           '\\color{cyan}Pose:\\rm\\color{white}  X: %+.2fm, Y: %+.2fm, Z: %.2fm (t=%.1fs)\n' ...
                                           '\\color{magenta}Next Waypoint:\\rm\\color{white} [%+.2f, %+.2f] m\n' ...
                                           '\\color{white}Progress:\\rm\\color{yellow} WP %d of %d (%.0f%%)'], ...
                                           x_act, y_act, z_act, t_cur, tx, ty, wp_curr, wp_tot, pct));
        end

        drawnow limitrate;
        pause(0.06); % ~16 FPS refresh
    end
end

function status = read_status(filename)
    status = [];
    if exist(filename, 'file')
        try
            status = readmatrix(filename);
        catch
            status = [];
        end
    end
end
