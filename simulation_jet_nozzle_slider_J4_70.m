function simulation_jet_nozzle_slider_J4()
    clc; clear; close all;
    
    %% ============================================================
    % LINK GEOMETRY & KINEMATIC DATUM PARAMETERS
    %% ============================================================
    L1 = 5.82;
    L2 = 6.22;
    L2_ext = 4.5;
    L3 = 9.07;
    petal_bend = deg2rad(-15);
    
    % GEOMETRIC DATUM 1: Master Fixed Ground Origin J1(0,0)
    J1 = [0, 0];
    
    % GEOMETRIC & KINEMATIC DATUM 2: Slider J4 Rail & Neutral Home (x2_0)
    theta6_ref = deg2rad(34.3);   
    L6_ref     = 3.55;
    y_rail_J4  = L6_ref * sin(theta6_ref);   % Horizontal rail height datum
    J4_x0      = L6_ref * cos(theta6_ref);   % Kinematic neutral home datum (x2_0)
    r_amp      = 2.0;                        % +/- travel amplitude
    r_min      = J4_x0 - r_amp;              % Aft kinematic limit stop
    r_max      = J4_x0 + r_amp;              % Forward kinematic limit stop
    slider_w   = 1.0; slider_h = 0.5;
    
    % GEOMETRIC & KINEMATIC DATUM 3: Piston J5 Rail
    R5 = 0.42;
    R6 = 0.32;
    bulge_amount = 1.8;
    y_rail_J5 = -0.5;                        % Piston axis height datum
    cyl_w = 2.5; cyl_h = 1.0;
    cyl_x = -3.5; cyl_y = y_rail_J5 - cyl_h/2;
    
    % GEOMETRIC DATUM 4: Engine Centerline (Axis of Symmetry)
    y_center = -14.0;         % Engine centerline Y-datum (mm)
    is_axisymmetric = true;   % true: A_t/A_e = (R_t/R_e)^2 | false: 2D (R_t/R_e)

    %% ============================================================
    % PRE-CALCULATE 1D COUPLED PATH & 2D WORKSPACE SURFACE (x1 vs x2)
    %% ============================================================
    theta1_min_datum = deg2rad(-50.0);
    theta1_max_datum = deg2rad(-15.0);
    x_min_stroke = getPistonX(theta1_min_datum);
    x_max_stroke = getPistonX(theta1_max_datum);
    x1_home_datum = 0.5 * (x_min_stroke + x_max_stroke); % Piston mid-stroke datum (x1_0)
    
    % --- 1. Coupled 1D Trajectory Pre-calculation ---
    N_pre = 180;
    xp_vec = linspace(x_min_stroke, x_max_stroke, N_pre);
    r_vec_pre = J4_x0 + r_amp * sin(linspace(0, 2*pi, N_pre));
    AreaRatio_path = zeros(1, N_pre);
    Tip_path = zeros(N_pre, 2);
    
    curr_th = theta1_min_datum;
    opt_pre = optimset('Display','off');
    for i = 1:N_pre
        th_solved = fzero(@(th) getPistonX(th) - xp_vec(i), curr_th, opt_pre);
        curr_th = th_solved;
        
        J2_t = J1 + L1 * [cos(th_solved), sin(th_solved)];
        J4_t = [r_vec_pre(i), y_rail_J4];
        J3_t = solveJ3(J2_t, J4_t);
        
        if any(isnan(J3_t))
            AreaRatio_path(i) = NaN;
            Tip_path(i,:) = [NaN, NaN];
            continue;
        end
        
        theta2_t = atan2(J3_t(2) - J2_t(2), J3_t(1) - J2_t(1));
        theta_ext_t = theta2_t + petal_bend;
        J_end_t = J3_t + L2_ext * [cos(theta_ext_t), sin(theta_ext_t)];
        Tip_path(i,:) = J_end_t;
        AreaRatio_path(i) = computeAreaRatio(J2_t, J_end_t);
    end

    % --- 2. Full 2D Grid Pre-calculation for 3D Surface Plot ---
    N_grid_x1 = 65;
    N_grid_x2 = 65;
    x1_grid_vec = linspace(x_min_stroke, x_max_stroke, N_grid_x1);
    x2_grid_vec = linspace(r_min, r_max, N_grid_x2);
    [X1_mesh, X2_mesh] = meshgrid(x1_grid_vec, x2_grid_vec);
    AreaRatio_mesh = nan(N_grid_x2, N_grid_x1);
    
    th_guess = theta1_min_datum;
    for col = 1:N_grid_x1
        th_s = fzero(@(th) getPistonX(th) - x1_grid_vec(col), th_guess, opt_pre);
        th_guess = th_s;
        J2_col = J1 + L1 * [cos(th_s), sin(th_s)];
        
        for row = 1:N_grid_x2
            J4_rc = [x2_grid_vec(row), y_rail_J4];
            J3_rc = solveJ3(J2_col, J4_rc);
            if ~any(isnan(J3_rc))
                th2_rc = atan2(J3_rc(2) - J2_col(2), J3_rc(1) - J2_col(1));
                J_end_rc = J3_rc + L2_ext * [cos(th2_rc + petal_bend), sin(th2_rc + petal_bend)];
                AreaRatio_mesh(row, col) = computeAreaRatio(J2_col, J_end_rc);
            end
        end
    end

    % --- 3. Compute Kinematic Home Datum State (x1 = x1_0, x2 = J4_x0) ---
    th_home = fzero(@(th) getPistonX(th) - x1_home_datum, deg2rad(-32), opt_pre);
    J2_home = J1 + L1 * [cos(th_home), sin(th_home)];
    J4_home = [J4_x0, y_rail_J4];
    J3_home = solveJ3(J2_home, J4_home);
    th2_home = atan2(J3_home(2) - J2_home(2), J3_home(1) - J2_home(1));
    J_end_home = J3_home + L2_ext * [cos(th2_home + petal_bend), sin(th2_home + petal_bend)];
    ar_home_datum = computeAreaRatio(J2_home, J_end_home);

    % --- 4. Identify Special Points (Cycle & Global Surface Extremes) ---
    [max_ar_cyc, idx_max_cyc] = max(AreaRatio_path);
    [min_ar_cyc, idx_min_cyc] = min(AreaRatio_path);
    
    [max_ar_glob, idx_glob_max] = max(AreaRatio_mesh(:));
    [r_gmax, c_gmax] = ind2sub(size(AreaRatio_mesh), idx_glob_max);
    x1_gmax = X1_mesh(r_gmax, c_gmax); x2_gmax = X2_mesh(r_gmax, c_gmax);
    
    [min_ar_glob, idx_glob_min] = min(AreaRatio_mesh(:));
    [r_gmin, c_gmin] = ind2sub(size(AreaRatio_mesh), idx_glob_min);
    x1_gmin = X1_mesh(r_gmin, c_gmin); x2_gmin = X2_mesh(r_gmin, c_gmin);

    %% ============================================================
    % FIGURE & 2-PANEL SIDE-BY-SIDE SETUP (NO FORCE PLOT)
    %% ============================================================
    fig = figure('Name','Jet Engine CD Nozzle - Geometric & Kinematic Datums + 3D Area Ratio',...
        'NumberTitle','off','Color',[0.08 0.08 0.09],'Position',[25 40 1620 860]);
    
    % LEFT PANEL: Expanded Mechanism Animation with Geometric & Kinematic Datums
    ax = axes('Parent',fig,'Position',[0.04 0.12 0.48 0.82],'Color',[0.05 0.05 0.06]);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    set(ax, 'XColor', [0.75 0.75 0.75], 'YColor', [0.75 0.75 0.75], 'GridColor', [0.22 0.22 0.22]);
    xlim(ax,[-4.8 18.5]); ylim(ax,[-15.5 5.5]);
    xlabel(ax,'X - Axial Coordinate from Datum J1 (mm)', 'Color', 'w', 'FontWeight', 'bold');
    ylabel(ax,'Y - Radial Coordinate from Datum J1 (mm)', 'Color', 'w', 'FontWeight', 'bold');
    title(ax,'2-DOF Nozzle Mechanism: Geometric & Kinematic Datums','Color','w','FontSize',12);
    
    %% ============================================================
    % PLOT GEOMETRIC & KINEMATIC DATUMS ON MECHANISM VIEW
    %% ============================================================
    % 1. Master Orthogonal Datum Axes through Origin J1(0,0)
    line(ax, [0 0], [-15.5 5.5], 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 1.2);
    line(ax, [-4.8 18.5], [0 0], 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 1.2);
    text(ax, 0.2, 4.8, 'AXIAL DATUM PLANE (X = 0)', 'Color', [0.65 0.65 0.65], 'FontSize', 8, 'FontWeight', 'bold');
    text(ax, 12.2, 0.35, 'RADIAL DATUM PLANE (Y = 0)', 'Color', [0.65 0.65 0.65], 'FontSize', 8, 'FontWeight', 'bold');
    
    % 2. Crank Angular Stroke Limit Datums (theta1_min = -50 deg, theta1_max = -15 deg)
    line(ax, [0, (L1+1.2)*cos(theta1_min_datum)], [0, (L1+1.2)*sin(theta1_min_datum)], ...
        'Color', [0.85 0.35 0.2], 'LineStyle', ':', 'LineWidth', 1.1);
    text(ax, (L1+1.3)*cos(theta1_min_datum), (L1+1.3)*sin(theta1_min_datum)-0.3, ...
        '\theta_{1,min} = -50^\circ', 'Color', [0.95 0.45 0.3], 'FontSize', 8);
    line(ax, [0, (L1+1.2)*cos(theta1_max_datum)], [0, (L1+1.2)*sin(theta1_max_datum)], ...
        'Color', [0.85 0.35 0.2], 'LineStyle', ':', 'LineWidth', 1.1);
    text(ax, (L1+1.3)*cos(theta1_max_datum), (L1+1.3)*sin(theta1_max_datum)+0.2, ...
        '\theta_{1,max} = -15^\circ', 'Color', [0.95 0.45 0.3], 'FontSize', 8);

    % 3. Ghost Reference Linkage at KINEMATIC HOME DATUM (x1_0, x2_0)
    plot(ax, [J1(1) J2_home(1)], [J1(2) J2_home(2)], '--', 'Color', [0.45 0.45 0.5], 'LineWidth', 1.5);
    plot(ax, [J2_home(1) J3_home(1) J_end_home(1) J2_home(1)], ...
             [J2_home(2) J3_home(2) J_end_home(2) J2_home(2)], '--', 'Color', [0.45 0.45 0.5], 'LineWidth', 1.5);
    plot(ax, [J4_home(1) J3_home(1)], [J4_home(2) J3_home(2)], '--', 'Color', [0.45 0.45 0.5], 'LineWidth', 1.5);
    plot(ax, J_end_home(1), J_end_home(2), 's', 'MarkerSize', 7, 'MarkerFaceColor', [0.2 0.95 0.3], 'MarkerEdgeColor', 'w');
    text(ax, J_end_home(1)+0.3, J_end_home(2)+0.3, 'Home Datum Tip', 'Color', [0.4 0.9 0.5], 'FontSize', 8);

    % 4. Engine Centerline Datum (Y_center = -14.0 mm)
    line(ax, [-4.8 18.5], [y_center y_center], 'Color', [1 0.25 0.25], 'LineStyle', '-.', 'LineWidth', 1.8);
    text(ax, -4.4, y_center + 0.65, sprintf('GEOMETRIC DATUM: Engine Centerline Axis (Y_{center} = %.1f mm)', y_center), ...
        'Color', [1 0.45 0.45], 'FontSize', 9, 'FontWeight', 'bold');
    
    % 5. Slider J4 Kinematic Rail Datum & Travel Stops (r_min, J4_x0, r_max)
    line(ax, [r_min-0.8, r_max+0.8], [y_rail_J4, y_rail_J4], ...
        'Color', [0.85 0.25 0.85], 'LineStyle', '--', 'LineWidth', 1.5);
    plot(ax, [r_min, J4_x0, r_max], [y_rail_J4, y_rail_J4, y_rail_J4], '|', ...
        'Color', [1 0.5 1], 'MarkerSize', 11, 'LineWidth', 2.2);
    plot(ax, J4_x0, y_rail_J4, 'd', 'MarkerSize', 7, 'MarkerFaceColor', [1 0.4 1], 'MarkerEdgeColor', 'w');
    text(ax, r_min, y_rail_J4 + 0.85, sprintf('x_{2,min}\n%.2f', r_min), 'Color', [0.9 0.5 0.9], 'FontSize', 7.5, 'HorizontalAlignment', 'center');
    text(ax, J4_x0, y_rail_J4 + 0.95, sprintf('HOME DATUM\nx_{2,0}=%.2f', J4_x0), 'Color', [1 0.8 1], 'FontSize', 7.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, r_max, y_rail_J4 + 0.85, sprintf('x_{2,max}\n%.2f', r_max), 'Color', [0.9 0.5 0.9], 'FontSize', 7.5, 'HorizontalAlignment', 'center');
    text(ax, r_max+1.0, y_rail_J4, sprintf('Slider Rail Datum\n(Y = %.2f mm)', y_rail_J4), 'Color', [0.9 0.5 0.9], 'FontSize', 8);

    % 6. Piston J5 Kinematic Rail Datum & Stroke Stops (x_min_stroke, x1_home_datum, x_max_stroke)
    line(ax, [-1 7.5], [y_rail_J5 y_rail_J5], 'Color', [0.85 0.85 0.2], 'LineStyle', '--', 'LineWidth', 1.5);
    plot(ax, [x_min_stroke, x1_home_datum, x_max_stroke], [y_rail_J5, y_rail_J5, y_rail_J5], '|', ...
        'Color', [1 1 0.2], 'MarkerSize', 11, 'LineWidth', 2.2);
    text(ax, x_min_stroke, y_rail_J5 - 0.95, sprintf('x_{1,min}\n%.2f', x_min_stroke), 'Color', [0.9 0.9 0.3], 'FontSize', 7.5, 'HorizontalAlignment', 'center');
    text(ax, x1_home_datum, y_rail_J5 - 1.05, sprintf('MID DATUM\nx_{1,0}=%.2f', x1_home_datum), 'Color', [1 1 0.6], 'FontSize', 7.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, x_max_stroke, y_rail_J5 - 0.95, sprintf('x_{1,max}\n%.2f', x_max_stroke), 'Color', [0.9 0.9 0.3], 'FontSize', 7.5, 'HorizontalAlignment', 'center');

    %% ============================================================
    % ACTIVE GRAPHICAL ELEMENTS (MECHANISM)
    %% ============================================================
    hSlider4 = patch(ax, 'XData', [], 'YData', [], 'FaceColor', [0.55 0.55 0.55], 'EdgeColor', 'w', 'LineWidth', 1.5);
    hL1_patch = patch(ax, 'XData', [], 'YData', [], 'FaceColor', [0.85 0.3 0.2], 'EdgeColor', 'w', 'LineWidth', 1.5, 'FaceAlpha', 0.9);
    
    hL1_holes = gobjects(1,3);
    for idx = 1:3
        hL1_holes(idx) = patch(ax, 'XData', [], 'YData', [], 'FaceColor', [0.05 0.05 0.06], 'EdgeColor', 'w', 'LineWidth', 1);
    end
    
    hL2 = line(ax, [0 0 0 0], [0 0 0 0], 'LineWidth', 7, 'Color', [0.2 0.75 0.35]);
    hL3 = line(ax, [0 0], [0 0], 'LineWidth', 6, 'Color', [0.2 0.55 0.95]);
    
    patch(ax, 'XData', [cyl_x cyl_x+cyl_w cyl_x+cyl_w cyl_x], ...
              'YData', [cyl_y cyl_y cyl_y+cyl_h cyl_y+cyl_h], ...
              'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'w', 'LineWidth', 1.5);
          
    hRod = line(ax, [0 0], [0 0], 'LineWidth', 5.5, 'Color', [0.75 0.75 0.75]);
    hTraj = line(ax, NaN, NaN, 'Color', [0 1 1], 'LineStyle', '--', 'LineWidth', 2.0);
    
    % Mark Special Tip Extremes on Mechanism View
    plot(ax, Tip_path(idx_max_cyc,1), Tip_path(idx_max_cyc,2), 'p', 'MarkerSize', 11, 'MarkerFaceColor', [1 0.85 0], 'MarkerEdgeColor', 'k');
    plot(ax, Tip_path(idx_min_cyc,1), Tip_path(idx_min_cyc,2), 'd', 'MarkerSize', 9, 'MarkerFaceColor', [1 0.35 0.8], 'MarkerEdgeColor', 'w');
    
    % Visual throat & outlet radius drop-lines to Centerline Datum
    hThroatLine = line(ax, [0 0], [0 0], 'Color', [1 0.85 0.2], 'LineStyle', ':', 'LineWidth', 2.0);
    hOutletLine = line(ax, [0 0], [0 0], 'Color', [0.2 1 0.85], 'LineStyle', ':', 'LineWidth', 2.0);
    hRtText = text(ax, 0, 0, '', 'Color', [1 0.88 0.3], 'FontSize', 8.5, 'FontWeight', 'bold');
    hReText = text(ax, 0, 0, '', 'Color', [0.3 1 0.88], 'FontSize', 8.5, 'FontWeight', 'bold');
    
    hJ4 = rectangle(ax, 'Position', [0 0 0 0], 'Curvature', [1 1], 'FaceColor', [0.95 0.75 0.1], 'EdgeColor', 'w');
    hJ5 = rectangle(ax, 'Position', [0 0 0 0], 'Curvature', [1 1], 'FaceColor', [0.95 0.75 0.1], 'EdgeColor', 'w');
    hJ_end = plot(ax, 0, 0, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', [0.2 0.75 0.35], 'LineWidth', 1.5);
    hJoints = plot(ax, zeros(1,4), zeros(1,4), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'w', 'LineWidth', 1.5);
    
    % Master Origin Datum Target Symbol at J1(0,0)
    plot(ax, J1(1), J1(2), 'ro', 'MarkerSize', 11, 'LineWidth', 1.8);
    plot(ax, J1(1), J1(2), 'r+', 'MarkerSize', 13, 'LineWidth', 1.8);
    
    hLabels.J1 = text(ax, J1(1)-2.4, J1(2)+0.65, 'ORIGIN DATUM J1(0,0)', 'Color', [1 0.45 0.45], 'FontWeight', 'bold', 'FontSize', 8.5);
    hLabels.J2 = text(ax, 0, 0, 'J2 (Throat)', 'Color', [1 0.9 0.4], 'FontWeight', 'bold', 'FontSize', 8.5);
    hLabels.J3 = text(ax, 0, 0, 'J3', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 8.5);
    hLabels.J4 = text(ax, 0, 0, 'J4 (Slider x_2)', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 8.5);
    hLabels.J5 = text(ax, 0, 0, 'J5 (Piston x_1)', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 8.5);
    hLabels.J7 = text(ax, cyl_x - 0.2, cyl_y + 1.4, 'Piston Actuator', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 8.5);
    hLabels.J_end = text(ax, 0, 0, '  Outlet Tip', 'Color', [0.4 1 0.8], 'FontWeight', 'bold', 'FontSize', 8.5);
    
    % Live Telemetry & Datum Offset HUD Box
    hHudBox = text(ax, 9.6, -10.2, '', 'Color', 'w', 'FontSize', 8.5, 'FontName', 'Consolas', ...
        'BackgroundColor', [0.11 0.11 0.14], 'EdgeColor', [0.45 0.45 0.5], 'Margin', 6);

    %% ============================================================
    % RIGHT SUBPLOT: 3D SURFACE, KINEMATIC DATUMS & SPECIAL POINTS
    %% ============================================================
    ax3 = axes('Parent',fig,'Position',[0.56 0.12 0.41 0.82],'Color',[0.05 0.05 0.06]);
    hold(ax3, 'on'); grid(ax3, 'on');
    
    z_floor = min_ar_glob - 0.08;
    z_ceil  = max_ar_glob + 0.10;
    
    % 1. Plot 3D Smooth Surface over (x1, x2) state space
    surf(ax3, X1_mesh, X2_mesh, AreaRatio_mesh, ...
        'EdgeColor', [0.2 0.2 0.2], 'EdgeAlpha', 0.28, 'FaceAlpha', 0.85, ...
        'DisplayName', '2-DOF State Surface A_t/A_e(x_1,x_2)');
    
    % 2. AERODYNAMIC SONIC DATUM PLANE: A_throat / A_outlet = 1.0
    surf(ax3, [x_min_stroke x_max_stroke; x_min_stroke x_max_stroke], ...
              [r_min r_min; r_max r_max], ...
              [1.0 1.0; 1.0 1.0], ...
              'FaceColor', [1 0.2 0.2], 'FaceAlpha', 0.20, ...
              'EdgeColor', [1 0.35 0.35], 'LineStyle', '--', 'LineWidth', 1.2, ...
              'DisplayName', 'Sonic Datum Plane (A_t/A_e = 1.0)');
          
    % 3. Sonic Transition Curve (A_t = A_e) on Surface
    contour3(ax3, X1_mesh, X2_mesh, AreaRatio_mesh, [1.0 1.0], ...
        'LineColor', 'w', 'LineWidth', 2.8, 'DisplayName', 'Sonic Datum Contour (A_t = A_e)');
    
    % 4. KINEMATIC DATUM CURVE 1: Slider Home Plane (x2 = J4_x0)
    ar_x2_datum = interp2(X1_mesh, X2_mesh, AreaRatio_mesh, x1_grid_vec, J4_x0 * ones(size(x1_grid_vec)));
    plot3(ax3, x1_grid_vec, J4_x0 * ones(size(x1_grid_vec)), ar_x2_datum, ...
        'Color', [1 0.45 1], 'LineStyle', '-.', 'LineWidth', 2.2, ...
        'DisplayName', sprintf('Slider Home Datum (x_{2,0} = %.2f mm)', J4_x0));

    % 5. KINEMATIC DATUM CURVE 2: Piston Mid-Stroke Plane (x1 = x1_home_datum)
    ar_x1_datum = interp2(X1_mesh, X2_mesh, AreaRatio_mesh, x1_home_datum * ones(size(x2_grid_vec)), x2_grid_vec);
    plot3(ax3, x1_home_datum * ones(size(x2_grid_vec)), x2_grid_vec, ar_x1_datum, ...
        'Color', [1 0.95 0.3], 'LineStyle', '-.', 'LineWidth', 2.2, ...
        'DisplayName', sprintf('Piston Mid-Stroke Datum (x_{1,0} = %.2f mm)', x1_home_datum));

    % 6. Plot 2D Floor Projection (Shadow) of Coupled Trajectory + Floor Datum Crosshairs
    plot3(ax3, [x_min_stroke x_max_stroke], [J4_x0 J4_x0], [z_floor z_floor], ...
        'Color', [0.6 0.3 0.6], 'LineStyle', ':', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot3(ax3, [x1_home_datum x1_home_datum], [r_min r_max], [z_floor z_floor], ...
        'Color', [0.6 0.6 0.2], 'LineStyle', ':', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot3(ax3, xp_vec, r_vec_pre, z_floor * ones(size(xp_vec)), ...
        'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 1.6, ...
        'DisplayName', '2D Trajectory Floor Projection');

    % 7. Plot the 3D Coupled Trajectory Curve on top of the surface
    plot3(ax3, xp_vec, r_vec_pre, AreaRatio_path, ...
        'Color', [0 1 1], 'LineWidth', 3.2, 'DisplayName', 'Coupled Cycle 3D Curve');
    
    % --- PLOT SPECIAL POINTS WITH VERTICAL DROP-LINES TO FLOOR ---
    % A. Kinematic Home Datum Point (x1_0, x2_0)
    plot3(ax3, [x1_home_datum x1_home_datum], [J4_x0 J4_x0], [z_floor ar_home_datum], 'g--', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    plot3(ax3, x1_home_datum, J4_x0, ar_home_datum, 's', 'MarkerSize', 11, ...
        'MarkerFaceColor', [0.2 0.95 0.3], 'MarkerEdgeColor', 'k', 'LineWidth', 1.4, ...
        'DisplayName', sprintf('Kinematic Home Datum: %.3f (x_{1,0}=%.2f, x_{2,0}=%.2f)', ar_home_datum, x1_home_datum, J4_x0));
    text(ax3, x1_home_datum, J4_x0, ar_home_datum + 0.055, ...
        sprintf('  HOME DATUM (%.2f)', ar_home_datum), ...
        'Color', [0.35 1 0.45], 'FontSize', 8.5, 'FontWeight', 'bold');

    % B. Cycle Peak Area Ratio
    x1_cmax = xp_vec(idx_max_cyc); x2_cmax = r_vec_pre(idx_max_cyc);
    plot3(ax3, [x1_cmax x1_cmax], [x2_cmax x2_cmax], [z_floor max_ar_cyc], 'y--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot3(ax3, x1_cmax, x2_cmax, max_ar_cyc, 'p', 'MarkerSize', 14, ...
        'MarkerFaceColor', [1 0.85 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.2, ...
        'DisplayName', sprintf('Cycle Max A_t/A_e = %.3f (x_1=%.2f, x_2=%.2f)', max_ar_cyc, x1_cmax, x2_cmax));
    text(ax3, x1_cmax, x2_cmax, max_ar_cyc + 0.05, ...
        sprintf('  Cycle Max: %.2f\n  (x_1=%.2f, x_2=%.2f)', max_ar_cyc, x1_cmax, x2_cmax), ...
        'Color', [1 0.9 0.2], 'FontSize', 8, 'FontWeight', 'bold');

    % C. Cycle Minimum Area Ratio
    x1_cmin = xp_vec(idx_min_cyc); x2_cmin = r_vec_pre(idx_min_cyc);
    plot3(ax3, [x1_cmin x1_cmin], [x2_cmin x2_cmin], [z_floor min_ar_cyc], 'm--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot3(ax3, x1_cmin, x2_cmin, min_ar_cyc, 'd', 'MarkerSize', 10, ...
        'MarkerFaceColor', [1 0.35 0.8], 'MarkerEdgeColor', 'w', 'LineWidth', 1.2, ...
        'DisplayName', sprintf('Cycle Min A_t/A_e = %.3f (x_1=%.2f, x_2=%.2f)', min_ar_cyc, x1_cmin, x2_cmin));
    text(ax3, x1_cmin, x2_cmin, min_ar_cyc - 0.055, ...
        sprintf('  Cycle Min: %.2f (Max Expansion)', min_ar_cyc), ...
        'Color', [1 0.5 0.9], 'FontSize', 8, 'FontWeight', 'bold');

    % D. Global Workspace Surface Maximum & Minimum
    plot3(ax3, x1_gmax, x2_gmax, max_ar_glob, 'h', 'MarkerSize', 12, ...
        'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'r', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Global Surface Max = %.3f (x_1=%.2f, x_2=%.2f)', max_ar_glob, x1_gmax, x2_gmax));
    text(ax3, x1_gmax, x2_gmax, max_ar_glob + 0.04, ...
        sprintf('  Global Max: %.2f', max_ar_glob), 'Color', 'w', 'FontSize', 8, 'FontWeight', 'bold');

    plot3(ax3, x1_gmin, x2_gmin, min_ar_glob, 'h', 'MarkerSize', 12, ...
        'MarkerFaceColor', [0 1 0.8], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Global Surface Min = %.3f (x_1=%.2f, x_2=%.2f)', min_ar_glob, x1_gmin, x2_gmin));
    text(ax3, x1_gmin, x2_gmin, min_ar_glob + 0.04, ...
        sprintf('  Global Min: %.2f', min_ar_glob), 'Color', [0.2 1 0.8], 'FontSize', 8, 'FontWeight', 'bold');

    % E. Live Operating Point Marker + Live Drop-Line
    hMarker3D = plot3(ax3, xp_vec(1), r_vec_pre(1), AreaRatio_path(1), ...
        'o', 'MarkerSize', 10, 'MarkerFaceColor', [1 0.15 0.15], 'MarkerEdgeColor', 'w', ...
        'LineWidth', 1.8, 'DisplayName', 'Live Operating State');
    hDropLine3D = plot3(ax3, [xp_vec(1) xp_vec(1)], [r_vec_pre(1) r_vec_pre(1)], ...
        [z_floor AreaRatio_path(1)], 'r-', 'LineWidth', 1.5, 'HandleVisibility', 'off');

    colormap(ax3, parula);
    cb = colorbar(ax3, 'Color', 'w');
    cb.Label.String = 'Area Ratio (A_{throat} / A_{outlet})';
    cb.Label.FontSize = 10;
    cb.Label.Color = 'w';
    
    zlim(ax3, [z_floor, z_ceil]);
    set(ax3, 'XColor', [0.8 0.8 0.2], 'YColor', [0.85 0.5 0.9], 'ZColor', [0 1 1], ...
             'GridColor', [0.4 0.4 0.4], 'GridAlpha', 0.5);
    xlabel(ax3, 'Piston Displacement x_1 (mm)', 'FontWeight', 'bold');
    ylabel(ax3, 'Slider Displacement x_2 (mm)', 'FontWeight', 'bold');
    zlabel(ax3, 'Area Ratio A_{throat} / A_{outlet}', 'FontWeight', 'bold');
    title(ax3, '3D Area Ratio Surface, Kinematic Home Crosshairs & Sonic Datum', 'Color', 'w', 'FontSize', 12);
    legend(ax3, 'Location', 'northwest', 'TextColor', 'w', 'Color', [0.13 0.13 0.13], 'FontSize', 8);
    
    view(ax3, -38, 26);
    rotate3d(ax3, 'on');

    %% ============================================================
    % BOTTOM DASHBOARD BAR: BUTTON & DATUM REFERENCE SUMMARY
    %% ============================================================
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.82 0.015 0.15 0.052], ...
        'String', 'ANALYZE & ANIMATE', 'FontSize', 10, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.18 0.62 0.32], 'ForegroundColor', 'w', 'Callback', @animate);
        
    info_str = sprintf([ ...
        'GEOMETRIC DATUMS:   • Origin Pivot Datum: J1 = (0.0, 0.0) mm   • Engine Centerline Axis: Y_center = %.1f mm   • Piston Rail Height: Y_J5 = %.2f mm   • Slider Rail Height: Y_J4 = %.2f mm\n' ...
        'KINEMATIC DATUMS:   • Piston Mid-Stroke Home: x1_0 = %.2f mm (Range: [%.2f, %.2f] mm)   • Slider Neutral Home: x2_0 = %.2f mm (Range: [%.2f, %.2f] mm)   • Home Area Ratio: A_t/A_e = %.3f'], ...
        y_center, y_rail_J5, y_rail_J4, ...
        x1_home_datum, x_min_stroke, x_max_stroke, J4_x0, r_min, r_max, ar_home_datum);
    
    uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.04 0.012 0.76 0.056], ...
        'String', info_str, 'FontSize', 8.5, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.14 0.14 0.16], 'ForegroundColor', [0.92 0.92 0.92], ...
        'HorizontalAlignment', 'left');

    current_theta_guess = theta1_min_datum;
    updateKinematics(current_theta_guess, r_vec_pre(1));
    drawnow;

    %% ============================================================
    % HELPER: COMPUTE AREA RATIO (A_throat / A_outlet)
    %% ============================================================
    function ar = computeAreaRatio(J2_pt, J_end_pt)
        R_throat = max(0.5, J2_pt(2) - y_center);
        R_outlet = max(0.5, J_end_pt(2) - y_center);
        
        if is_axisymmetric
            ar = (R_throat / R_outlet)^2;
        else
            ar = R_throat / R_outlet;
        end
    end

    %% ============================================================
    % SOLVER 1: PISTON X (x1) TO THETA1
    %% ============================================================
    function piston_x = getPistonX(theta_test)
        J2_tmp = J1 + L1 * [cos(theta_test), sin(theta_test)];
        u_L1_tmp = (J2_tmp - J1) / L1;
        n_L1_tmp = [-u_L1_tmp(2), u_L1_tmp(1)];
        rail_func = @(t) J1(2) + t*(J2_tmp(2)-J1(2)) + (bulge_amount * sin(pi * t) + R5) * n_L1_tmp(2) - y_rail_J5;
        options_fzero = optimset('Display','off');
        t_contact = fzero(rail_func, 0.5, options_fzero);
        J5_surface = J1 + t_contact * (J2_tmp - J1) + (bulge_amount * sin(pi * t_contact)) * n_L1_tmp;
        piston_x = J5_surface(1) + R5 * n_L1_tmp(1);
    end

    %% ============================================================
    % HELPER: SOLVE J3 POSITION
    %% ============================================================
    function J3 = solveJ3(J2in, J4in)
        V_link = J4in - J2in; d = norm(V_link);
        if d > (L2 + L3) || d < abs(L2 - L3)
            J3 = [NaN, NaN]; return;
        end
        a = (L2^2 - L3^2 + d^2) / (2 * d);
        h = sqrt(max(0, L2^2 - a^2));
        e = V_link / d; ep = [-e(2), e(1)];
        P1 = J2in + a*e + h*ep; P2 = J2in + a*e - h*ep;
        if P1(1) > P2(1), J3 = P1; else, J3 = P2; end
    end

    %% ============================================================
    % SOLVER 2: GRAPHICS UPDATE (FORWARD KINEMATICS)
    %% ============================================================
    function [J_end_pos, ar_now] = updateKinematics(theta1, r)
        J2 = J1 + L1 * [cos(theta1), sin(theta1)];
        J4 = [r, y_rail_J4];
        J3 = solveJ3(J2, J4);
        if any(isnan(J3))
            J_end_pos = [NaN, NaN]; ar_now = NaN; return;
        end
        
        theta2 = atan2(J3(2) - J2(2), J3(1) - J2(1));
        theta_ext = theta2 + petal_bend;
        J_end = J3 + L2_ext * [cos(theta_ext), sin(theta_ext)];
        J_end_pos = J_end;
        
        ar_now = computeAreaRatio(J2, J_end);
        R_throat_now = J2(2) - y_center;
        R_outlet_now = J_end(2) - y_center;
        
        u_L1 = (J2 - J1) / L1;
        n_L1 = [-u_L1(2), u_L1(1)];
        t_search = linspace(0.01, 0.99, 400);
        J5_search_y = J1(2) + t_search.*(J2(2)-J1(2)) + (bulge_amount .* sin(pi .* t_search) + R5) .* n_L1(2);
        [~, min_idx] = min(abs(J5_search_y - y_rail_J5));
        t_contact = t_search(min_idx);
        J5_surface = J1 + t_contact * (J2 - J1) + (bulge_amount * sin(pi * t_contact)) * n_L1;
        J5 = J5_surface + R5 * n_L1;
        J5(2) = y_rail_J5;
        
        t_cam = linspace(0, 1, 30);
        cam_curve = zeros(2, 30);
        for i = 1:30
            pt = J1 + t_cam(i) * (J2 - J1);
            bulge = bulge_amount * sin(pi * t_cam(i));
            cam_curve(:, i) = (pt + bulge * n_L1)';
        end
        L1_X = [J1(1), cam_curve(1,:), J2(1)];
        L1_Y = [J1(2), cam_curve(2,:), J2(2)];
        
        hole_fractions = [0.25, 0.50, 0.75];
        hole_radius = 0.5;
        
        set(hRod, 'XData', [cyl_x + cyl_w, J5(1)], 'YData', [y_rail_J5, y_rail_J5]);
        set(hL1_patch, 'XData', L1_X, 'YData', L1_Y);
        for idx = 1:3
            center_pt = J1 + hole_fractions(idx) * (J2 - J1) + (bulge_amount/2 * sin(pi * hole_fractions(idx))) * n_L1;
            th = linspace(0, 2*pi, 20);
            hx = center_pt(1) + hole_radius * cos(th);
            hy = center_pt(2) + hole_radius * sin(th);
            set(hL1_holes(idx), 'XData', hx, 'YData', hy);
        end
        
        set(hL2, 'XData', [J2(1), J3(1), J_end(1), J2(1)], 'YData', [J2(2), J3(2), J_end(2), J2(2)]);
        set(hL3, 'XData', [J4(1) J3(1)], 'YData', [J4(2) J3(2)]);
        set(hJ4, 'Position', [J4(1)-R6*0.9, J4(2)-R6*0.9, 2*R6*0.9, 2*R6*0.9]);
        set(hSlider4, 'XData', J4(1) + [-slider_w/2 slider_w/2 slider_w/2 -slider_w/2], ...
                      'YData', J4(2) + [-slider_h/2 -slider_h/2 slider_h/2 slider_h/2]);
        set(hJ5, 'Position', [J5(1)-R5, J5(2)-R5, 2*R5, 2*R5]);
        set(hJ_end, 'XData', J_end(1), 'YData', J_end(2));
        set(hJoints, 'XData', [J1(1) J2(1) J3(1) J5(1)], ...
                     'YData', [J1(2) J2(2) J3(2) J5(2)]);
                 
        set(hThroatLine, 'XData', [J2(1) J2(1)], 'YData', [y_center J2(2)]);
        set(hOutletLine, 'XData', [J_end(1) J_end(1)], 'YData', [y_center J_end(2)]);
        set(hRtText, 'Position', [J2(1)+0.2, (y_center + J2(2))/2, 0], 'String', sprintf('R_t=%.2f', R_throat_now));
        set(hReText, 'Position', [J_end(1)+0.2, (y_center + J_end(2))/2, 0], 'String', sprintf('R_e=%.2f', R_outlet_now));
        
        set(hLabels.J2, 'Position', [J2(1)-1.2, J2(2)-0.55, 0]);
        set(hLabels.J3, 'Position', [J3(1)+0.2, J3(2)+0.35, 0]);
        set(hLabels.J4, 'Position', [J4(1)-0.8, J4(2)-0.7, 0]);
        set(hLabels.J5, 'Position', [J5(1)-0.6, J5(2)+0.6, 0]);
        set(hLabels.J_end, 'Position', [J_end(1)+0.3, J_end(2)-0.3, 0]);
        
        % Calculate instantaneous displacements relative to Kinematic Home Datums
        dx1_datum = J5(1) - x1_home_datum;
        dx2_datum = J4(1) - J4_x0;
        
        if ar_now < 1.0
            regime_str = 'SUPERSONIC C-D';
        else
            regime_str = 'CONVERGENT PINCH';
        end
        
        hud_text = sprintf([ ...
            'LIVE STATE & DATUM OFFSETS\n' ...
            '------------------------------\n' ...
            'Piston x1     : %6.2f mm\n' ...
            '  vs Home x1_0: %+6.2f mm\n' ...
            'Slider x2     : %6.2f mm\n' ...
            '  vs Home x2_0: %+6.2f mm\n' ...
            'Crank theta1  : %6.1f deg\n' ...
            'Throat Rad Rt : %6.2f mm\n' ...
            'Outlet Rad Re : %6.2f mm\n' ...
            'A_throat/A_out: %6.3f\n' ...
            'Mode: %s'], ...
            J5(1), dx1_datum, J4(1), dx2_datum, rad2deg(theta1), ...
            R_throat_now, R_outlet_now, ar_now, regime_str);
        set(hHudBox, 'String', hud_text);
    end

    %% ============================================================
    % ANIMATION LOOP
    %% ============================================================
    function animate(~,~)
        set(hTraj, 'XData', [], 'YData', []);
        traj_x = []; traj_y = [];
        N = 100;
        linear_stroke_out = linspace(x_min_stroke, x_max_stroke, N);
        linear_stroke_in  = fliplr(linear_stroke_out);
        full_cycle = [linear_stroke_out, linear_stroke_in];
        Nfull = length(full_cycle);
        r_cycle = J4_x0 + r_amp * sin(linspace(0, 2*pi, Nfull));
        opt = optimset('Display','off');
        
        for k = 1:Nfull
            if ~isvalid(fig), break; end
            target_piston_x = full_cycle(k);
            objective_func = @(th) getPistonX(th) - target_piston_x;
            solved_theta = fzero(objective_func, current_theta_guess, opt);
            current_theta_guess = solved_theta;
            r_now = r_cycle(k);
            
            [curr_tip, ar_val] = updateKinematics(solved_theta, r_now);
                        
            set(hMarker3D, 'XData', target_piston_x, 'YData', r_now, 'ZData', ar_val);
            set(hDropLine3D, 'XData', [target_piston_x target_piston_x], ...
                             'YData', [r_now r_now], ...
                             'ZData', [z_floor ar_val]);
            
            if ~isnan(curr_tip(1))
                traj_x(end+1) = curr_tip(1); %#ok<AGROW>
                traj_y(end+1) = curr_tip(2); %#ok<AGROW>
                set(hTraj, 'XData', traj_x, 'YData', traj_y);
            end
            drawnow;
            pause(0.02);
        end
    end
end