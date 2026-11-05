function simulation_jet_nozzle_linear_input()
    clc; clear; close all;
    
    %% ============================================================
    % LINK GEOMETRY & PARAMETERS
    %% ============================================================
    L1 = 5.82;      
    L2 = 6.22;      
    L2_ext = 4.5;   
    L3 = 9.07;      
    L6 = 3.55;      
    theta6 = deg2rad(34.3); 
    petal_bend = deg2rad(-15); 
    
    J1 = [0, 0];
    J4 = L6 * [cos(theta6), sin(theta6)];
    
    R5 = 0.42;      
    R6 = 0.32;      
    bulge_amount = 1.8; 
    
    y_rail_J5 = -0.5; 
    dX_carrier = 2.0; 
    
    cyl_w = 2.5; cyl_h = 1.0; 
    cyl_x = -3.5; cyl_y = y_rail_J5 - cyl_h/2;
    
    %% ============================================================
    % FIGURE SETUP
    %% ============================================================
    fig = figure('Name','Jet Engine CD Nozzle - Linear Piston Input',...
        'NumberTitle','off','Color',[0.1 0.1 0.1],'Position',[100 80 1250 720]);
    ax = axes('Parent',fig,'Position',[0.06 0.12 0.88 0.82],'Color',[0.06 0.06 0.06]);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    set(ax, 'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], 'GridColor', [0.25 0.25 0.25]);
    
    xlim(ax,[-4 18]); ylim(ax,[-7 5]);
    xlabel(ax,'X (Linear Displacement Axis)'); ylabel(ax,'Y');
    title(ax,'Jet Engine Nozzle: Driven by Linear Piston Displacement','Color','w','FontSize',13);
    
    %% ============================================================
    % GRAPHICAL ELEMENTS
    %% ============================================================
    hL6 = line(ax, [J1(1) J4(1)], [J1(2) J4(2)], 'LineWidth', 4, 'Color', [0.5 0.5 0.5]); 
    hL1_patch = patch(ax, 'XData', [], 'YData', [], 'FaceColor', [0.85 0.3 0.2], 'EdgeColor', 'w', 'LineWidth', 1.5, 'FaceAlpha', 0.9);
    hL1_holes = gobjects(1,3);
    for idx = 1:3
        hL1_holes(idx) = patch(ax, 'XData', [], 'YData', [], 'FaceColor', [0.06 0.06 0.06], 'EdgeColor', 'w', 'LineWidth', 1);
    end
    hL2 = line(ax, [0 0 0 0], [0 0 0 0], 'LineWidth', 8, 'Color', [0.2 0.7 0.3]);               
    hL3 = line(ax, [0 0], [0 0], 'LineWidth', 7, 'Color', [0.2 0.5 0.9]);               
    patch(ax, 'XData', [cyl_x cyl_x+cyl_w cyl_x+cyl_w cyl_x], ...
              'YData', [cyl_y cyl_y cyl_y+cyl_h cyl_y+cyl_h], ...
              'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'w', 'LineWidth', 1.5);
    hRod = line(ax, [0 0], [0 0], 'LineWidth', 6, 'Color', [0.75 0.75 0.75]);
    hRail = line(ax, [-1 9], [y_rail_J5 y_rail_J5], 'Color', [0.8 0.8 0.2], 'LineStyle', '--', 'LineWidth', 1.5);
    hCarrier = line(ax, [0 0], [0 0], 'LineWidth', 4, 'Color', 'w');
    hTraj = line(ax, NaN, NaN, 'Color', [0 1 1], 'LineStyle', '--', 'LineWidth', 2);
    hJ5 = rectangle(ax, 'Position', [0 0 0 0], 'Curvature', [1 1], 'FaceColor', [0.95 0.75 0.1], 'EdgeColor', 'w');
    hJ6 = rectangle(ax, 'Position', [0 0 0 0], 'Curvature', [1 1], 'FaceColor', [0.95 0.75 0.1], 'EdgeColor', 'w');
    hJ_end = plot(ax, 0, 0, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', [0.2 0.7 0.3], 'LineWidth', 1.5);
    hJoints = plot(ax, zeros(1,6), zeros(1,6), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'w', 'LineWidth', 1.5);
    
    hLabels.J1 = text(ax, J1(1)-0.4, J1(2)+0.35, 'J1', 'Color', 'w', 'FontWeight', 'bold');
    hLabels.J2 = text(ax, 0, 0, 'J2', 'Color', 'w', 'FontWeight', 'bold');
    hLabels.J3 = text(ax, 0, 0, 'J3', 'Color', 'w', 'FontWeight', 'bold');
    hLabels.J4 = text(ax, J4(1)-0.3, J4(2)+0.35, 'J4', 'Color', 'w', 'FontWeight', 'bold');
    hLabels.J5 = text(ax, 0, 0, 'J5', 'Color', 'w', 'FontWeight', 'bold');
    hLabels.J6 = text(ax, 0, 0, 'J6', 'Color', 'w', 'FontWeight', 'bold');
    hLabels.J7 = text(ax, cyl_x + 0.3, cyl_y + 1.4, 'J7 (Piston)', 'Color', 'w', 'FontWeight', 'bold');
    hLabels.J_end = text(ax, 0, 0, '  Exit Tip', 'Color', 'w', 'FontWeight', 'bold');
    
    hPistonLabel = text(ax, 10, 4, 'Piston X: ', 'Color', [0.8 0.8 0.2], 'FontSize', 12, 'FontWeight', 'bold');
    
    % Controls
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.15 0.03 0.15 0.05], ...
        'String', 'ANIMATE', 'FontSize', 10, 'FontWeight', 'bold', 'Callback', @animate);
    
    x_min_stroke = getPistonX(deg2rad(-50.0));
    x_max_stroke = getPistonX(deg2rad(-10.0));
    
    current_theta_guess = deg2rad(-50.0);
    updateKinematics(current_theta_guess);
    
    %% ============================================================
    % SOLVER 1: PISTON X TO THETA1 (NUMERICAL ROOT FINDER)
    %% ============================================================
    function piston_x = getPistonX(theta_test)
        J2_tmp = J1 + L1 * [cos(theta_test), sin(theta_test)];
        u_L1_tmp = (J2_tmp - J1) / L1;
        n_L1_tmp = [-u_L1_tmp(2), u_L1_tmp(1)]; 
        
        t_search = linspace(0.01, 0.99, 400);
        J5_search_y = J1(2) + t_search.*(J2_tmp(2)-J1(2)) + (bulge_amount .* sin(pi .* t_search) + R5) .* n_L1_tmp(2);
        
        [~, min_idx] = min(abs(J5_search_y - y_rail_J5));
        t_contact = t_search(min_idx);
        
        J5_surface = J1 + t_contact * (J2_tmp - J1) + (bulge_amount * sin(pi * t_contact)) * n_L1_tmp;
        piston_x = J5_surface(1) + R5 * n_L1_tmp(1);
    end

    %% ============================================================
    % SOLVER 2: GRAPHICS UPDATE (FORWARD KINEMATICS)
    %% ============================================================
    function J_end_pos = updateKinematics(theta1)
        J2 = J1 + L1 * [cos(theta1), sin(theta1)];
        V = J4 - J2; d = norm(V);
        if d > (L2 + L3) || d < abs(L2 - L3)
            J_end_pos = [NaN, NaN]; return; 
        end
        a = (L2^2 - L3^2 + d^2) / (2 * d);
        h = sqrt(max(0, L2^2 - a^2));
        e = V / d; ep = [-e(2), e(1)];
        P1 = J2 + a*e + h*ep; P2 = J2 + a*e - h*ep;
        if P1(1) > P2(1), J3 = P1; else, J3 = P2; end
        
        theta2 = atan2(J3(2) - J2(2), J3(1) - J2(1)); 
        theta_ext = theta2 + petal_bend; 
        
        J_end = J3 + L2_ext * [cos(theta_ext), sin(theta_ext)];
        J_end_pos = J_end; 
        
        u_L1 = (J2 - J1) / L1;
        n_L1 = [-u_L1(2), u_L1(1)];
        u_L3 = (J3 - J4) / L3;
        n_L3 = [-u_L3(2), u_L3(1)]; 
        
        t_search = linspace(0.01, 0.99, 400);
        J5_search_y = J1(2) + t_search.*(J2(2)-J1(2)) + (bulge_amount .* sin(pi .* t_search) + R5) .* n_L1(2);
        [~, min_idx] = min(abs(J5_search_y - y_rail_J5));
        t_contact = t_search(min_idx);
        J5_surface = J1 + t_contact * (J2 - J1) + (bulge_amount * sin(pi * t_contact)) * n_L1;
        J5 = J5_surface + R5 * n_L1;
        J5(2) = y_rail_J5; 
        
        J6_x = J5(1) + dX_carrier;
        L3_offset_start = J4 + R6 * n_L3;
        if abs(u_L3(1)) > 1e-4
            s = (J6_x - L3_offset_start(1)) / u_L3(1);
            J6_y = L3_offset_start(2) + s * u_L3(2);
        else
            J6_y = J5(2) + 0.5;
        end
        J6 = [J6_x, J6_y];
        
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
        
        % Visual Updates
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
        set(hCarrier, 'XData', [J5(1) J6(1)], 'YData', [J5(2) J6(2)]);
        set(hJ5, 'Position', [J5(1)-R5, J5(2)-R5, 2*R5, 2*R5]);
        set(hJ6, 'Position', [J6(1)-R6, J6(2)-R6, 2*R6, 2*R6]);
        set(hJ_end, 'XData', J_end(1), 'YData', J_end(2));
        set(hJoints, 'XData', [J1(1) J2(1) J3(1) J4(1) J5(1) J6(1)], ...
                     'YData', [J1(2) J2(2) J3(2) J4(2) J5(2) J6(2)]);
        
        set(hLabels.J2, 'Position', [J2(1)-0.1, J2(2)-0.4, 0]);
        set(hLabels.J3, 'Position', [J3(1)+0.2, J3(2)+0.3, 0]);
        set(hLabels.J5, 'Position', [J5(1)-0.6, J5(2)-0.5, 0]);
        set(hLabels.J6, 'Position', [J6(1)+0.3, J6(2)+0.3, 0]);
        set(hLabels.J_end, 'Position', [J_end(1)+0.3, J_end(2)-0.3, 0]);
        
        set(hPistonLabel, 'String', sprintf('Linear Piston Input (X): %.2f mm', J5(1)));
    end
    
    %% ============================================================
    % ANIMATION 
    %% ============================================================
    function animate(~,~)
        set(hTraj, 'XData', [], 'YData', []); 
        traj_x = []; traj_y = [];
        
        N = 100;
        linear_stroke_out = linspace(x_min_stroke, x_max_stroke, N);
        linear_stroke_in  = fliplr(linear_stroke_out);
        full_cycle = [linear_stroke_out, linear_stroke_in];
        
        opt = optimset('Display','off'); 
        
        for k = 1:length(full_cycle)
            if ~isvalid(fig), break; end
            
            target_piston_x = full_cycle(k);
            
            objective_func = @(th) getPistonX(th) - target_piston_x;
            solved_theta = fzero(objective_func, current_theta_guess, opt);
            
            current_theta_guess = solved_theta; 
            
            curr_tip = updateKinematics(solved_theta);
            
            if ~isnan(curr_tip(1))
                traj_x(end+1) = curr_tip(1);
                traj_y(end+1) = curr_tip(2);
                set(hTraj, 'XData', traj_x, 'YData', traj_y);
            end
            
            drawnow;
            pause(0.02); % Slows down the simulation speed (adjust higher if needed)
        end
    end
end