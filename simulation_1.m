function simulation_1
clc;
close all;

%% ============================================================
%              SIMPLE FOUR BAR MECHANISM
%
%                    J4
%                    O
%                   / \
%                  /   \
%                 /     \
%                /       \
%           J1 O          O J3
%               \        /
%                \      /
%                 \    /
%                  O
%                 J2
%
% Link 1 : J1-J4 = Fixed link
% Link 2 : J1-J2
% Link 3 : J2-J3
% Link 4 : J3-J4
%
% Orientation kept similar to the reference image.
%% ============================================================


%% ============================================================
% 1. LINK LENGTHS
% Estimated from the reference image
%% ============================================================

L1 = 100;       % J1-J4 : Fixed link
L2 = 142;       % J1-J2
L3 = 166;       % J2-J3
L4 = 205;       % J3-J4


%% ============================================================
% 2. FIXED LINK
%
% J1 is lower-left
% J4 is upper-right
%% ============================================================

theta1 = deg2rad(34);

J1 = [0 0];

J4 = J1 + L1 * ...
    [cos(theta1), sin(theta1)];


%% ============================================================
% 3. INITIAL INPUT ANGLE
%
% J1 -> J2 points down and to the right,
% similar to the reference figure.
%% ============================================================

theta2_initial = deg2rad(-37);


%% ============================================================
% 4. INITIAL POSITION OF J2
%% ============================================================

J2 = J1 + L2 * ...
    [cos(theta2_initial), ...
     sin(theta2_initial)];


%% ============================================================
% 5. FIND INITIAL J3
%% ============================================================

[J3a,J3b,valid] = circleIntersection( ...
    J2,L3,J4,L4);


if ~valid
    error('Initial configuration is not possible.');
end


%% ------------------------------------------------------------
% Select the right-hand assembly configuration
%% ------------------------------------------------------------

if J3a(1) > J3b(1)

    J3 = J3a;

else

    J3 = J3b;

end


% Previous J3 is used to maintain the same assembly branch
previousJ3 = J3;


%% ============================================================
% 6. CREATE FIGURE
%% ============================================================

fig = figure( ...
    'Name','Simple Four Bar Mechanism', ...
    'NumberTitle','off', ...
    'Color','w', ...
    'Position',[100 100 1100 700]);


ax = axes( ...
    'Parent',fig, ...
    'Position',[0.08 0.15 0.82 0.75]);


hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');


xlabel(ax,'X (mm)');
ylabel(ax,'Y (mm)');


title(ax,'Simple Four-Bar Mechanism');


%% ============================================================
% 7. FIXED SUPPORTS
%% ============================================================

plot(ax, ...
    J1(1),J1(2), ...
    'ko', ...
    'MarkerSize',14, ...
    'MarkerFaceColor','k');


plot(ax, ...
    J4(1),J4(2), ...
    'ko', ...
    'MarkerSize',14, ...
    'MarkerFaceColor','k');


%% ============================================================
% 8. FIXED LINK - J1 TO J4
%% ============================================================

hLink1 = plot(ax, ...
    [J1(1),J4(1)], ...
    [J1(2),J4(2)], ...
    'k-', ...
    'LineWidth',7);


%% ============================================================
% 9. MOVING LINK 2 - J1 TO J2
%% ============================================================

hLink2 = plot(ax, ...
    [J1(1),J2(1)], ...
    [J1(2),J2(2)], ...
    'LineWidth',7);


%% ============================================================
% 10. MOVING LINK 3 - J2 TO J3
%% ============================================================

hLink3 = plot(ax, ...
    [J2(1),J3(1)], ...
    [J2(2),J3(2)], ...
    'LineWidth',7);


%% ============================================================
% 11. MOVING LINK 4 - J3 TO J4
%% ============================================================

hLink4 = plot(ax, ...
    [J3(1),J4(1)], ...
    [J3(2),J4(2)], ...
    'LineWidth',7);


%% ============================================================
% 12. JOINTS
%% ============================================================

hJ1 = plot(ax, ...
    J1(1),J1(2), ...
    'ko', ...
    'MarkerSize',11, ...
    'MarkerFaceColor','k');


hJ2 = plot(ax, ...
    J2(1),J2(2), ...
    'ko', ...
    'MarkerSize',11, ...
    'MarkerFaceColor','k');


hJ3 = plot(ax, ...
    J3(1),J3(2), ...
    'ko', ...
    'MarkerSize',11, ...
    'MarkerFaceColor','k');


hJ4 = plot(ax, ...
    J4(1),J4(2), ...
    'ko', ...
    'MarkerSize',11, ...
    'MarkerFaceColor','k');


%% ============================================================
% 13. JOINT LABELS
%% ============================================================

hJ1Text = text(ax, ...
    J1(1)-15, ...
    J1(2)+10, ...
    'J1', ...
    'FontSize',13, ...
    'FontWeight','bold');


hJ2Text = text(ax, ...
    J2(1)-5, ...
    J2(2)-15, ...
    'J2', ...
    'FontSize',13, ...
    'FontWeight','bold');


hJ3Text = text(ax, ...
    J3(1)+7, ...
    J3(2)+10, ...
    'J3', ...
    'FontSize',13, ...
    'FontWeight','bold');


hJ4Text = text(ax, ...
    J4(1)+5, ...
    J4(2)+10, ...
    'J4', ...
    'FontSize',13, ...
    'FontWeight','bold');


%% ============================================================
% 14. LINK LABELS
%% ============================================================

hL1Text = text(ax, ...
    (J1(1)+J4(1))/2-20, ...
    (J1(2)+J4(2))/2+8, ...
    'Link 1 (Fixed)', ...
    'FontSize',10, ...
    'FontWeight','bold');


hL2Text = text(ax, ...
    (J1(1)+J2(1))/2-10, ...
    (J1(2)+J2(2))/2, ...
    'Link 2', ...
    'FontSize',10, ...
    'FontWeight','bold');


hL3Text = text(ax, ...
    (J2(1)+J3(1))/2, ...
    (J2(2)+J3(2))/2+10, ...
    'Link 3', ...
    'FontSize',10, ...
    'FontWeight','bold');


hL4Text = text(ax, ...
    (J3(1)+J4(1))/2+10, ...
    (J3(2)+J4(2))/2, ...
    'Link 4', ...
    'FontSize',10, ...
    'FontWeight','bold');


%% ============================================================
% 15. TRACE OF J2
%% ============================================================

hTraceJ2 = plot(ax, ...
    J2(1),J2(2), ...
    '--', ...
    'LineWidth',1);


%% ============================================================
% 16. TRACE OF J3
%% ============================================================

hTraceJ3 = plot(ax, ...
    J3(1),J3(2), ...
    '--', ...
    'LineWidth',1);


%% ============================================================
% 17. AXIS LIMITS
%% ============================================================

xlim(ax,[-100 350]);

ylim(ax,[-230 150]);


%% ============================================================
% 18. SLIDER
%% ============================================================

slider = uicontrol( ...
    'Parent',fig, ...
    'Style','slider', ...
    'Units','normalized', ...
    'Position',[0.20 0.055 0.50 0.045], ...
    'Min',-180, ...
    'Max',180, ...
    'Value',rad2deg(theta2_initial));


%% ============================================================
% 19. ANGLE DISPLAY
%% ============================================================

angleText = uicontrol( ...
    'Parent',fig, ...
    'Style','text', ...
    'Units','normalized', ...
    'Position',[0.35 0.095 0.30 0.035], ...
    'String',sprintf( ...
        'Input Angle = %.1f°', ...
        rad2deg(theta2_initial)), ...
    'FontSize',11, ...
    'FontWeight','bold', ...
    'BackgroundColor','w');


%% ============================================================
% 20. ANIMATE BUTTON
%% ============================================================

animateButton = uicontrol( ...
    'Parent',fig, ...
    'Style','pushbutton', ...
    'Units','normalized', ...
    'Position',[0.74 0.045 0.12 0.055], ...
    'String','Animate', ...
    'FontSize',11);


%% ============================================================
% 21. RESET BUTTON
%% ============================================================

resetButton = uicontrol( ...
    'Parent',fig, ...
    'Style','pushbutton', ...
    'Units','normalized', ...
    'Position',[0.88 0.045 0.10 0.055], ...
    'String','Reset', ...
    'FontSize',11);


%% ============================================================
% 22. CALLBACKS
%% ============================================================

slider.Callback = @sliderCallback;

animateButton.Callback = @animateCallback;

resetButton.Callback = @resetCallback;


%% ============================================================
% 23. SLIDER CALLBACK
%% ============================================================

    function sliderCallback(src,~)

        angleDeg = src.Value;

        updateMechanism(angleDeg);

    end


%% ============================================================
% 24. ANIMATION
%% ============================================================

    function animateCallback(~,~)

        % Start from the reference orientation
        angles = linspace(-37,143,400);


        for i = 1:length(angles)

            if ~ishandle(fig)

                return;

            end


            set(slider, ...
                'Value',angles(i));


            success = updateMechanism(angles(i));


            % Stop if configuration becomes impossible
            if ~success

                break;

            end


            pause(0.02);

        end

    end


%% ============================================================
% 25. RESET
%% ============================================================

    function resetCallback(~,~)

        set(slider, ...
            'Value',rad2deg(theta2_initial));


        % Reset branch
        previousJ3 = J3;


        % Reset traces
        set(hTraceJ2, ...
            'XData',J2(1), ...
            'YData',J2(2));


        set(hTraceJ3, ...
            'XData',J3(1), ...
            'YData',J3(2));


        updateMechanism( ...
            rad2deg(theta2_initial));

    end


%% ============================================================
% 26. UPDATE MECHANISM
%% ============================================================

    function success = updateMechanism(angleDeg)

        theta2 = deg2rad(angleDeg);


        %% ----------------------------------------------------
        % Calculate J2
        %% ----------------------------------------------------

        J2new = J1 + L2 * ...
            [cos(theta2),sin(theta2)];


        %% ----------------------------------------------------
        % Calculate J3
        %
        % J3 must satisfy:
        %
        % |J3-J2| = L3
        % |J3-J4| = L4
        %% ----------------------------------------------------

        [A,B,valid] = circleIntersection( ...
            J2new,L3,J4,L4);


        %% ----------------------------------------------------
        % If no solution exists
        %% ----------------------------------------------------

        if ~valid

            success = false;

            set(angleText, ...
                'String',sprintf( ...
                'Configuration limit at %.1f°', ...
                angleDeg));


            return;

        end


        %% ----------------------------------------------------
        % Select solution closest to previous J3
        %
        % This prevents sudden jumping between assembly modes.
        %% ----------------------------------------------------

        if norm(A-previousJ3) <= norm(B-previousJ3)

            J3new = A;

        else

            J3new = B;

        end


        previousJ3 = J3new;


        %% ----------------------------------------------------
        % Update Link 2
        %% ----------------------------------------------------

        set(hLink2, ...
            'XData',[J1(1),J2new(1)], ...
            'YData',[J1(2),J2new(2)]);


        %% ----------------------------------------------------
        % Update Link 3
        %% ----------------------------------------------------

        set(hLink3, ...
            'XData',[J2new(1),J3new(1)], ...
            'YData',[J2new(2),J3new(2)]);


        %% ----------------------------------------------------
        % Update Link 4
        %% ----------------------------------------------------

        set(hLink4, ...
            'XData',[J3new(1),J4(1)], ...
            'YData',[J3new(2),J4(2)]);


        %% ----------------------------------------------------
        % Update J2
        %% ----------------------------------------------------

        set(hJ2, ...
            'XData',J2new(1), ...
            'YData',J2new(2));


        %% ----------------------------------------------------
        % Update J3
        %% ----------------------------------------------------

        set(hJ3, ...
            'XData',J3new(1), ...
            'YData',J3new(2));


        %% ----------------------------------------------------
        % Update J2 label
        %% ----------------------------------------------------

        set(hJ2Text, ...
            'Position',[ ...
            J2new(1)-5, ...
            J2new(2)-15, ...
            0]);


        %% ----------------------------------------------------
        % Update J3 label
        %% ----------------------------------------------------

        set(hJ3Text, ...
            'Position',[ ...
            J3new(1)+7, ...
            J3new(2)+10, ...
            0]);


        %% ----------------------------------------------------
        % Update Link 2 label
        %% ----------------------------------------------------

        set(hL2Text, ...
            'Position',[ ...
            (J1(1)+J2new(1))/2-10, ...
            (J1(2)+J2new(2))/2, ...
            0]);


        %% ----------------------------------------------------
        % Update Link 3 label
        %% ----------------------------------------------------

        set(hL3Text, ...
            'Position',[ ...
            (J2new(1)+J3new(1))/2, ...
            (J2new(2)+J3new(2))/2+10, ...
            0]);


        %% ----------------------------------------------------
        % Update Link 4 label
        %% ----------------------------------------------------

        set(hL4Text, ...
            'Position',[ ...
            (J3new(1)+J4(1))/2+10, ...
            (J3new(2)+J4(2))/2, ...
            0]);


        %% ----------------------------------------------------
        % J2 trajectory
        %% ----------------------------------------------------

        x2 = get(hTraceJ2,'XData');

        y2 = get(hTraceJ2,'YData');


        if length(x2) > 800

            x2 = x2(end-400:end);

            y2 = y2(end-400:end);

        end


        set(hTraceJ2, ...
            'XData',[x2,J2new(1)], ...
            'YData',[y2,J2new(2)]);


        %% ----------------------------------------------------
        % J3 trajectory
        %% ----------------------------------------------------

        x3 = get(hTraceJ3,'XData');

        y3 = get(hTraceJ3,'YData');


        if length(x3) > 800

            x3 = x3(end-400:end);

            y3 = y3(end-400:end);

        end


        set(hTraceJ3, ...
            'XData',[x3,J3new(1)], ...
            'YData',[y3,J3new(2)]);


        %% ----------------------------------------------------
        % Update angle display
        %% ----------------------------------------------------

        set(angleText, ...
            'String',sprintf( ...
            'Input Angle = %.1f°', ...
            angleDeg));


        drawnow;


        success = true;

    end


%% ============================================================
% 27. CIRCLE INTERSECTION
%% ============================================================

    function [P1,P2,valid] = circleIntersection(C1,r1,C2,r2)

        d = norm(C2-C1);


        P1 = [NaN NaN];

        P2 = [NaN NaN];

        valid = true;


        %% ----------------------------------------------------
        % No intersection
        %% ----------------------------------------------------

        if d > r1+r2+1e-10

            valid = false;

            return;

        end


        %% ----------------------------------------------------
        % One circle completely inside the other
        %% ----------------------------------------------------

        if d < abs(r1-r2)-1e-10

            valid = false;

            return;

        end


        %% ----------------------------------------------------
        % Same center
        %% ----------------------------------------------------

        if d < 1e-10

            valid = false;

            return;

        end


        %% ----------------------------------------------------
        % Distance from C1 to intersection chord
        %% ----------------------------------------------------

        a = (r1^2-r2^2+d^2)/(2*d);


        %% ----------------------------------------------------
        % Height
        %% ----------------------------------------------------

        h2 = r1^2-a^2;


        if h2 < -1e-8

            valid = false;

            return;

        end


        h = sqrt(max(h2,0));


        %% ----------------------------------------------------
        % Point on line joining centers
        %% ----------------------------------------------------

        P = C1 + ...
            a*(C2-C1)/d;


        %% ----------------------------------------------------
        % Perpendicular vector
        %% ----------------------------------------------------

        v = C2-C1;

        perp = [-v(2),v(1)]/d;


        %% ----------------------------------------------------
        % Two possible intersection points
        %% ----------------------------------------------------

        P1 = P+h*perp;

        P2 = P-h*perp;

    end

end