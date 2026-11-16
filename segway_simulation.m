%% SEGWAY_SIMULATION
% Three-coordinate Segway model based on the Lagrangian equations.
% Coordinates: q = [theta_w_left; theta_w_right; theta_b]
% The wheels are assumed to roll without slipping, and the body has no roll DOF.
%
% IMPORTANT:
% The numerical parameters below are illustrative placeholders. Replace them
% with the measured mass, geometry, inertia and motor parameters of your robot.

clear; clc; close all;

%% 1. Physical parameters (SI units)
p.mb   = 2.0;        % body mass [kg]
p.mw   = 0.25;       % mass of EACH wheel [kg]
p.lw   = 0.050;      % wheel radius [m]
p.L    = 0.300;      % distance between wheel centres [m]
p.lb   = 0.150;      % axle-to-body-COM distance [m]
p.g    = 9.81;       % gravity [m/s^2]

% Body principal moments of inertia [kg*m^2]
p.Ibxx = 0.010;
p.Ibyy = 0.015;
p.Ibzz = 0.010;

% Wheel moments of inertia about the axes used in the derivation [kg*m^2]
p.Iwxx = 0.5*p.mw*p.lw^2;       % wheel spin axis (axle)
p.Iwzz = 0.25*p.mw*p.lw^2;      % thin-disk in-plane diameter axis

% DC motor parameters
p.Kt = 0.050;       % torque constant [N*m/A]
p.Ke = 0.050;       % back-EMF constant [V*s/rad]
p.Rm = 2.0;         % winding resistance [ohm]
p.bf = 0.001;       % viscous mechanical friction [N*m*s/rad]

%% 2. Initial conditions
% State vector:
% y = [theta_wL theta_wR theta_b omega_wL omega_wR omega_b x y phi]'
theta_wL_0 = 0;
theta_wR_0 = 0;
theta_b_0  = deg2rad(5);  % initial body lean: 5 degrees
omega_wL_0 = 0;
omega_wR_0 = 0;
omega_b_0  = 0;

x_0   = 0;       % initial axle midpoint x-position [m]
y_0   = 0;       % initial axle midpoint y-position [m]
phi_0 = 0;       % initial heading [rad]

y0 = [theta_wL_0; theta_wR_0; theta_b_0; ...
      omega_wL_0; omega_wR_0; omega_b_0; x_0; y_0; phi_0];

%% 3. Motor voltage inputs
% These are open-loop inputs. With zero voltage and an initial lean, the
% robot will generally fall. Change them to test other motor inputs.
vLeft  = 5;      % left motor voltage [V]
vRight = 0;      % right motor voltage [V]

% Example: a constant voltage pulse can be implemented in motorVoltages()
% below. The default here is zero volts for both motors.


%% 4. LIVE 3D SEGWAY SIMULATION
% Integrate the equations and update the 3D figure while the simulation runs.
% No video is created or saved.

clearLive = true;

if clearLive
    close all;
end

%% Simulation settings

simDuration = 5;      % simulation duration [s]
dt = 0.02;            % simulation update interval [s]

solverOptions = odeset( ...
    'RelTol',1e-7, ...
    'AbsTol',1e-9, ...
    'MaxStep',dt);

% Start from the initial conditions
tLive = 0;
yLive = y0;

%% Visualization dimensions

wheelRadius = p.lw;
wheelWidth = 0.045;

bodyLength = 0.28;
bodyWidth = 0.16;
bodyHeight = 0.25;

%% Create the 3D figure

fig = figure( ...
    'Name','Live 3D Segway Simulation', ...
    'Color','w', ...
    'Position',[100 100 700 500]);

ax = axes('Parent',fig);

hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
axis(ax,'vis3d');

xlabel(ax,'X [m]');
ylabel(ax,'Y [m]');
zlabel(ax,'Z [m]');

title(ax,'Live 3D Segway Simulation');

view(ax,35,25);
camproj(ax,'perspective');

xlim(ax,[-0.7 0.7]);
ylim(ax,[-0.7 0.7]);
zlim(ax,[0 0.8]);

%% Ground plane

[Xg,Yg] = meshgrid([-2 2],[-2 2]);

surf(ax,Xg,Yg,zeros(size(Xg)), ...
    'FaceColor',[0.85 0.85 0.85], ...
    'FaceAlpha',0.5, ...
    'EdgeColor','none');

%% Wheel geometry

N = 50;

angles = linspace(0,2*pi,N);

% Circular wheel faces
[TH,RR] = meshgrid(angles,linspace(0,wheelRadius,20));

% Outer tyre surface
[TH2,SS] = meshgrid(angles,[-wheelWidth/2 wheelWidth/2]);

%% Create wheel graphics objects

hTyre = gobjects(2,1);
hFace = gobjects(2,2);
hHub = gobjects(2,2);

for w = 1:2

    % Outer tyre
    hTyre(w) = surf(ax, ...
        nan(size(TH2)),nan(size(TH2)),nan(size(TH2)), ...
        'FaceColor',[0.05 0.05 0.05], ...
        'EdgeColor','none');

    % Two circular faces
    for side = 1:2

        hFace(w,side) = surf(ax, ...
            nan(size(TH)),nan(size(TH)),nan(size(TH)), ...
            'FaceColor',[0.15 0.15 0.15], ...
            'EdgeColor','none');

        % Small central hub
        hHub(w,side) = surf(ax, ...
            nan(size(TH)),nan(size(TH)),nan(size(TH)), ...
            'FaceColor',[0.65 0.65 0.68], ...
            'EdgeColor','none');
    end
end

%% Create body graphics objects

bodyFaces = [
    1 2 4 3
    5 6 8 7
    1 2 6 5
    2 4 8 6
    4 3 7 8
    3 1 5 7
];

hBody = patch(ax, ...
    'Vertices',zeros(8,3), ...
    'Faces',bodyFaces, ...
    'FaceColor',[0.05 0.35 0.85], ...
    'EdgeColor','k', ...
    'LineWidth',2);

%% Axle, frame and centre of mass

hAxle = plot3(ax,nan,nan,nan, ...
    'k','LineWidth',8);

hFrame = plot3(ax,nan,nan,nan, ...
    'Color',[0.5 0.5 0.5], ...
    'LineWidth',10);

hCOM = plot3(ax,nan,nan,nan,'ro', ...
    'MarkerFaceColor','r', ...
    'MarkerSize',8);

%% Simulation and animation loop

while tLive < simDuration && ishandle(fig)

    loopTimer = tic;

    % Integrate one time interval
    tNext = min(tLive+dt,simDuration);

    [~,Ystep] = ode45( ...
        @(t,y) segwayODE(t,y,p,vLeft,vRight), ...
        [tLive tNext],yLive,solverOptions);

    % New state
    yLive = Ystep(end,:).';
    tLive = tNext;

    %% Read the current state

    thetaL = yLive(1);
    thetaR = yLive(2);
    thetaB = yLive(3);

    phi = yLive(9);

    x = yLive(7);
    y = yLive(8);

    %% World directions

    forward = [cos(phi);sin(phi);0];

    lateral = [-sin(phi);cos(phi);0];

    vertical = [0;0;1];

    %% Axle midpoint and wheel centres

    axle = [x;y;wheelRadius];

    cL = axle + (p.L/2)*lateral;
    cR = axle - (p.L/2)*lateral;

    wheelCentres = [cL,cR];

    %% Update both wheels

    for w = 1:2

        c = wheelCentres(:,w);

        % Wheel faces
        for side = 1:2

            if side == 1
                offset = -wheelWidth/2;
            else
                offset = wheelWidth/2;
            end

            X = c(1) + RR.*cos(TH)*forward(1) ...
                      + RR.*sin(TH)*vertical(1) ...
                      + offset*lateral(1);

            Yw = c(2) + RR.*cos(TH)*forward(2) ...
                       + RR.*sin(TH)*vertical(2) ...
                       + offset*lateral(2);

            Zw = c(3) + RR.*cos(TH)*forward(3) ...
                       + RR.*sin(TH)*vertical(3) ...
                       + offset*lateral(3);

            set(hFace(w,side), ...
                'XData',X,'YData',Yw,'ZData',Zw);

            % Hub radius
            hubRadius = 0.012;

            Xhub = c(1) + hubRadius*cos(TH)*forward(1) ...
                         + hubRadius*sin(TH)*vertical(1) ...
                         + offset*lateral(1);

            Yhub = c(2) + hubRadius*cos(TH)*forward(2) ...
                         + hubRadius*sin(TH)*vertical(2) ...
                         + offset*lateral(2);

            Zhub = c(3) + hubRadius*cos(TH)*forward(3) ...
                         + hubRadius*sin(TH)*vertical(3) ...
                         + offset*lateral(3);

            set(hHub(w,side), ...
                'XData',Xhub,'YData',Yhub,'ZData',Zhub);
        end

        % Outer tyre surface
        X = c(1) + wheelRadius*cos(TH2)*forward(1) ...
                  + wheelRadius*sin(TH2)*vertical(1) ...
                  + SS*lateral(1);

        Yw = c(2) + wheelRadius*cos(TH2)*forward(2) ...
                   + wheelRadius*sin(TH2)*vertical(2) ...
                   + SS*lateral(2);

        Zw = c(3) + wheelRadius*cos(TH2)*forward(3) ...
                   + wheelRadius*sin(TH2)*vertical(3) ...
                   + SS*lateral(3);

        set(hTyre(w), ...
            'XData',X,'YData',Yw,'ZData',Zw);
    end

    %% Update axle

    set(hAxle, ...
        'XData',[cL(1),cR(1)], ...
        'YData',[cL(2),cR(2)], ...
        'ZData',[cL(3),cR(3)]);

    %% Body orientation

    eLong = [
        sin(thetaB)*cos(phi)
        sin(thetaB)*sin(phi)
        cos(thetaB)
    ];

    eSide = lateral;

    eDepth = [
        cos(thetaB)*cos(phi)
        cos(thetaB)*sin(phi)
        -sin(thetaB)
    ];

    %% Body centre

    bodyCentre = axle + p.lb*eLong;

    %% Calculate the eight body corners

    signs = [
        -1 -1 -1
         1 -1 -1
        -1  1 -1
         1  1 -1
        -1 -1  1
         1 -1  1
        -1  1  1
         1  1  1
    ];

    vertices = zeros(8,3);

    for j = 1:8

        point = bodyCentre ...
            + signs(j,1)*(bodyLength/2)*eDepth ...
            + signs(j,2)*(bodyWidth/2)*eSide ...
            + signs(j,3)*(bodyHeight/2)*eLong;

        vertices(j,:) = point.';
    end

    set(hBody,'Vertices',vertices);

    %% Update frame

    set(hFrame, ...
        'XData',[axle(1),bodyCentre(1)], ...
        'YData',[axle(2),bodyCentre(2)], ...
        'ZData',[axle(3),bodyCentre(3)]);

    %% Update centre of mass

    set(hCOM, ...
        'XData',bodyCentre(1), ...
        'YData',bodyCentre(2), ...
        'ZData',bodyCentre(3));

    %% Update camera

    xlim(ax,[x-0.7 x+0.7]);
    ylim(ax,[y-0.7 y+0.7]);

    title(ax,sprintf( ...
        'Live Segway | Time: %.2f s | Pitch: %.2f degrees', ...
        tLive,rad2deg(thetaB)));

    drawnow;

    % Keep approximately real-time playback.
    % If computation takes longer than dt, MATLAB continues immediately.
    elapsed = toc(loopTimer);
    pause(max(0,dt-elapsed));

end

disp('Live Segway simulation finished.');

%% Local function: equations of motion
function dydt = segwayODE(t,y,p,vLeft,vRight)
    % Generalized coordinates and velocities
    thetaL = y(1);
    thetaR = y(2);
    thetaB = y(3);

    omegaL = y(4);
    omegaR = y(5);
    omegaB = y(6);
    phi    = y(9);

    %#ok<NASGU> thetaL and thetaR are retained as states; the kinetic energy
    % depends on their velocities, not their absolute values.

    % Optional time-dependent voltage input can be defined here.
    [vL,vR] = motorVoltages(t,vLeft,vRight);

    % Relative motor angular velocities (same convention as the supplied model)
    motorSpeedL = omegaL - omegaB;
    motorSpeedR = omegaR - omegaB;

    % Generalized forces: wheel torques and equal/opposite body reaction
    QL = (p.Kt/p.Rm)*vL ...
       - (p.Ke*p.Kt/p.Rm)*motorSpeedL ...
       - p.bf*motorSpeedL;

    QR = (p.Kt/p.Rm)*vR ...
       - (p.Ke*p.Kt/p.Rm)*motorSpeedR ...
       - p.bf*motorSpeedR;

    QB = -QL - QR;

    % Useful abbreviations
    D = omegaR - omegaL;
    K = p.Ibxx*sin(thetaB)^2 + p.Ibzz*cos(thetaB)^2;

    c = (p.mb*p.lb*p.lw/2)*cos(thetaB);
    A = p.mb*p.lb^2 + p.Ibyy;

    % Coefficient multiplying (alphaR - alphaL) in the wheel equations
    H = (p.lw^2/p.L^2) * ...
        (p.mb*p.lb^2*sin(thetaB)^2 + K + 2*p.Iwzz);

    % Velocity-dependent yaw coupling term
    C = (2*p.lw^2/p.L^2) * D*omegaB*sin(thetaB)*cos(thetaB) ...
        * (p.mb*p.lb^2 + p.Ibxx - p.Ibzz);

    % Mass matrix M(q) * qdd = rhs
    J = p.mw*p.lw^2 + p.Iwxx;
    B = p.mb*p.lw^2/4;

    M = [B + J + H,  B - H,       c; ...
         B - H,      B + J + H,   c; ...
         c,          c,           A];

    % Right-hand sides obtained by moving the non-acceleration terms
    % in the user's three Euler-Lagrange equations to the right-hand side.
    yawBody = (p.lw^2/p.L^2) * ...
        (p.mb*p.lb^2 + p.Ibxx - p.Ibzz) * D^2 ...
        * sin(thetaB)*cos(thetaB);

    gravityTerm = p.mb*p.g*p.lb*sin(thetaB);

    rhs = [QL + C; ...
           QR - C; ...
           QB + yawBody + gravityTerm];

    qdd = M\rhs;

    % Differential-drive kinematics
    xdot   = (p.lw/2)*(omegaL+omegaR)*cos(phi);
    ydot   = (p.lw/2)*(omegaL+omegaR)*sin(phi);
    phidot = (p.lw/p.L)*(omegaR-omegaL);

    dydt = [omegaL; omegaR; omegaB; ...
            qdd(1); qdd(2); qdd(3); ...
            xdot; ydot; phidot];
end

function [vL,vR] = motorVoltages(t,vLeft,vRight)
    % Default: constant voltages supplied at the top of the script.
    % Replace this function body for a controller or voltage profile.
    vL = vLeft;
    vR = vRight;

    % Example voltage pulse:
    % if t >= 0.5 && t < 0.8
    %     vL = 1.0;
    %     vR = 1.0;
    % end
end
