%% SEGWAY LQR CONTROLLER + NYQUIST PLOT
% Linearised two-wheel Segway model
% Generalized coordinates:
% q = [theta_wL; theta_wR; theta_b]
%
% State vector:
% x = [theta_wL; theta_wR; theta_b;
%      theta_dot_wL; theta_dot_wR; theta_dot_b]
%
% Control inputs:
% u = [V_left; V_right]
%
% Model:
% M*q_ddot + C*q_dot + Kstiff*q = Bm*u

clear;
clc;
close all;

%% 1. PHYSICAL PARAMETERS
% Enter your actual Segway parameters here.

mw = 28 * 10^(-3);       % Mass of ONE wheel [kg]
mb = 465.1 - 2*mw;       % Body mass [kg]

lb = 59.4 * 10^(-3);       % Distance from axle to body COM [m]
lw = ( 45.65 / 2 ) * 10^(-3);       % Wheel radius [m]
L  = ( 101 - 17.5 / 2 ) * 10^(-3);       % Distance between wheel centres [m]

Ibyy = mb * ( ( 26 * 10^(-3) )^2 + ( ( 155 + 15 ) * 10^(-3) )^2 ) / 12;     % Body inertia around body y-axis [kg*m^2]
Ibzz = mb * ( ( 26 * 10^(-3) )^2 + ( ( 101 - 17.5 ) * 10^(-3) )^2 ) / 12;     % Body inertia around body z-axis [kg*m^2]

Iwxx = mw * lw^2 / 4 + mw * 17.5^2 / 12;     % One wheel inertia around x-axis [kg*m^2]
Iwzz = mw * lw^2 / 4 + mw * 17.5^2 / 12;     % One wheel inertia around z-axis [kg*m^2]

g = 9.81;       % Gravitational acceleration [m/s^2]

Ke = 0.191;       % Motor back-EMF constant
Kt = 0.191;       % Motor torque constant
Rm = 4.4;       % Motor resistance [Ohm]
bf = 0.001;       % Friction coefficient

%% Check that parameters have been entered

parameters = [mb mw lb lw L Ibyy Ibzz Iwxx Iwzz ...
              Ke Kt Rm bf];

if any(~isfinite(parameters)) || any(parameters <= 0)
    error('Enter all physical parameters before running the script.');
end

%% 2. MODEL CONSTANTS

km = Kt/Rm;
cm = Ke*Kt/Rm + bf;

alpha = mb*lw^2/4;
beta  = (lw^2/L^2)*(Ibzz + 2*Iwzz);
gamma = mw*lw^2 + Iwxx;

h  = mb*lb*lw/2;
Jb = mb*lb^2 + Ibyy;

%% 3. MASS MATRIX M

M = [alpha + gamma + beta, alpha - beta, h;
     alpha - beta, alpha + gamma + beta, h;
     h, h, Jb];

%% 4. DAMPING MATRIX C

C = cm*[ 1,  0, -1;
         0,  1, -1;
        -1, -1,  2];

%% 5. STIFFNESS MATRIX K

Kstiff = [0, 0, 0;
          0, 0, 0;
          0, 0, -mb*g*lb];

%% 6. MOTOR INPUT MATRIX Bm

Bm = km*[ 1,  0;
          0,  1;
         -1, -1];

%% 7. STATE-SPACE MATRICES A AND B

% State:
% x = [q; q_dot]

A = [zeros(3), eye(3);
     -M\Kstiff, -M\C];

B = [zeros(3,2);
     M\Bm];

disp('Mass matrix M:');
disp(M);

disp('State matrix A:');
disp(A);

disp('Input matrix B:');
disp(B);

%% 8. LQR WEIGHT MATRICES Q AND R

% Bryson's rule:
% Q(i,i) = 1/(maximum acceptable state deviation)^2
% R(i,i) = 1/(maximum acceptable control input)^2
%
% Change these limits to match your requirements.

% Maximum acceptable deviations:
% [wheel angle L, wheel angle R, body pitch,
%  wheel angular velocity L, wheel angular velocity R,
%  body angular velocity]

x_max = [pi;
         pi;
         0.15;
         5;
         5;
         1];

% Maximum motor voltages [V]
u_max = [12;
         12];

Q = diag(1./x_max.^2);
R = diag(1./u_max.^2);

disp('LQR weight matrix Q:');
disp(Q);

disp('LQR weight matrix R:');
disp(R);

%% 9. CALCULATE LQR GAIN

[Klqr, P, poles] = lqr(A, B, Q, R);

disp('LQR feedback gain:');
disp(Klqr);

disp('Closed-loop poles:');
disp(poles);

%% 10. CLOSED-LOOP SYSTEM

% Feedback law:
% u = -Klqr*x

Acl = A - B*Klqr;

disp('Closed-loop eigenvalues:');
disp(eig(Acl));

if all(real(eig(Acl)) < 0)
    disp('The closed-loop system is asymptotically stable.');
else
    warning('The closed-loop system is NOT asymptotically stable.');
end

%% 11. NYQUIST PLOT

% Plant from motor voltage to the six states:
Pplant = ss(A, B, eye(6), zeros(6,2));

% Open-loop return ratio:
% L(s) = Klqr*(sI-A)^(-1)*B

Lopen = ss(Klqr)*Pplant;

figure;
nyquistplot(Lopen);
grid on;
title('Nyquist Plot - Segway LQR Open-Loop Return Ratio');

%% 12. INITIAL CONDITION SIMULATION

% Initial state:
% Small initial body tilt of 5 degrees.
% All other initial states are zero.

x0 = [0;
      0;
      deg2rad(5);
      0;
      0;
      0];

% Simulation time
t = 0:0.01:10;

% Closed-loop system with state output
sys_cl = ss(Acl, zeros(6,1), eye(6), zeros(6,1));

[y, tout] = initial(sys_cl, x0, t);

%% 13. PLOT BODY ANGLE

figure;
plot(tout, rad2deg(y(:,3)), 'LineWidth', 1.5);
grid on;

xlabel('Time [s]');
ylabel('Body pitch [degrees]');
title('Segway Body Pitch - LQR Response');

%% 14. PLOT WHEEL ANGLES

figure;

plot(tout, y(:,1), 'LineWidth', 1.5);
hold on;

plot(tout, y(:,2), 'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Wheel angle [rad]');
title('Wheel Angles - LQR Response');

legend('Left wheel', 'Right wheel');

%% 15. PLOT ANGULAR VELOCITIES

figure;

plot(tout, y(:,4), 'LineWidth', 1.5);
hold on;

plot(tout, y(:,5), 'LineWidth', 1.5);
plot(tout, y(:,6), 'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Angular Velocities - LQR Response');

legend('Left wheel', 'Right wheel', 'Body');
