%% Z-Plane Contours
close all; clear;

%% Let's draw a unit circle centered @ origin
gamma = (0:pi/50:2*pi)';    % angles around unit circle
x_coord = cos(gamma);
y_coord = sin(gamma);

figure; hold on;
p1 = plot(x_coord,y_coord, 'r--', 'LineWidth', 2);
axis equal
title('z-plane', 'FontSize', 16);
xlabel('real axis', 'FontSize', 16);
ylabel('imaginary axis', 'FontSize', 16);
legend('stability condition', 'Location','bestoutside')
ax = gca;       % grabs current axis
ax.FontSize = 16;
%% Preformance constraints
ts_max = 2;         % maximum settling time 
ts_min = 0.3;       % minimum settling time
Mp_max = 0.5;       % maximum overshoot constraint
Mp_min = 0.2;       % minimum overshoot constraint
tr_max = 0.5;       % maximum rise time
%% Hardware constraints
fs = 10;        % sampling frequency in Hz
T = 1/fs;       % sampling period in seconds
%% Settling time constraints

% maximum settling time 
mag_ts_max = exp(-4*T/ts_max);
x_coord_ts_max = mag_ts_max*cos(gamma);
y_coord_ts_max = mag_ts_max*sin(gamma);

% minimum settling time 
mag_ts_min = exp(-4*T/ts_min);
x_coord_ts_min = mag_ts_min*cos(gamma);
y_coord_ts_min = mag_ts_min*sin(gamma);

% combine the max and min coordinates to fill a "donut"
x_coord_ts = [x_coord_ts_max; x_coord_ts_min];
y_coord_ts = [y_coord_ts_max; y_coord_ts_min];

p2 = fill(x_coord_ts, y_coord_ts, 'g', 'linestyle','none');   % fills plotted area
alpha(0.25);    % adds transparency
legend([p1,p2],{'stability condition','settling time constraint'})

%% Overshoot constraint

% define lambda
phase_lambda = [0:pi/50:pi]';

% maximum overshoot constraint
zeta_Mp_max = sqrt(log(Mp_max)^2/(pi^2 + log(Mp_max)^2));
mag_Mp_max = exp(-zeta_Mp_max/sqrt(1-zeta_Mp_max^2)*phase_lambda);

x_coord_Mp_max = mag_Mp_max.*cos(phase_lambda);
y_coord_Mp_max = mag_Mp_max.*sin(phase_lambda);
x_coord_Mp_max = [x_coord_Mp_max; flipud(x_coord_Mp_max)];
y_coord_Mp_max = [y_coord_Mp_max; flipud(-y_coord_Mp_max)];

% minimum overshoot constraint
zeta_Mp_min = sqrt(log(Mp_min)^2/(pi^2 + log(Mp_min)^2));
mag_Mp_min = exp(-zeta_Mp_min/sqrt(1-zeta_Mp_min^2)*phase_lambda);

x_coord_Mp_min = mag_Mp_min.*cos(phase_lambda);
y_coord_Mp_min = mag_Mp_min.*sin(phase_lambda);
x_coord_Mp_min = [x_coord_Mp_min; flipud(x_coord_Mp_min)];  % flips shape up/down
y_coord_Mp_min = [y_coord_Mp_min; flipud(-y_coord_Mp_min)];

% combine the max and min coordinates
x_coord_Mp = [x_coord_Mp_max; x_coord_Mp_min];
y_coord_Mp = [y_coord_Mp_max; y_coord_Mp_min];

p3 = fill(x_coord_Mp, y_coord_Mp, 'b', 'linestyle','none');
alpha(0.25);
legend([p1,p2,p3],{'stability condition','settling time constraint',...
                    'overshoot constraint' })

%% Rise time constraint

% define terms
zeta_tr = linspace(0,0.999,501)';                           % damping ratio
beta = atan(sqrt(1-zeta_tr.^2)./zeta_tr);                   % derived beta term
omegan_tr = (pi-beta)./(tr_max*sqrt(1-zeta_tr.^2));         % natural frequencies
omegad_tr = omegan_tr.*sqrt(1-zeta_tr.^2);                  % damped frequencies

mag_tr = exp(-zeta_tr.*omegan_tr*T);
phase_tr = omegad_tr*T;

% maximum rise time

% "pacman's mouth"
x_coord_tr_max = mag_tr.*cos(phase_tr);
y_coord_tr_max = mag_tr.*sin(phase_tr);

% tracing from "pacman's mouth" around the unit circle, up to an angle pi
x_coord_tr_max = [flipud(x_coord_tr_max); cos(linspace(phase_tr(1),pi,201)')];
y_coord_tr_max = [flipud(y_coord_tr_max); sin(linspace(phase_tr(1),pi,201)')];

% mirroring the shape of "pacman's head" over the real axis
x_coord_tr_max = [x_coord_tr_max; flipud(x_coord_tr_max)];
y_coord_tr_max = [y_coord_tr_max; -flipud(y_coord_tr_max)];

p4 = fill(x_coord_tr_max, y_coord_tr_max, 'r', 'linestyle','none');
alpha(0.25);    % adds transparency
legend([p1,p2,p3,p4],{'stability condition','settling time constraint',...
                    'overshoot constraint', 'rise time constraint'})