close all; clear all; clc;

global r alpha beta Fo omega

r=0.23; alpha=-1; beta=1; omega=1.2;

%% Fo = 0.20
Fo=0.20;

[t x]=ode45(@duffing,0:2*pi/omega/100:4000,[0 1]);

figure(1)
plot(t(2000:4000),x(2000:4000,1),'g')
axis tight
title('Time Series')
figure(2)
plot(x(5000:10000,2),x(5000:10000,1),'y')
axis tight
title('Phase Space')
%% Fo = 0.218
Fo=0.218;

[t x]=ode45(@duffing,0:2*pi/omega/100:4000,[0 1]);

figure(3)
plot(t(2000:4000),x(2000:4000,1),'g')
axis tight
title('Time Series')
figure(4)
plot(x(5000:10000,2),x(5000:10000,1),'y')
axis tight
title('Phase Space')
%% Fo = 0.22
Fo=0.22;

[t x]=ode45(@duffing,0:2*pi/omega/100:4000,[0 1]);

figure(5)
plot(t(2000:4000),x(2000:4000,1),'g')
axis tight
title('Time Series')
figure(6)
plot(x(2000:10000,2),x(2000:10000,1),'y')
axis tight
title('Phase Space')
%% Fo = 0.37
Fo=0.37;

[t x]=ode45(@duffing,0:2*pi/omega/100:4000,[0 1]);

figure(7)
plot(t(2000:4000),x(2000:4000,1),'g')
axis tight
title('Time Series')
figure(8)
plot(x(2000:10000,2),x(2000:10000,1),'y')
axis tight
title('Phase Space')
%% Fo = 0.50
Fo=0.50;

[t x]=ode45(@duffing,0:2*pi/omega/100:4000,[0 1]);

figure(9)
plot(t(2000:6000),x(2000:6000,1),'g')
axis tight
title('Time Series')
figure(10)
plot(x(2000:10000,2),x(2000:10000,1),'y')
axis tight
title('Phase Space')
%% Fo = 0.65
Fo=0.65;

[t x]=ode45(@duffing,0:2*pi/omega/100:4000,[0 1]);

figure(11)
plot(t(2000:4000),x(2000:4000,1),'g')
axis tight
title('Time Series')
figure(12)
plot(x(2000:10000,2),x(2000:10000,1),'y')
axis tight
title('Phase Space')