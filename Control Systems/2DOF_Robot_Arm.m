%Initialize the state of the system
X_int = [2,0,0,0]';
%Specify the step-size
step_size = 1E-2;
%Specify the end time of 5 minutes
end_time = 5*60;
%Create a time vector
t = 0:step_size:end_time;
%Pre-allocate a state vector matrx
X = zeros(4,length(t)+1);
%Assign the initial state value to state vector matrix
X(:,1) = X_int;
%Some system parameters
L_1 = 0.5;
L_2 = 0.5;

for i = 1:length(t)
K_1 = step_size*SystemDyn(t(i),X(:,i));
K_2 = step_size*SystemDyn(t(i)+0.5*step_size,X(:,i)+0.5*K_1);
K_3 = step_size*SystemDyn(t(i)+0.5*step_size,X(:,i)+0.5*K_2);
K_4 = step_size*SystemDyn(t(i)+step_size,X(:,i)+K_3);
X(:,i+1) = X(:,i) + (K_1+2*K_2+2*K_3+K_4)/6;
end


X_1 = @(X) L_1*cos(X(1));
Y_1 = @(X) L_1*sin(X(1));
X_2 = @(X) L_1*cos(X(1)) + L_2*cos(X(3));
Y_2 = @(X) L_1*sin(X(1))+ L_2*sin(X(3));
h = figure;
axis tight manual
plot([0,Y_1(X(:,1)),Y_2(X(:,1))],[0,-X_1(X(:,1)),-X_2(X(:,1))])
ax = gca;
ax.NextPlot = 'replaceChildren';
loops = 800;
M(loops) = struct('cdata',[],'colormap',[]);
h.Visible = 'off';
for i = 1:loops
plot([0,Y_1(X(:,i+1)),Y_2(X(:,i+1))]',[0,-X_1(X(:,i+1)),-X_2(X(:,i+1))]','-o','MarkerSize',10)
hold on
yline(0)
hold off
title('Double Pendulum')
xlabel('X')
ylabel('Y')
ylim([-2,2])
xlim([-2,2])
drawnow
M(i) = getframe;
end
h.Visible = 'on';
