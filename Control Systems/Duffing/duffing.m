function x_dot=duffing(t,x)

global r alpha beta Fo omega

x_dot(1)=-r*x(1)+alpha^2*x(2)-beta*x(2)^3+Fo*cos(omega*t);
x_dot(2)=x(1);
x_dot=x_dot';

end
