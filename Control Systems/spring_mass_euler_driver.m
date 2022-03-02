function spring_mass_euler_driver(omega,zeta,tmax,h,u0,v0)

nmax = tmax/h;
y1(1) = u0;
y2(1) = v0;
ti(1) = 0;

for i=1:nmax
    t = i*h;
    y = [y1(i),y2(i)];
    dydt = spring_mass_damper(t,y);
    y1(i+1) = y1(i) + h*dydt(1);
    y2(i+1) = y2(i) + h*dydt(2);
    ti(i+1) = t;
end

% time history plot
figure (1)
plot(ti,y1,'-',ti,y2,'-.')
title('time history')
xlabel('t')
ylabel('{\bf y}(t)')
legend('u(t)','v(t)')

% phase space plot
figure (2)
plot(y1,y2,'-')
title('phase space')
xlabel('u(t)')
ylabel('v(t)')

axis equal

    function [ dydt ] = spring_mass_damper( t,y )
    %   spring-mass-damper system (state-space form)

    dydt = zeros(2,1);
    dydt(1) = y(2);
    dydt(2) = -omega*omega*y(1)-2*zeta*omega*y(2);
    end

end


