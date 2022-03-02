function X_dot = SystemDyn(t,X)
m_1 = 5;
m_2 = 2.5;
L_1 = 0.5;
L_2 = 0.5;
g = 9.8;
C_damp = 2;
omega = 4;
X_dot = zeros(4,1);

A_11 = m_1*L_1^2 + m_2*L_1^2;
A_12 = m_2*L_1*L_2*cos(X(3)-X(1));
A_21 = m_2*L_1*L_2*cos(X(3)-X(1));
A_22 = m_2*L_2^2;

A = [A_11, A_12;
    A_21, A_22];

b_1 = -(m_1+m_2)*g*L_1*sin(X(1))-m_2*L_1*L_2*X(4)^2*sin(X(3)-X(1))-C_damp*X(2);
b_2 = -m_2*g*L_2*sin(X(3))+m_2*L_1*L_2*X(2)^2*sin(X(3)-X(1))-C_damp*X(4);
l = abs(det(A));
b = [b_1;b_2];
Acc = inv(A)*b;

X_dot(1) = X(2);
X_dot(2) = Acc(1);
X_dot(3) = X(4);
X_dot(4) = Acc(2);

end