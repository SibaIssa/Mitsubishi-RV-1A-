function J=Geometrical_Jacobian(q)
 T=forward_kinematics(q) ;
% Geometrical Jacobian
disp('Geometrical Method')

O=T(1:3,4);
T1=tranz(0.3)*rotz(q(1));
T2=T1*roty(q(2))*tranz(0.25);
T3=T2*roty(q(3))*tranz(0.09)*tranx(0.160);
T4=T3*rotx(q(4));
T5=T4*roty(q(5));
T6=T5*rotx(q(6))*tranx(0.072+0.107);
R1=T1(1:3,1:3);
R2=T2(1:3,1:3);
R3=T3(1:3,1:3);
R4=T4(1:3,1:3);
R5=T5(1:3,1:3);
R6=T6(1:3,1:3);

% The first joint (revolute)
O0 = [0 0 0]';
z0 = [0 0 1]';
Jv1 = cross(z0, O - O0);
Jw1 = z0;

% The second joint (revolute)
O1=T1(1:3,4);
z1 = R1*[0 1 0]';
Jv2 = cross(z1, O - O1);
Jw2 = z1;

% The third joint (revolute)
O2=T2(1:3,4);
z2 = R2*[0 1 0]';
Jv3 = cross(z2, O - O2);
Jw3 = z2;

% The fourth joint (revolute)
O3=T3(1:3,4);
z3 = R3*[1 0 0]';
Jv4 = cross(z3, O - O3);
Jw4 = z3;

% The fifth joint (revolute)
O4=T4(1:3,4);
z4 = R4*[0 1 0]';
Jv5 = cross(z4, O - O4);
Jw5 = z4;

% The sixth joint (revolute)
O5=T5(1:3,4);
z5 = R5*[1 0 0]';
Jv6 = cross(z5, O - O5);
Jw6 = z5;


Jv = [Jv1, Jv2, Jv3,Jv4, Jv5, Jv6];
Jw = [Jw1, Jw2, Jw3,Jw4, Jw5, Jw6];
J = [Jv; Jw];
end


