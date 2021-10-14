function J=Numerical_Jacobian(q)
%Numerical Jacobian 
disp('Numerical Method')
T=forward_kinematics(q);
R=T(1:3,1:3);
j1=tranz(0.3)*rotzd(q(1))*roty(q(2))*tranz(0.25)*roty(q(3))*tranz(0.09)*tranx(0.160)*rotx(q(4))*roty(q(5))*rotx(q(6))*tranx(0.072+0.107)*[inv(R) zeros(3,1); 0 0 0 1];

J1=[j1(1,4);j1(2,4);j1(3,4);j1(3,2);j1(1,3);j1(2,1)];

j2=tranz(0.3)*rotz(q(1))*rotyd(q(2))*tranz(0.25)*roty(q(3))*tranz(0.09)*tranx(0.160)*rotx(q(4))*roty(q(5))*rotx(q(6))*tranx(0.072+0.107)*[inv(R) zeros(3,1); 0 0 0 1];

J2=[j2(1,4);j2(2,4);j2(3,4);j2(3,2);j2(1,3);j2(2,1)];

j3=tranz(0.3)*rotz(q(1))*roty(q(2))*tranz(0.25)*rotyd(q(3))*tranz(0.09)*tranx(0.160)*rotx(q(4))*roty(q(5))*rotx(q(6))*tranx(0.072+0.107)*[inv(R) zeros(3,1); 0 0 0 1];

J3=[j3(1,4);j3(2,4);j3(3,4);j3(3,2);j3(1,3);j3(2,1)];

j4=tranz(0.3)*rotz(q(1))*roty(q(2))*tranz(0.25)*roty(q(3))*tranz(0.09)*tranx(0.160)*rotxd(q(4))*roty(q(5))*rotx(q(6))*tranx(0.072+0.107)*[inv(R) zeros(3,1); 0 0 0 1];

J4=[j4(1,4);j4(2,4);j4(3,4);j4(3,2);j4(1,3);j4(2,1)];

j5=tranz(0.3)*rotz(q(1))*roty(q(2))*tranz(0.25)*roty(q(3))*tranz(0.09)*tranx(0.160)*rotx(q(4))*rotyd(q(5))*rotx(q(6))*tranx(0.072+0.107)*[inv(R) zeros(3,1); 0 0 0 1];

J5=[j5(1,4);j5(2,4);j5(3,4);j5(3,2);j5(1,3);j5(2,1)];

j6=tranz(0.3)*rotz(q(1))*roty(q(2))*tranz(0.25)*roty(q(3))*tranz(0.09)*tranx(0.160)*rotx(q(4))*roty(q(5))*rotxd(q(6))*tranx(0.072+0.107)*[inv(R) zeros(3,1); 0 0 0 1];

J6=[j6(1,4);j6(2,4);j6(3,4);j6(3,2);j6(1,3);j6(2,1)];

J=[J1 J2 J3 J4 J5 J6];
end

