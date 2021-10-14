function [T,R03,R36]=forward_kinematics(q)

l1=0.3; l2=0.25; l3=0.09; l4=0.16; l5=0.072; l6=0.107;

T= Tz(l1)*Rz(q(1))*Tx(-l2*cos(q(2)))*Tz(l2*sin(q(2)))*Ry(q(2))*Tx(-l3*cos(q(3)))*Tz(l3*sin(q(3)))*Ry(q(3))*Tz(l4)*Rz(q(4))*Ry(q(5))*Rz(q(6))*Tz(l5+l6);
R03=Tz(0.4)*Rz(q(1))*Tx(0.025)*Ry(q(2))*Tz(0.56)*Ry(q(3))*Tz(0.025)*Tx(0.515);
R03=R03(1:3,1:3);
R36=Rx(q(4))*Ry(q(5))*Rx(q(6))*Tx(0.09);
R36=R36(1:3,1:3);

end

