% clear
% clc
format short
%% Coordinates of end-effector
pxt=-46.6461;  
pyt=91.5128;
pzt=218.5668;
%Orientation expressed in RPY angles
psi=91.5128*pi/180;% angles in degree
theta=91.5128*pi/180;
phi=-66.7045*pi/180;
%% vector of end effector
X_tool=[pxt,pyt,pzt,psi,theta,phi].'
%X_tool=[-117,167.5,535,-pi/2,0,pi/2].'
%  X_tool=[0,0,820,0,0,0].'
%X_tool=[-24.9054    0.0000  408.3907 -110.3857   90.0000 -110.3857]
%% find joint variables
q = inverse_kinematics(X_tool);
%% printing joint variables
q_joint=q*180/pi
