%% Direct geometric model

%syms d1 q2 d3 q4 q5 q6
%syms l1 l2 l4 l5
format short
%% DH parameters

alpha=[-pi/2 0 pi/2 -pi/2 pi/2 0];
d=[300 0 0 160 0 179];
a=[0 -250 -90 0 0 0];
theta=[90.0000   0   -0.0000  -90.0000  -90.0000   -0.0000]*pi/180;
sigma=[0 0 0 0 0 0];
q=[256.5122   28.9580  103.2580  -24.6348   63.3799  -24.6347]*pi/180;
%% calling Forward \\kinematics function

T_tool=forward_kinematics(sigma, alpha, a, theta, d, q)
phi=atan2(T_tool(2,1),T_tool(1,1));
phi_1=phi+pi;
theta=atan2(-T_tool(3,1),cos(phi)*T_tool(1,1)+sin(phi)*T_tool(2,1));
theta_1=atan2(-T_tool(3,1),cos(phi_1)*T_tool(1,1)+sin(phi_1)*T_tool(2,1));
psi=atan2(-cos(phi)*T_tool(2,3)+sin(phi)*T_tool(1,3),...
    cos(phi)*T_tool(2,2)-sin(phi)*T_tool(1,2));
psi_1=atan2(-cos(phi_1)*T_tool(2,3)+sin(phi_1)*T_tool(1,3),...
    cos(phi_1)*T_tool(2,2)-sin(phi_1)*T_tool(1,2));

RPY=[phi;theta;psi;psi_1;theta_1;phi_1];
for j=1:length(RPY)
        if RPY(j)>pi
            RPY(j)=RPY(j)-2*pi;
        elseif RPY(j)<-pi
            RPY(j)=2*pi+RPY(j);
        end
end
X_tool=[T_tool(1:3,4);RPY(1:3)].';
X_tool2=[T_tool(1:3,4);RPY(4:6)].';
save mitsu X_tool X_tool2
X_tool=[T_tool(1:3,4);RPY(1:3)*180/pi].'
X_tool2=[T_tool(1:3,4);RPY(4:6)*180/pi].'
%% printing transformation matrices for check
% j=3
% T3_2=[cos(theta(j)) -cos(alpha(j))*sin(theta(j))  sin(alpha(j))*sin(theta(j))     a(j)* cos(theta(j));
%                  sin(theta(j)) cos(alpha(j))*cos(theta(j))  -sin(alpha(j))* cos(theta(j))    a(j)*sin(theta(j));
%                  0             sin(alpha(j))                 cos(alpha(j))                   d(j);
%                  0 0 0 1]
% %        
% %              
