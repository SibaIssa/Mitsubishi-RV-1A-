function q = inverse_kinematics(X)
global d1 d4 d6 a2 a3
d1=300;d4=160;d6=179;a2=-250;a3=-90;
pxt=X(1);pyt=X(2);pzt=X(3);
psi=X(4);theta=X(5);phi=X(6);%in radian
%% Roll-Pitch-Yaw
sx=cos(phi)*cos(theta);
sy=sin(phi)*cos(theta);
sz=-sin(theta);
nx=cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
ny=sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
nz=cos(theta)*sin(psi);
ax=cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
ay=sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
az=cos(theta)*cos(psi);
px4=pxt - ax*d6;
py4=pyt - ay*d6;
pz4=pzt - az*d6;

T_tool=[sx nx ax pxt;sy ny ay pyt;sz nz az pzt;0 0 0 1]
%%
q=[];
q1=[atan2(py4,px4);atan2(-py4,-px4)];
for i=1:length(q1)
    q_1= q1(i);
    P41_x=(px4*cos(q_1) + py4*sin(q_1));
    P41_y=(d1-pz4);
    a=P41_x;
    b=P41_y;
    c=(d4^2 -P41_x^2-P41_y^2-a2^2)/(2*a2);
    if (a^2 + b^2 - c^2)<0
         q2(i)=nan;
%          q3(:,i)=nan;q4(:,i)=nan;q5(:,i)=nan
%          q6(:,i)=nan
    else
         q2(1)=atan2(c,sqrt(a^2 + b^2 - c^2))-atan2(a,b);
         q2(2)=atan2(c,-sqrt(a^2 + b^2 - c^2))-atan2(a,b);
            for j=1:length(q2)
                q3=atan2((cos(q2(j))*a+sin(q2(j))*b+a2)/d4,(-cos(q2(j))*b+sin(q2(j))*a)/d4);
                Gx=cos(q_1)*cos(q2(j)+q3)*ax+sin(q_1)*cos(q2(j)+q3)*ay-sin(q2(j)+q3)*az;
                Gy=cos(q_1)*ay-ax*sin(q_1);
                Ex=cos(q_1)*cos(q2(j)+q3)*sx+sin(q_1)*cos(q2(j)+q3)*sy-sin(q2(j)+q3)*sz;
                Ey=cos(q_1)*sy-sx*sin(q_1);
                Fx=cos(q_1)*cos(q2(j)+q3)*nx+sin(q_1)*cos(q2(j)+q3)*ny-sin(q2(j)+q3)*nz;
                Fy=cos(q_1)*ny-nx*sin(q_1);
                Gz=cos(q_1)*sin(q2(j)+q3)*ax+sin(q_1)*sin(q2(j)+q3)*ay+cos(q2(j)+q3)*az;
                q4(1)=atan2(Gy,Gx);
                q4(2)=atan2(-Gy,-Gx);
                for k=1:length(q4)
                        s6=-sin(q4(k))*Ex+cos(q4(k))*Ey;
                        c6=-sin(q4(k))*Fx+cos(q4(k))*Fy;
                    q6=atan2(s6,c6);
                        s5=cos(q4(k))*Gx+sin(q4(k))*Gy;
                        c5=Gz;
                    q5=atan2(s5,c5);
                q1_6=[q_1,q2(j),q3,q4(k),q5,q6];  
                q=[q;q1_6];
                end
            end
    end
end   
end
