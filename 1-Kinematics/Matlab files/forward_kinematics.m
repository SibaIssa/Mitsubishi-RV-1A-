function T0Tn = forward_kinematics(sigma, alpha, a, theta, d, q)
n = length(q);
T0Tn = eye(4);
for j=1:1:n
    if sigma(j)==0,
       theta(j)=q(j);
    else
        d(j)=q(j);
    end
    T0Tn = T0Tn*[cos(theta(j)) -cos(alpha(j))*sin(theta(j))  sin(alpha(j))*sin(theta(j))     a(j)* cos(theta(j));
                 sin(theta(j)) cos(alpha(j))*cos(theta(j))  -sin(alpha(j))* cos(theta(j))    a(j)*sin(theta(j));
                 0             sin(alpha(j))                 cos(alpha(j))                   d(j);
                 0 0 0 1];
end          

