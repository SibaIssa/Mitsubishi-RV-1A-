%derivitive of rotation matrix around x-axis
function f= rotxd(q)
f= [0 0 0 0; 0 -sin(q) -cos(q) 0; 0 cos(q) -sin(q) 0;0 0 0 0];
end


