clc
draw = 0;
n = 6;

% start position q0 and final position qf in joint space 
q0 = [0, 0, 0, 0, 0, 0];                                  % start
qf = [7.5921,0.5532, 11.1868, 3.4214, 0.0872, 5.7279];    % end
deltaq = qf - q0; 

% max speed and accelaration for each joint  
vm = [3.1459, 1.5708, 2.35619, 3.14159, 3.14159, 3.66519];%rad.s^-1
am = [12, 8, 10, 4, 4, 2];                                %rad.s^-2

% calculating tb, tao and tf for each joint
tb = vm./am;
tf = abs(deltaq)./vm + tb;
tao = tf - tb;

% displaying the result
res = array2table([tb', tao', tf'], 'VariableNames',{'tb','tao','tf'});
fprintf('Trajectory time points:\n\n');
disp(res)


vm = vm .* sign(deltaq);
am = am .* sign(deltaq);
traj_calc(q0, qf, tb, tf, tao, vm, am, vm, am, n, draw, 'Normal control');

% Synchronization
% Determine maximum tb and tao then calculate tf
tb_syn = max(tb);
tao_syn = max(tao);
tf_syn =  tao_syn + tb_syn;

disp(['Synchronized tb  = ', num2str(tb_syn)]);
disp(['Synchronized tao = ', num2str(tao_syn)]);
disp(['Synchronized tf  = ', num2str(tf_syn)]);

% Recalculating the trajectory using the new time values
vm_syn = deltaq/tao_syn;                       
am_syn = vm_syn/tb_syn;


% Recalculate trajectories
traj_calc(q0, qf, tb_syn, tf_syn, tao_syn, vm_syn, am_syn, vm, am, n, draw, 'Sychronised control');

% numerical control solution
dt = 0.1;

% flooring dt to the nearist tenth 
num = 0;
while (floor(dt*10^num)~=dt*10^num)
    num=num+1;
end
E = 1*10^-num;

% calculating numerical time points to be identical to sample multiplication  
if( rem(tb_syn, dt) ~= 0)
    tb_num = round(tb_syn, num) + E;
else
    tb_num = round(tb_syn, num);
end

if( rem(tao_syn, dt) ~= 0)
    tao_num = round(tao_syn, num) + E;
else
    tao_num = round(tao_syn, num);
end

tf_num = tao_num + tb_num;


% Recalculating the trajectory using the new time values
vm_num = deltaq/tao_num;                     
am_num = vm_num/tb_num;

% ploting trajectories
traj_calc(q0, qf, tb_num, tf_num, tao_num, vm_num, am_num, vm, am, n, draw, 'Numirical Synchronise control');
