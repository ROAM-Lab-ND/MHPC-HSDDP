function [ f_s, p_s, f_s_fb ] = control_alpha_ztau3D_frontal( t,x_c,x_d,model,control_params)
%control Stance force computation for rigid-body quadruped model
f_s = cell(4,1);
p_s = cell(4,1);
f_s_fb=cell(4,1);

x_c;
quat =x_c(1:4);
R   = quatToR(quat);
p_c = x_c(5:7);
omega=x_c(8:10);
v_c  =x_c(11:13);
rpy = quatToRpy(quat);

rpy_yaw = [0 0 rpy(3)];
quat_yaw = rpyToQuat(rpy_yaw);
R_yaw    = quatToR(quat_yaw);
%R_yaw = R_yaw';


f_total = zeros(3,1);


for i = 1:4
    
   % Only create force for legs in stance 
   if x_d.leg_state{i} == 1
       t_state = t - x_d.leg_t0{i};
       t_state_norm = t_state / control_params.t_stance{i};
       
       bez_x = [0 control_params.bez_x{i} 0];
       bez_y = [0 control_params.bez_y{i} 0];
       bez_z = [0 control_params.bez_z{i} 0];
       
       fx = 100 * polyval_bz(bez_x, t_state_norm);
       fy = 100 * polyval_bz(bez_y, t_state_norm);
       fz = 100 * polyval_bz(bez_z, t_state_norm);
       
       f_s{i} = R_yaw*[fx fy fz]';
       p_s{i} = x_d.p_foot{i};
       f_s_fb{i} = [0 0 0]';
       f_total = f_total + f_s{i};
   else
       f_s{i} = zeros(3,1);
       p_s{i} = zeros(3,1);
       f_s_fb{i} = zeros(3,1);
   end
end



