% copyright @ Ezekiel Mok
% mozhb@mail2.sysu.edu.cn
% 2024/12/11
clc,clear
close all
sim.t_total = 24;%s
sim.point_num=2000;
sim.ts = sim.t_total / sim.point_num;
sim.t=linspace(0, sim.t_total, sim.point_num);
cx = zeros(sim.point_num,1);
cy= zeros(sim.point_num,1);
cz= zeros(sim.point_num,1);
start_ang=0.55*pi;
end_ang=2.25*pi;
x0 = @(t_step) 16*sin(2 * t_step + 1);
y0 = @(t_step) -40*cos(t_step+ 0.5);
z0 = @(t_step) 0;
start_psi=0;
end_psi=sim.t_total;
k=0;
for theta=start_psi:(end_psi-start_psi)/(sim.point_num-1):end_psi
    k=k+1;
    cx(k) = x0( start_ang+theta/(end_psi-start_psi)*(end_ang-start_ang));
    cy(k) = y0( start_ang+theta/(end_psi-start_psi)*(end_ang-start_ang));
    cz(k)=0;
end
% figure(1)
% plot(cx, cy, 'm.',LineWidth=2)
% axis equal
% grid on
refer_path(:, 1:3) =[cx, cy, cz];
for i=1:length(refer_path)
    if i==1
        dx = refer_path(i + 1, 1) - refer_path(i, 1);
        dy = refer_path(i + 1, 2) - refer_path(i, 2);
        dz = refer_path(i + 1, 3) - refer_path(i, 3);
        ddx = refer_path(3, 1) + refer_path(1, 1) - 2 * refer_path(2, 1);
        ddy = refer_path(3, 2) + refer_path(1, 2) - 2 * refer_path(2, 2);
        ddz = refer_path(3, 3) + refer_path(1, 3) - 2 * refer_path(2, 3);
    elseif  i==length(refer_path)
        dx = refer_path(i, 1) - refer_path(i - 1, 1);
        dy = refer_path(i, 2) - refer_path(i - 1, 2);
        dz = refer_path(i, 3) - refer_path(i - 1, 3);
        ddx = refer_path(i, 1) + refer_path(i - 2, 1) - 2 * refer_path(i - 1, 1);
        ddy = refer_path(i, 2) + refer_path(i - 2, 2) - 2 * refer_path(i - 1, 2);
        ddz = refer_path(i, 3) + refer_path(i - 2, 3) - 2 * refer_path(i - 1, 3);
    else
        dx = refer_path(i + 1, 1) - refer_path(i, 1);
        dy = refer_path(i + 1, 2) - refer_path(i, 2);
        dz = refer_path(i + 1, 3) - refer_path(i, 3);
        ddx = refer_path(i + 1, 1) + refer_path(i - 1, 1) - 2 * refer_path(i, 1);
        ddy = refer_path(i + 1, 2) + refer_path(i - 1, 2) - 2 * refer_path(i, 2);
        ddz = refer_path(i + 1, 3) + refer_path(i - 1, 3) - 2 * refer_path(i, 3);
    end
    refer_path(i,4)=atan2(dy, dx);%
    refer_path(i,5)=(ddy * dx - ddx * dy) / ((dx ^ 2 + dy ^ 2) ^ (3 / 2));
    refer_path(i,6) = dx;
    refer_path(i,7) = dy;
    refer_path(i,8) = dz;
end
for i=1:length(refer_path)
    if i==1
        dphai = refer_path(i + 1, 4) - refer_path(i, 4);
    elseif  i==length(refer_path)
        dphai = refer_path(i,4) - refer_path(i - 1, 4);
    else
        dphai = refer_path(i + 1, 4) - refer_path(i, 4);
    end
    refer_path(i, 9) = dphai;%yaw
end
x_d=refer_path(:, 1);
y_d=refer_path(:, 2);
z_d=refer_path(:, 3);
phai_d = refer_path(:, 4);
dx_d = refer_path(:, 6);
dy_d = refer_path(:, 7);
dz_d = refer_path(:, 8);
dphai_d = refer_path(:, 9);
sim.list_x_rd = refer_path(:, 1:3)';
sim.list_v_rd =refer_path(:, 6:8)';
sim.x_0 =sim.list_x_rd(:,1);% 可以看做领航者的初始位置
sim.v_0 = [0; 0;0];% 可以看做领航者的初始速度
sim.x_10 =[-20,5,0]';%
sim.v_10 = [0; 0;0];%
sim.x_20 =[-10,15,0]';%
sim.v_20 = [0; 0;0];%
sim.x_30 =[-30,10,0]';%
sim.v_30 = [0; 0;0];%
ds_agent0=4;
ds_agent=2.5;
r_robot=0.2;
agent0 = agentclass2dim(sim.x_0, sim.v_0, sim.list_v_rd, sim.list_x_rd, ds_agent0, r_robot);% 虚拟领航者
agent1 = agentclass2dim(sim.x_10, sim.v_10, sim.list_v_rd, sim.list_x_rd, ds_agent, r_robot);% 智能体1
agent2 = agentclass2dim(sim.x_20, sim.v_20, sim.list_v_rd, sim.list_x_rd, ds_agent, r_robot);% 智能体2
agent3 = agentclass2dim(sim.x_30, sim.v_30, sim.list_v_rd, sim.list_x_rd, ds_agent, r_robot);% 智能体3
k=0;
param.km = 3.5;
param.k_f = 12;
param.ka_gain = 24;
param.k1 = 1;
param.k2 = 1-param.k1;
param.A_matrix = [1, 0, 0;
    1, 0, 0;
    1, 1, 0];%有向生成树图
param.B_matrix = [1, 0, 0]';%表征与领航者的直接通讯关系
param.fontsize=15;
a_interg=0;
aa_interg=0;
b_interg=0;
bb_interg=0;
x1_esm=agent0.x-[0.5,0.5,0]';
x2_esm=agent0.x+[0.4,0.4,0]';
x3_esm=agent0.x+[0.3,0.3,0]';
v1_esm = agent0.v;
v2_esm = agent0.v;
v3_esm = agent0.v;
ldestimator.lo1 = 0.1;
ldestimator.lo2 = 0.1;
ldestimator.lo3 = 25;
ldestimator.lo4 = 6;
ldestimator.yita1 = 3/2;
ldestimator.yita2 = 3/2;
sm_p1=[0,0,0]';
sm_p2=[-4,-4,6]';
sm_p3=[-4,4,6]';
sm_p1_dot=[0,0,0]';
sm_p2_dot=[0,0,0]';
sm_p3_dot=[0,0,0]';
sigma_p = [sm_p1, sm_p2, sm_p3];
sigma_p1 = [sm_p1, sm_p2, sm_p3];
sigma_pv = [sm_p1_dot, sm_p2_dot, sm_p3_dot];
trace_err1=nan+[sim.t];
trace_err2=nan+[sim.t];
trace_err3=nan+[sim.t];
trace_terr1=nan+[sim.t];
trace_terr2=nan+[sim.t];
trace_terr3=nan+[sim.t];
trace_esterr1=nan+[sim.t];
trace_esterr2=nan+[sim.t];
trace_esterr3=nan+[sim.t];
trace_obs1=nan+[sim.t];
trace_obs2=nan+[sim.t];
trace_obs3=nan+[sim.t];
trace_u1=nan+[sim.t];
trace_u2=nan+[sim.t];
trace_u3=nan+[sim.t];
trace_x0=nan+[sim.t;sim.t;sim.t];
trace_agentdis1=nan+[sim.t];
trace_agentdis2=nan+[sim.t];
trace_agentdis3=nan+[sim.t];
sim.obstacle_x =[0,9.5,0;9.5,2,0;20,0,0; 20,0,0; 20,0,0]';%
sim.obstacle_v =zeros(3,5);
for j = sim.t
    k=k+1;
    agent0.record_x(k)=agent0.x(1);
    agent0.record_y(k)=agent0.x(2);
    agent0.record_z(k)=agent0.x(3);
    agent1.record_x(k)=agent1.x(1);
    agent1.record_psi(k)=refer_path(k, 4);
    agent1.record_y(k)=agent1.x(2);
    agent1.record_z(k)=agent1.x(3);
    agent2.record_x(k)=agent2.x(1);
    agent2.record_y(k)=agent2.x(2);
    agent2.record_psi(k)=refer_path(k, 4);
    agent2.record_z(k)=agent2.x(3);
    agent3.record_x(k)=agent3.x(1);
    agent3.record_y(k)=agent3.x(2);
    agent3.record_z(k)=agent3.x(3);
    agent3.record_psi(k)=refer_path(k, 4);
    [v_m0, jacobi_m0] = motion(agent0.x, sim.list_x_rd(:, k), sim.list_v_rd(:, k) ./ sim.ts, param.km);
    agent0.v = v_m0;
    agent0.x = agent0.x + sim.ts .* agent0.v;
    p_now = [agent1.x, agent2.x, agent3.x];
    v_now = [agent1.v, agent2.v, agent3.v];
    [min_id1,min_pos1, min_dis_detected1] = returnnearobstacle(p_now,  sim.obstacle_x,  1) ;
    [min_id2,min_pos2, min_dis_detected2] = returnnearobstacle(p_now,  sim.obstacle_x,  2) ;
    [min_id3,min_pos3, min_dis_detected3] = returnnearobstacle(p_now,  sim.obstacle_x,  3) ;
    [pmin_id1, min_dis1] = returnnearagent(p_now, 1) ;
    [pmin_id2, min_dis2] = returnnearagent(p_now, 2) ;
    [pmin_id3, min_dis3] = returnnearagent(p_now, 3) ;
    trace_obs1(k)=min_dis_detected1;
    trace_obs2(k)=min_dis_detected2;
    trace_obs3(k)=min_dis_detected3;
    trace_agentdis1(k)=min_dis1;
    trace_agentdis2(k)=min_dis2;
    trace_agentdis3(k)=min_dis3;
    time_start_scaleup=820*sim.ts;
    time_span_scaledown=1;
    if j>=time_start_scaleup && j<=time_start_scaleup+time_span_scaledown
        % agent 3
        a=Accel_slow_fast(time_start_scaleup, time_span_scaledown, j, 0.5);
        % agent 3
        b=Accel_slow_fast(time_start_scaleup, time_span_scaledown, j, 0.5);
        a_interg = a_interg + a * sim.ts;
        aa_interg = aa_interg + a_interg * sim.ts;
        b_interg = b_interg + b * sim.ts;
        bb_interg = bb_interg + b_interg * sim.ts;
    end
    time_start_scaledown=920*sim.ts;
    time_span_scaledown=1;
    if j>=time_start_scaledown && j<=time_start_scaledown+time_span_scaledown
        % agent 3
        a=-Accel_slow_fast(time_start_scaledown, time_span_scaledown, j, 0.5);
        % agent 3
        b=-Accel_slow_fast(time_start_scaledown, time_span_scaledown, j, 0.5);
        a_interg = a_interg + a * sim.ts;
        aa_interg = aa_interg + a_interg * sim.ts;
        b_interg = b_interg + b * sim.ts;
        bb_interg = bb_interg + b_interg * sim.ts;
    end
    offset=[0,0,0];
    sigma_p1_last=sigma_p1;
    for i=1:1:3
        sigma_p1(:,i)=Aff('A', refer_path(k, 4), 1+aa_interg, 1+aa_interg,1,  0, 0, 0, sigma_p(:,i));
    end
    sigma_pv= (1/sim.ts).*(sigma_p1_last-sigma_p1);
    p_esm   = [x1_esm, x2_esm, x3_esm];
    pv_esm = [v1_esm, v2_esm, v3_esm];
    [x1_esm_dot, v1_esm_dot] = retrunaikx(ldestimator, param.A_matrix, param.B_matrix, p_esm, pv_esm, agent0.x, agent0.v,  1);
    [x2_esm_dot, v2_esm_dot] = retrunaikx(ldestimator, param.A_matrix, param.B_matrix, p_esm, pv_esm, agent0.x, agent0.v,  2);
    [x3_esm_dot, v3_esm_dot] = retrunaikx(ldestimator, param.A_matrix, param.B_matrix, p_esm, pv_esm, agent0.x, agent0.v,  3);
    x1_esm = x1_esm + sim.ts .* x1_esm_dot;
    x2_esm = x2_esm + sim.ts .* x2_esm_dot;
    x3_esm = x3_esm + sim.ts .* x3_esm_dot;
    v1_esm = v1_esm + sim.ts .* v1_esm_dot;
    v2_esm = v2_esm + sim.ts .* v2_esm_dot;
    v3_esm = v3_esm + sim.ts .* v3_esm_dot;
    [bar_sigma_f1, sigmaf_i1, pf_i1, vf_i1] = return_bar_sigmaf(param, p_now, v_now, p_esm, pv_esm, sigma_p1, sigma_pv, 1);
    [bar_sigma_f2, sigmaf_i2, pf_i2, vf_i2] = return_bar_sigmaf(param, p_now, v_now, p_esm, pv_esm, sigma_p1, sigma_pv, 2);
    [bar_sigma_f3, sigmaf_i3, pf_i3, vf_i3] = return_bar_sigmaf(param, p_now, v_now, p_esm, pv_esm, sigma_p1, sigma_pv, 3);
    [v_fm1, jacobi_fm1] = PDFBC(param,bar_sigma_f1, p_now, sigma_p1, pf_i1, vf_i1, 1);
    [v_fm2, jacobi_fm2] = PDFBC(param,bar_sigma_f2, p_now, sigma_p1, pf_i2, vf_i2, 2);
    [v_fm3, jacobi_fm3] = PDFBC(param,bar_sigma_f3, p_now, sigma_p1, pf_i3, vf_i3, 3);
    if min_dis_detected1<=agent1.R_safe
        [smrho_oa1, jacobi_o1] = returnobstaclejacobi(p_now, agent1.R_safe, ...
            agent1.r_robot, sim.obstacle_x(:,min_id1), min_dis_detected1, 1);
        v_o1 = pinv(jacobi_o1).*param.ka_gain*(agent1.R_safe - smrho_oa1);
    else
        v_o1 = zeros(3, 1);
        jacobi_o1 = zeros(1, 3);
    end
    if min_dis_detected2<=agent2.R_safe
        [smrho_oa2, jacobi_o2] = returnobstaclejacobi(p_now, agent2.R_safe, ...
            agent2.r_robot, sim.obstacle_x(:,min_id2), min_dis_detected2, 2);
        v_o2 = pinv(jacobi_o2).*param.ka_gain*(agent2.R_safe - smrho_oa2);
    else
        v_o2 = zeros(3, 1);
        jacobi_o2 = zeros(1, 3);
    end
    if min_dis_detected3<=agent3.R_safe
        [smrho_oa3, jacobi_o3] = returnobstaclejacobi(p_now, agent3.R_safe, ...
            agent3.r_robot, sim.obstacle_x(:,min_id3), min_dis_detected3, 3);
        v_o3 = pinv(jacobi_o3).*param.ka_gain*(agent3.R_safe - smrho_oa3);
    else
        v_o3 = zeros(3, 1);
        jacobi_o3 = zeros(1, 3);
    end
    if min_dis_detected1<=agent1.R_safe
        agent1.v =  v_o1 + (eye(3, 3) - pinv(jacobi_o1) * jacobi_o1) *v_fm1;
    else
        agent1.v =  v_fm1 + (eye(3, 3) - pinv(jacobi_fm1) * jacobi_fm1) * v_o1;
    end
    agent1.x =  agent1.x + sim.ts .* agent1.v;
    if min_dis_detected2<=agent2.R_safe
        agent2.v =  v_o2 + (eye(3, 3) - pinv(jacobi_o2) * jacobi_o2) *v_fm2;
    else
        agent2.v =  v_fm2 + (eye(3, 3) - pinv(jacobi_fm2) * jacobi_fm2) * v_o2;
    end
    agent2.x =  agent2.x + sim.ts .* agent2.v;
    if min_dis_detected3<=agent3.R_safe
        agent3.v =  v_o3 + (eye(3, 3) - pinv(jacobi_o3) * jacobi_o3) *v_fm3;
    else
        agent3.v =  v_fm3 + (eye(3, 3) - pinv(jacobi_fm3) * jacobi_fm3) * v_o3;
    end
    agent3.x =  agent3.x + sim.ts .* agent3.v;
end
agcolor1=[0.99, 0.49, 0.00];
agcolor2=[0.17, 0.62, 0.47];
agcolor3=[0.88, 0.17, 0.56];
figure(1);
hold on; box on;
grid on
set(gcf, 'Position', [-1200, 100, 1.2*800, 1.2*600]);
set(gca, 'fontSize', param.fontsize)
set(get(gca, 'xlabel'), 'String', 'x (m)', 'fontSize', param.fontsize);
set(get(gca, 'ylabel'), 'String', 'y (m)', 'fontSize', param.fontsize);
set(get(gca, 'zlabel'), 'String', 'z (m)', 'fontSize', param.fontsize);
axis([-38,38,-48,48,-10,40])
view(3)
set(gca, 'fontsize', param.fontsize);
idx=2000;
h0=plot3(agent0.record_x(1:idx),agent0.record_y(1:idx),agent0.record_z(1:idx),'--','Color','b', 'linewidth', 4);
h1=plot3(agent1.record_x(1:idx),agent1.record_y(1:idx),agent1.record_z(1:idx),'-','Color',agcolor1, 'linewidth', 2);
h2=plot3(agent2.record_x(1:idx),agent2.record_y(1:idx),agent2.record_z(1:idx),'-','Color',agcolor2, 'linewidth', 2);
h3=plot3(agent3.record_x(1:idx),agent3.record_y(1:idx),agent3.record_z(1:idx),'-','Color',agcolor3, 'linewidth', 2);
[~,obs_num]=size(sim.obstacle_x);
obs1=pbsplot('C',sim.obstacle_x(1,1),sim.obstacle_x(2,1),sim.obstacle_x(3,1),3,10,0.3*ones(1,3));
obs2=pbsplot('C',sim.obstacle_x(1,2),sim.obstacle_x(2,2),sim.obstacle_x(3,2),3,10,0.3*ones(1,3));
index_star=700;
line_color='k';
line_styte=':';
line_w=2;
% text(16, -27,['#t=', num2str(0/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
% text(30, -25, ['#t=', num2str(80/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
% text(20, 24, ['#t=', num2str(700/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
% text(-9.65, 19, ['#t=', num2str(2000/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
% text(-26.8, -8, ['#t=', num2str(1250/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
% text(4.27, 14, ['#t=', num2str(900/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
legend('Virtual leader','Robot 1','Robot 2','Robot 3','Location','northEast','Orientation','Vertical');
function updown = pbsplot(mode,x_p,y_p,z_p,r_p,hgt,col)
if mode=='S'
 [X,Y,Z]=sphere(20);
xp = r_p*X+x_p;
yp = r_p*Y+y_p;
zp = r_p*Z+z_p;
updown(1) = surf(xp,yp,zp,'EdgeColor','none',FaceColor=col);
else
  [x_x1,y_y1,z_z1] = cylinder(1);
updown(1)=surf(r_p*x_x1+x_p,r_p*y_y1+y_p,hgt*z_z1+z_p,'FaceColor',col,'EdgeColor','none'); 
to = linspace(0, 2*pi);
xp = x_p+r_p*cos(to);
yp = y_p+r_p*sin(to);
zp = z_p+zeros(1,length(to));
updown(2)  = patch(xp,yp,  zp,col,'EdgeColor','w');
updown(3)  = patch(xp,yp,  zp+hgt,col,'EdgeColor','w');
end
end
function p2=Aff(mode, theta, a, b,c, xm, ym,zm, p)
if mode=='S'
    A=[a,0,0;0,b,0;0,0,c];
    B=[0;0;0];
elseif mode=='T'
    A=[1,0,0;0,1,0;0,0,1];
    B=[xm;ym;zm];
elseif mode=='R'
    A=[a,0,0;0,b,0;0,0,c] * [cos(-theta),sin(-theta),0;-sin(-theta),cos(-theta),0;0,0,1];
    B=[0;0;0];
elseif mode=='A'
    A=[a,0,0;0,b,0;0,0,c] * [cos(-theta),sin(-theta),0;-sin(-theta),cos(-theta),0;0,0,1];
    B=[xm;ym;ym];
end
p2=A*p+B;
end
function [v_m, jacobi_m] = motion(x_now, x_expected, v_expected, km_gain)
[scale_x, ~]  = size(x_now);
jacobi_m = eye(scale_x, scale_x);
v_m = pinv(jacobi_m) * (v_expected + km_gain .* (x_expected - x_now));
end
function [esm_x_dot, esm_v_dot] = retrunaikx(estimator, A_matrix, B_matrix, esm_p, esm_pv, x0, v0,  agent_id)
[~, cow] =  size(A_matrix);
sumaijx = zeros(3, 1);
sumaijv = zeros(3, 1);
for j = 1:1:cow
    sumaijx = sumaijx  + A_matrix(agent_id, j) .* (esm_p(:, agent_id) - esm_p(:, j)) ;
    sumaijv = sumaijv   + A_matrix(agent_id, j) .* (esm_pv(:, agent_id) - esm_pv(:, j)) ;
end
e1 = return_sign2(sumaijx  +B_matrix(agent_id).*(esm_p(:, agent_id) - x0), estimator.yita1);
e2 = return_sign2(sumaijv  +B_matrix(agent_id).*(esm_pv(:, agent_id) - v0), estimator.yita2);
esm_x_dot = -estimator.lo1 .* (e1) - estimator.lo2 .* sign(sumaijx  +B_matrix(agent_id).*(esm_p(:, agent_id) - x0)) + esm_pv(:, agent_id);
esm_v_dot = -estimator.lo3 .* (e2) - estimator.lo4 .* sign(sumaijv  +B_matrix(agent_id).*(esm_pv(:, agent_id) - v0));
end
function [sign_r1] = return_sign2(e1, r1)
[scale_e, ~] = size(e1);
if scale_e  == 2
    sign_r1= [((abs(e1(1)))^ r1) * sign(e1(1)), ((abs(e1(2)))^ r1) * sign(e1(2))]';
elseif scale_e  == 3
    sign_r1 = [((abs(e1(1)))^ r1) * sign(e1(1)), ((abs(e1(2)))^ r1) * sign(e1(2)), ((abs(e1(3)))^ r1) * sign(e1(3))]';
else
    sign_r1 = ((abs(e1))^ r1) * sign(e1);
end
end
function [v_f, jacobi_f] = PDFBC(param,bar_sigma_f, p_now, sigma_x, pf_i, vf_i,agent_id)
jacobi_f = (1/norm(p_now(:, agent_id) - pf_i - sigma_x(:, agent_id))) * (p_now(:, agent_id) - pf_i - sigma_x(:, agent_id))';
v_f = pinv(jacobi_f) * (param.k_f .* (bar_sigma_f)+ jacobi_f * vf_i );
end
%
function [bar_sigma_f, sigmaf_i, pf_i, vf_i] = return_bar_sigmaf(param, p_now, v_now, p_esm, pv_esm, sigma_x, sigma_v, agent_id)
[~, cow] =  size(param.A_matrix);
sum_aijpj = zeros(3, 1);
sumv_aijpj = zeros(3, 1);
sumaij = 0;
for i = 1:1:cow
    sum_aijpj = sum_aijpj   + param.A_matrix(agent_id, i) .* (p_now(:, i) -  sigma_x(:, i) + sigma_x(:, agent_id)) ;% 可以改为测量值
    sumv_aijpj = sumv_aijpj   + param.A_matrix(agent_id, i) .* (v_now(:, i) -  sigma_v(:, i) + sigma_v(:, agent_id)) ;% 可以改为测量值
    sumaij = sumaij  + param.A_matrix(agent_id, i);
end
pf_i = (1/(param.k1+ param.k2 .* sumaij)) * (param.k1 .* (p_esm(:, agent_id)) + param.k2 .* sum_aijpj);
vf_i = (1/(param.k1+ param.k2 .* sumaij)) * (param.k1 .* (pv_esm(:, agent_id)) + param.k2 .* sumv_aijpj);
sigmaf_i = norm(pf_i + sigma_x(:,agent_id)-p_now(:,agent_id));
bar_sigma_f = 0-sigmaf_i;
end
function [id,pmin_id, min_dis] = returnnearobstacle(p, obs_detect, agent_id)
[~,a_num] = size(obs_detect);
p_id = p(:, agent_id);
min_id  = 1;
min_dis = norm(p_id - obs_detect(:, min_id));
for i = 1:1:a_num
    if i~= min_id
        if norm(p_id - obs_detect(:, i)) <= min_dis
            min_dis = norm(p_id - obs_detect(:, i));
            min_id = i;
        end
    end
end
pmin_id = obs_detect(:, min_id);
id=min_id;
end
function [smrho_oa, jacobi_oa] = returnobstaclejacobi(p, R, r, pmin_obid, min_obdis, agent_id)
if  min_obdis ~= inf
    smrho_oa =  R - R*sin((min_obdis + R - 2*r)*pi/(2*(R-1*r)));
    jacobi_oa = (-R*pi*(p(:, agent_id) - pmin_obid)'/(2*(R-1*r)*min_obdis))...
        *cos(pi*(min_obdis + R -2*r)/(2 * ( R - 1*r )));
else
    smrho_oa  = inf;
    jacobi_oa= zeros(1, 2);
end
if isnan(smrho_oa)
    smrho_oa  = inf;
    jacobi_oa= zeros(1, 2);
end
end
function [T_phai, T_phai_dot] = return_T_phai(phai, phai_dot)
T_phai = [cos(phai), -sin(phai), 0;
    sin(phai), cos(phai), 0;
    0,               0,            1];
T_phai_dot = [-sin(phai)/(cos(phai)^2 + sin(phai)^2),  cos(phai)/(cos(phai)^2 + sin(phai)^2), 0;
    -cos(phai)/(cos(phai)^2 + sin(phai)^2), -sin(phai)/(cos(phai)^2 + sin(phai)^2), 0;
    0,                                      0, 0] .* phai_dot;
end
function [A1, A2, e_ddt] = return_A1A2(tao, x0d_dot, x0d_ddot, e_dot, H_d, M, T_phai,T_phai_dot, J, J_inv, F_f, D_theta, R)
A1 = T_phai * T_phai_dot + D_theta .* (R .^2) * T_phai * J_inv * inv(M) * J * inv(T_phai);
A2 =(T_phai * T_phai_dot + D_theta .* (R .^2) * T_phai * J_inv * inv(M) * J * inv(T_phai)) * x0d_dot + (R.^2) .* T_phai * J_inv * inv(M) * F_f + x0d_ddot;
u = R .* T_phai * J_inv * inv(M) * tao;
kexi = R .* T_phai * J_inv * inv(M) * H_d;
e_ddt = -A1 * e_dot - A2 + u + kexi;
end
function [sign_r1] = return_sign(e1, r1)
sign_r1 = [((abs(e1(1)))^ r1) * sign(e1(1)), ((abs(e1(2)))^ r1) * sign(e1(2)), ((abs(e1(3)))^ r1) * sign(e1(3))]';
end
function [x0_dot, x1_dot] = dynamic_model(tao, x1, H_d , M, T_phai,T_phai_dot, J, J_inv, F_f, D_theta, R)
x0_dot = x1;
g= - (T_phai * T_phai_dot + D_theta .* (R .^2) * T_phai * J_inv * inv(M) * J * inv(T_phai)) * x1 - (R.^2) .* T_phai * J_inv * inv(M) * F_f;
u = R .* T_phai * J_inv * inv(M) * tao;
kexi = R .* T_phai * J_inv * inv(M) * H_d;
x1_dot = u + g + kexi;
end
function v_term = ctr_term(ctr_param, e_rr, e_dot, z2_esm)
rho = ctr_param.rho;
cmu_p = ctr_param.cmu_p;
T = ctr_param.T;
gama = ctr_param.gama;
phai_function = (2 /(rho * cmu_p * T )) * ( 2 * cmu_p*(norm(e_rr)) + (norm(e_rr)).^(-rho) +cmu_p^2 * (norm(e_rr)).^(rho)) * e_rr;
phai_function_dot = (2 /(rho * cmu_p * T )) * ( 2 * cmu_p*(norm(e_rr)) + (norm(e_rr)).^(-rho) +cmu_p^2 * (norm(e_rr)).^(rho)) * e_dot;
s = e_dot + phai_function;
v_term = - (2 /(rho * cmu_p * T )) * (cmu_p +  (0.5 ^ (1-rho/2)) * ((norm(s))^(-rho)) + ...
    cmu_p^2 * (0.5 ^ (1+rho/2)) * ((norm(s))^(rho))) * s - z2_esm - phai_function_dot - (1 / 2) * (1 / (gama ^ 2)) *s ;
end
function [pmin_id, min_dis] = returnnearagent(p, agent_id)
[~,a_num] = size(p);
p_id = p(:, agent_id);
min_id  = agent_id;
min_dis  = inf;
for i = 1:1:a_num
    if i~= agent_id
        if norm(p(:, i) - p_id) <= min_dis
            min_dis = norm(p(:, i) - p_id);
            min_id = i;
        end
    end
end
pmin_id = p(:, min_id);
end
