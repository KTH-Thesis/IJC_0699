close all
clear all
load('tT_1.mat')
load('tT_2.mat')
load('xX_1.mat')
load('xX_2.mat')
load('uU_1.mat')
load('uU_2.mat')
load('uU_3.mat')
load('xX_3.mat')
load('tT_3.mat')

%--------------------------------------------------------------------------
% V -----------------------------------------------------------------------
%--------------------------------------------------------------------------
V_1 = zeros(size(xX_1,1), 1);
V_2 = zeros(size(xX_2,1), 1);
V_3 = zeros(size(xX_3,1), 1);
for i = 1:size(xX_1,1)
  V_1(i) = xX_1(i,:) * P * xX_1(i,:)';
  V_2(i) = xX_2(i,:) * P * xX_2(i,:)';
  V_3(i) = xX_3(i,:) * P * xX_3(i,:)';
end

figure
hold on
grid

v1 = plot(tT_1(1:100), V_1(1:100), 'LineWidth', 2, 'color', 'r');
v2 = plot(tT_2(1:100), V_2(1:100), 'LineWidth', 2, 'color', 'b');
v3 = plot(tT_3(1:100), V_3(1:100), 'LineWidth', 2, 'color', 'g');

title('$\rm P-weighted \ norms \ of \ the \ error \ signals \ of \ the \ 3 \ agents$','interpreter','latex','fontsize',16);
xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
ylabel({'$V_i(t) = e_i^T P e_i, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
legend([v1, v2, v3], {'$V_1(t)$','$V_2(t)$', '$V_3(t)$'},'interpreter','latex','fontsize',16);

%--------------------------------------------------------------------------
figure
hold on
grid

V1 = plot(tT_1, V_1, 'LineWidth', 2, 'color', 'r');
V2 = plot(tT_2, V_2, 'LineWidth', 2, 'color', 'b');
V3 = plot(tT_3, V_3, 'LineWidth', 2, 'color', 'g');
V4 = plot(tT_1, epsilon_omega*ones(length(tT_1),1), 'LineWidth', 2, 'color', 'c');

title('$\rm P-weighted \ norms \ of \ the \ error \ signals \ of \ the \ 3 \ agents$','interpreter','latex','fontsize',16);
xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
ylabel({'$V_i(t) = e_i^T P e_i, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
axis([0.0 100 0 epsilon_omega+0.001]);
legend([V1, V2, V3, V4], {'$V_1(t)$','$V_2(t)$', '$V_3(t)$', '$\varepsilon_{\Omega}$'},'interpreter','latex','fontsize',16);


% limit everything to 10 sec

tT_1 = tT_1(2:100);
tT_2 = tT_2(2:100);
tT_3 = tT_3(2:100);

xX_1 = xX_1(2:100,:);
xX_2 = xX_2(2:100,:);
xX_3 = xX_3(2:100,:);

uU_1 = uU_1(:,2:100);
uU_2 = uU_2(:,2:100);
uU_3 = uU_3(:,2:100);

figure
hold on
grid
axis([-7 7 0 5])
axis equal
filename = 'trajectories.gif';
visualize_trajectories = false;
if visualize_trajectories
  for i=1:size(tT_1,1)

    plot(des_1(1), des_1(2), 'X', 'Color', 'b')
    plot(des_2(1), des_2(2), 'X', 'Color', 'r')
    plot(des_3(1), des_3(2), 'X', 'Color', 'g')
  
    % Plot agents as circles
    viscircles([xX_1(i,1) + des_1(1),  xX_1(i,2) + des_1(2)], r(1), 'EdgeColor', 'b');
    viscircles([xX_2(i,1) + des_2(1),  xX_2(i,2) + des_2(2)], r(2), 'EdgeColor', 'r');
    viscircles([xX_3(i,1) + des_3(1),  xX_3(i,2) + des_3(2)], r(3), 'EdgeColor', 'g');
    
    % Plot orientation of agents
    or_1 = xX_1(i,3) + des_1(3);
    fx_1 = xX_1(i,1) + des_1(1) + r(1) * cos(or_1);
    fy_1 = xX_1(i,2) + des_1(2) + r(1) * sin(or_1);
    li_1 = [xX_1(i,1) + des_1(1), xX_1(i,2) + des_1(2)];
    li_1 = [li_1; [fx_1, fy_1]];
    plot(li_1(:,1), li_1(:,2))
    
    or_2 = xX_2(i,3) + des_2(3);
    fx_2 = xX_2(i,1) + des_2(1) + r(1) * cos(or_2);
    fy_2 = xX_2(i,2) + des_2(2) + r(1) * sin(or_2);
    li_2 = [xX_2(i,1) + des_2(1), xX_2(i,2) + des_2(2)];
    li_2 = [li_2; [fx_2, fy_2]];
    plot(li_2(:,1), li_2(:,2))
    
    or_3 = xX_3(i,3) + des_3(3);
    fx_3 = xX_3(i,1) + des_3(1) + r(1) * cos(or_3);
    fy_3 = xX_3(i,2) + des_3(2) + r(1) * sin(or_3);
    li_3 = [xX_3(i,1) + des_3(1), xX_3(i,2) + des_3(2)];
    li_3 = [li_3; [fx_3, fy_3]];
    plot(li_3(:,1), li_3(:,2))
    

    viscircles([obs_0(1,1), obs_0(1,2)], obs_0(1,3), 'LineStyle', ':', 'EdgeColor', 'k', 'LineWidth', 1);
    viscircles([obs_0(2,1), obs_0(2,2)], obs_0(2,3), 'LineStyle', ':', 'EdgeColor', 'k', 'LineWidth', 1);

    % plot visible obstacle points
    if size(visible_obstacle_points_all_1{i},1) > 0
      plot(visible_obstacle_points_all_1{i}(:,1), visible_obstacle_points_all_1{i}(:,2), '.', 'Color', 'b');
    end

    if size(visible_obstacle_points_all_2{i},1) > 0
      plot(visible_obstacle_points_all_2{i}(:,1), visible_obstacle_points_all_2{i}(:,2), '.', 'Color', 'r');
    end

    if size(visible_obstacle_points_all_3{i},1) > 0
      plot(visible_obstacle_points_all_3{i}(:,1), visible_obstacle_points_all_3{i}(:,2), '.', 'Color', 'g');
    end

    pause()

%     drawnow
  %   frame = getframe(1);
  %   im = frame2im(frame);
  %   [imind,cm] = rgb2ind(im,256);
  %   if i == 1;
  %    imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
  %   else
  %    imwrite(imind,cm,filename,'gif','WriteMode','append');
  %   end
    cla
  end
end


%--------------------------------------------------------------------------
% Distances of agents 1,2,3 to obstacle 1 ---------------------------------
%--------------------------------------------------------------------------
figure
hold on
grid

dist_ag1_obs1 = sqrt((xX_1(:,1)+(des_1(1)-obs_0(1,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs_0(1,2))*ones(size(tT_1))).^2);
dist_ag2_obs1 = sqrt((xX_2(:,1)+(des_2(1)-obs_0(1,1))*ones(size(tT_2))).^2 + (xX_2(:,2)+(des_2(2)-obs_0(1,2))*ones(size(tT_2))).^2);
dist_ag3_obs1 = sqrt((xX_3(:,1)+(des_3(1)-obs_0(1,1))*ones(size(tT_3))).^2 + (xX_3(:,2)+(des_3(2)-obs_0(1,2))*ones(size(tT_3))).^2);

h1 = plot(tT_1, dist_ag1_obs1, 'LineWidth', 2, 'color', 'r');
h2 = plot(tT_2, dist_ag2_obs1, 'LineWidth', 2, 'color', 'b');
h3 = plot(tT_3, dist_ag3_obs1, 'LineWidth', 2, 'color', 'g');
h4 = plot(tT_1, 1.46*ones(length(tT_1),1), 'color', 'c');

title('$\rm Distance \ between \ agents \ and \ obstacle \ 1$','interpreter','latex','fontsize',16);
ylabel({'$\|p_i(t)-p_{\scriptscriptstyle O_1}\|, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
legend([h1 h2 h3 h4], {'$\|p_1(t)-p_{\scriptscriptstyle O_1}\|$','$\|p_2(t)-p_{\scriptscriptstyle O_1}\|$', '$\|p_3(t)-p_{\scriptscriptstyle O_1}\|$', '$r_i+r_{\scriptscriptstyle O_1}+\varepsilon$'},'interpreter','latex','fontsize',16);


%--------------------------------------------------------------------------
% Distances of agents 1,2,3 to obstacle 2 ---------------------------------
%--------------------------------------------------------------------------
figure
hold on
grid

dist_ag1_obs2 = sqrt((xX_1(:,1)+(des_1(1)-obs_0(2,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs_0(2,2))*ones(size(tT_1))).^2);
dist_ag2_obs2 = sqrt((xX_2(:,1)+(des_2(1)-obs_0(2,1))*ones(size(tT_2))).^2 + (xX_2(:,2)+(des_2(2)-obs_0(2,2))*ones(size(tT_2))).^2);
dist_ag3_obs2 = sqrt((xX_3(:,1)+(des_3(1)-obs_0(2,1))*ones(size(tT_3))).^2 + (xX_3(:,2)+(des_3(2)-obs_0(2,2))*ones(size(tT_3))).^2);

h1 = plot(tT_1, dist_ag1_obs2, 'LineWidth', 2, 'color', 'r');
h2 = plot(tT_2, dist_ag2_obs2, 'LineWidth', 2, 'color', 'b');
h3 = plot(tT_3, dist_ag3_obs2, 'LineWidth', 2, 'color', 'g');
h4 = plot(tT_1, 1.46*ones(length(tT_1),1), 'color', 'c');

title('$\rm Distance \ between \ agents \ and \ obstacle \ 2$','interpreter','latex','fontsize',16);
ylabel({'$\|p_i(t)-p_{\scriptscriptstyle O_2}\|, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
legend([h1 h2 h3 h4], {'$\|p_1(t)-p_{\scriptscriptstyle O_2}\|$','$\|p_2(t)-p_{\scriptscriptstyle O_2}\|$', '$\|p_3(t)-p_{\scriptscriptstyle O_2}\|$', '$r_i+r_{\scriptscriptstyle O_2}+\varepsilon$'},'interpreter','latex','fontsize',16);




%--------------------------------------------------------------------------
% Distances of agent 1-2 and 1-3 ------------------------------------------
%--------------------------------------------------------------------------
figure
hold on
grid

dist_ag1_ag2 = sqrt((xX_1(:,1) - xX_2(:,1)+ (des_1(1) - des_2(1))*ones(size(tT_1))).^2 + ...
    (xX_1(:,2) - xX_2(:,2)+ (des_1(2) - des_2(2))*ones(size(tT_1))).^2);
dist_ag1_ag3 = sqrt((xX_1(:,1) - xX_3(:,1)+ (des_1(1) - des_3(1))*ones(size(tT_1))).^2 + ...
  (xX_1(:,2) - xX_3(:,2)+ (des_1(2) - des_3(2))*ones(size(tT_1))).^2);
  
h1 = plot(tT_1, dist_ag1_ag2, 'LineWidth', 2, 'color', 'b');
h2 = plot(tT_1, dist_ag1_ag3, 'LineWidth', 2, 'color', 'g');
h3 = plot(tT_1, 1.0*ones(length(tT_1),1), 'color', 'c');
h4 = plot(tT_1, 2.0*ones(length(tT_1),1), 'color', 'y');

axis([0 10 0.9 2.1])
title('$\rm Distance \ between \ agents \ 1-2 \ and \ 1-3$','interpreter','latex','fontsize',16);
ylabel({'$\|p_i(t)-p_{j}(t)\|, i \in \{1,2,3\}, j \in \mathcal{N}_i$'},'interpreter','latex','fontsize',16);
legend([h1 h2 h3 h4], {'$\|p_1(t)-p_{2}(t)\|$','$\|p_1(t)-p_{3}(t)\|$', '$r_i+r_j+\varepsilon$', '$d_i - \varepsilon$'},'interpreter','latex','fontsize',16);

%--------------------------------------------------------------------------
% Distance of agent 2 to agent 3 ------------------------------------------
%--------------------------------------------------------------------------
% figure
% hold on
% constr_x = [0, 10];
% constr_y = [1.0, 1.0];
% plot(constr_x, constr_y, 'Color', 'c')
% grid
% dist_ag2_ag3 = sqrt((xX_2(:,1) - xX_3(:,1)+ (des_2(1) - des_3(1))*ones(size(tT_2))).^2 + ...
%     (xX_2(:,2) - xX_3(:,2)+ (des_2(2) - des_3(2))*ones(size(tT_2))).^2);
% plot(tT_2, dist_ag2_ag3, 'Color', 'b');


%--------------------------------------------------------------------------
% errors ------------------------------------------------------------------
%--------------------------------------------------------------------------
norm_error = zeros(length(tT_1),3);
for i = 1:length(tT_1)
   norm_error(i,1) = norm(xX_1(i,:));
   norm_error(i,2) = norm(xX_2(i,:));
   norm_error(i,3) = norm(xX_3(i,:));
end

figure
hold on
grid

e1 = plot(tT_1, norm_error(:,1), 'LineWidth', 2, 'color', 'r');
e2 = plot(tT_2, norm_error(:,2), 'LineWidth', 2, 'color', 'b');
e3 = plot(tT_3, norm_error(:,3), 'LineWidth', 2, 'color', 'g');

title('$\rm Error \ signals \ of \ the \ 3 \ agents$','interpreter','latex','fontsize',16);
xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
ylabel({'$\|e_i(t)\|, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
axis([0.0 10 -1 14]);
legend([e1, e2, e3], {'$\|e_1(t)\|$','$\|e_2(t)\|$', '$\|e_3(t)\|$'},'interpreter','latex','fontsize',16);


%--------------------------------------------------------------------------
% Inputs ------------------------------------------------------------------
%--------------------------------------------------------------------------

norm_control = zeros(length(tT_1),2);

for i = 1:length(tT_1)
   norm_control(i,1) = norm(uU_1(:,i), 2);
   norm_control(i,2) = norm(uU_2(:,i), 2);
   norm_control(i,3) = norm(uU_3(:,i), 2);
end

figure
hold on
grid

u1 = plot(tT_1, norm_control(:,1), 'LineWidth', 2, 'color', 'r');
u2 = plot(tT_2, norm_control(:,2), 'LineWidth', 2, 'color', 'b');
u3 = plot(tT_3, norm_control(:,3), 'LineWidth', 2, 'color', 'g');
u4 = plot(tT_1, (sqrt(2)*u_abs)*ones(length(tT_1),1), 'LineWidth', 2, 'color', 'c');

title('$\rm Control \ inputs $','interpreter','latex','fontsize',16);
xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
ylabel({'$\|u_i(t)\|, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
axis([0.0 10 -1 sqrt(2)*u_abs+1]);
legend([u1 u2 u3 u4], {'$\|u_1(t)\|$','$\|u_2(t)\|$', '$\|u_3(t)\|$', '$\overline{u}_i$'},'interpreter','latex','fontsize',16);