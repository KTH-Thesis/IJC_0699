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
figure(1)
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

% Distances of agents 1,2,3 to obstacle 1
figure
hold on
plot(sqrt((xX_1(:,1)+(des_1(1)-obs_0(1,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs_0(1,2))*ones(size(tT_1))).^2));
plot(sqrt((xX_2(:,1)+(des_2(1)-obs_0(1,1))*ones(size(tT_2))).^2 + (xX_2(:,2)+(des_2(2)-obs_0(1,2))*ones(size(tT_2))).^2));
plot(sqrt((xX_3(:,1)+(des_3(1)-obs_0(1,1))*ones(size(tT_3))).^2 + (xX_3(:,2)+(des_3(2)-obs_0(1,2))*ones(size(tT_3))).^2));
grid
constr_x = [0, 100];
constr_y = [1.5, 1.5];
plot(constr_x, constr_y, 'Color', 'c')
axis([0 100 1.2 7])


% Distances of agents 1,2,3 to obstacle 2
figure
hold on
plot(sqrt((xX_1(:,1)+(des_1(1)-obs_0(2,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs_0(2,2))*ones(size(tT_1))).^2));
plot(sqrt((xX_2(:,1)+(des_2(1)-obs_0(2,1))*ones(size(tT_2))).^2 + (xX_2(:,2)+(des_2(2)-obs_0(2,2))*ones(size(tT_2))).^2));
plot(sqrt((xX_3(:,1)+(des_3(1)-obs_0(2,1))*ones(size(tT_3))).^2 + (xX_3(:,2)+(des_3(2)-obs_0(2,2))*ones(size(tT_3))).^2));
grid
constr_x = [0, 100];
constr_y = [1.5, 1.5];
plot(constr_x, constr_y, 'Color', 'c')
axis([0 100 1.2 7])

% Distance of agent 1 to agent 2
figure
hold on
constr_x = [0, 100];
constr_y = [1.0, 1.0];
plot(constr_x, constr_y, 'Color', 'c')
constr_x = [0, 100];
constr_y = [2.0, 2.0];
plot(constr_x, constr_y, 'Color', 'c')
axis([0 100 0.8 2.2])
grid
plot(...
  sqrt(...
    (xX_1(:,1) - xX_2(:,1)+ (des_1(1) - des_2(1))*ones(size(tT_1))).^2 + ...
    (xX_1(:,2) - xX_2(:,2)+ (des_1(2) - des_2(2))*ones(size(tT_1))).^2 ...
  ), 'Color', 'b' ...
);

% Distance of agent 1 to agent 3
figure
hold on
constr_x = [0, 100];
constr_y = [1.01, 1.01];
plot(constr_x, constr_y, 'Color', 'c')
constr_x = [0, 100];
constr_y = [2.0, 2.0];
plot(constr_x, constr_y, 'Color', 'c')
axis([0 100 0.8 2.2])
grid
plot(...
  sqrt(...
    (xX_1(:,1) - xX_3(1:end-1,1)+ (des_1(1) - des_3(1))*ones(size(tT_1))).^2 + ...
    (xX_1(:,2) - xX_3(1:end-1,2)+ (des_1(2) - des_3(2))*ones(size(tT_1))).^2 ...
  ), 'Color', 'b' ...
);

% Distance of agent 2 to agent 3
figure
hold on
constr_x = [0, 100];
constr_y = [1.0, 1.0];
plot(constr_x, constr_y, 'Color', 'c')
grid
plot(...
  sqrt(...
    (xX_2(:,1) - xX_3(1:end-1,1)+ (des_2(1) - des_3(1))*ones(size(tT_2))).^2 + ...
    (xX_2(:,2) - xX_3(1:end-1,2)+ (des_2(2) - des_3(2))*ones(size(tT_2))).^2 ...
  ), 'Color', 'b' ...
);

% errors
figure
plot(xX_1)
grid

figure
plot(xX_2)
grid

figure
plot(xX_3)
grid

% Inputs
figure
hold on
constr_x = [0, 100];
constr_y = [u_min, u_min];
plot(constr_x, constr_y, 'Color', 'c')
constr_x = [0, 100];
constr_y = [u_max, u_max];
plot(constr_x, constr_y, 'Color', 'c')
plot(uU_1(1,:), 'Color', [0    0.4470    0.7410])
plot(uU_1(2,:), 'Color',  [0.8500    0.3250    0.0980])
grid
axis([0 100 -11 11])

figure
hold on
constr_x = [0, 100];
constr_y = [u_min, u_min];
plot(constr_x, constr_y, 'Color', 'c')
constr_x = [0, 100];
constr_y = [u_max, u_max];
plot(constr_x, constr_y, 'Color', 'c')
plot(uU_2(1,:), 'Color', [0    0.4470    0.7410])
plot(uU_2(2,:), 'Color', [ 0.8500    0.3250    0.0980])
grid
axis([0 30 -11 11])


figure
hold on
constr_x = [0, 100];
constr_y = [u_min, u_min];
plot(constr_x, constr_y, 'Color', 'c')
constr_x = [0, 100];
constr_y = [u_max, u_max];
plot(constr_x, constr_y, 'Color', 'c')
plot(uU_3(1,:), 'Color', [0    0.4470    0.7410])
plot(uU_3(2,:), 'Color', [ 0.8500    0.3250    0.0980])
grid
axis([0 100 -11 11])


% V
V_1 = zeros(size(xX_1,1), 1);
V_2 = zeros(size(xX_2,1), 1);
V_3 = zeros(size(xX_2,1), 1);
for i = 1:size(xX_1,1)
  V_1(i) = xX_1(i,:) * P * xX_1(i,:)';
  V_2(i) = xX_2(i,:) * P * xX_2(i,:)';
  V_3(i) = xX_3(i,:) * P * xX_3(i,:)';
end

figure
plot(V_1)
hold on
plot(V_2)
plot(V_3)
grid

figure
plot(V_1)
hold on
plot(V_2)
plot(V_3)
constr_x = [0, 1000];
constr_y = [epsilon_psi, epsilon_psi];
plot(constr_x, constr_y, 'Color', 'm')
constr_x = [0, 1000];
constr_y = [epsilon_omega, epsilon_omega];
plot(constr_x, constr_y, 'Color', 'c')
grid
axis([1 size(V_1,1) 0 epsilon_psi])
