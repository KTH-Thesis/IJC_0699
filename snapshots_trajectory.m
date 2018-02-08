close all
clear all
load('xX_1.mat')
load('xX_2.mat')
load('xX_3.mat')
load('tT_1.mat')
load('tT_2.mat')
load('tT_3.mat')
load('uU_1.mat')
load('uU_2.mat')
load('uU_3.mat')


% Split the visible obstacle points per obstacle
visible_obstacle_points_up_1 = cell(1,1000);
visible_obstacle_points_dn_1 = cell(1,1000);

for i=1:size(visible_obstacle_points_all_1,2)
  for j=1:size(visible_obstacle_points_all_1{i},1)
    if visible_obstacle_points_all_1{i}(j,2) > 4
      visible_obstacle_points_up_1{i} = [visible_obstacle_points_up_1{i}; visible_obstacle_points_all_1{i}(j,:)];
    else
      visible_obstacle_points_dn_1{i} = [visible_obstacle_points_dn_1{i}; visible_obstacle_points_all_1{i}(j,:)];
    end
  end
end
visible_obstacle_points_up_2 = cell(1,1000);
visible_obstacle_points_dn_2 = cell(1,1000);

for i=1:size(visible_obstacle_points_all_2,2)
  for j=1:size(visible_obstacle_points_all_2{i},1)
    if visible_obstacle_points_all_2{i}(j,2) > 4
      visible_obstacle_points_up_2{i} = [visible_obstacle_points_up_2{i}; visible_obstacle_points_all_2{i}(j,:)];
    else
      visible_obstacle_points_dn_2{i} = [visible_obstacle_points_dn_2{i}; visible_obstacle_points_all_2{i}(j,:)];
    end
  end
end
visible_obstacle_points_up_3 = cell(1,1000);
visible_obstacle_points_dn_3 = cell(1,1000);

for i=1:size(visible_obstacle_points_all_3,2)
  for j=1:size(visible_obstacle_points_all_3{i},1)
    if visible_obstacle_points_all_3{i}(j,2) > 4
      visible_obstacle_points_up_3{i} = [visible_obstacle_points_up_3{i}; visible_obstacle_points_all_3{i}(j,:)];
    else
      visible_obstacle_points_dn_3{i} = [visible_obstacle_points_dn_3{i}; visible_obstacle_points_all_3{i}(j,:)];
    end
  end
end

frames = [1,3,6,7,8,10,13,118];
counter = 1;


for i = 1:size(frames,2)
  
  subplot(4,2,counter)
  hold on
 
  axis([-9 9 0 6])
  axis equal
   
  plot(des_1(1), des_1(2), 'X', 'Color', 'r')
  plot(des_2(1), des_2(2), 'X', 'Color', 'b')
  plot(des_3(1), des_3(2), 'X', 'Color', 'g')
    
  plot(xX_1(1,1) + des_1(1), xX_1(1,2) + des_1(2), '.', 'Color', 'r', 'MarkerSize', 2);
  plot(xX_2(1,1) + des_2(1), xX_2(1,2) + des_2(2), '.', 'Color', 'b', 'MarkerSize', 2);
  plot(xX_3(1,1) + des_3(1), xX_3(1,2) + des_3(2), '.', 'Color', 'g', 'MarkerSize', 2);
  
  viscircles([xX_1(frames(i),1) + des_1(1),  xX_1(frames(i),2) + des_1(2)], r(1), 'EdgeColor', 'r', 'LineWidth', 2);
  viscircles([xX_2(frames(i),1) + des_2(1),  xX_2(frames(i),2) + des_2(2)], r(2), 'EdgeColor', 'b', 'LineWidth', 2);
  viscircles([xX_3(frames(i),1) + des_3(1),  xX_3(frames(i),2) + des_3(2)], r(3), 'EdgeColor', 'g', 'LineWidth', 2);
  
  % Plot b-ranges
%   viscircles([xX_1(frames(i),1) + des_1(1),  xX_1(frames(i),2) + des_1(2)], b(1), 'EdgeColor', 'r', 'LineWidth', 2);
%   viscircles([xX_2(frames(i),1) + des_2(1),  xX_2(frames(i),2) + des_2(2)], b(2), 'EdgeColor', 'b', 'LineWidth', 2);
%   viscircles([xX_3(frames(i),1) + des_3(1),  xX_3(frames(i),2) + des_3(2)], b(3), 'EdgeColor', 'g', 'LineWidth', 2);
  
  if i < size(frames,2)
    centers_connector_x = [xX_1(frames(i),1) + des_1(1), xX_2(frames(i),1) + des_2(1)];
    centers_connector_y = [xX_1(frames(i),2) + des_1(2), xX_2(frames(i),2) + des_2(2)];
    plot(centers_connector_x, centers_connector_y, 'Color', 'k', 'LineWidth', 1)

    centers_connector_x = [xX_1(frames(i),1) + des_1(1), xX_3(frames(i),1) + des_3(1)];
    centers_connector_y = [xX_1(frames(i),2) + des_1(2), xX_3(frames(i),2) + des_3(2)];
    plot(centers_connector_x, centers_connector_y, 'Color', 'k', 'LineWidth', 1)
  end
  
  % Plot orientation of agents
  
%   or_1 = xX_1(frames(i),3) + des_1(3);
%   fx_1 = xX_1(frames(i),1) + des_1(1) + r(1) * cos(or_1);
%   fy_1 = xX_1(frames(i),2) + des_1(2) + r(1) * sin(or_1);
%   li_1 = [xX_1(frames(i),1) + des_1(1), xX_1(frames(i),2) + des_1(2)];
%   li_1 = [li_1; [fx_1, fy_1]];
%   plot(li_1(:,1), li_1(:,2), 'Color', 'r')
% 
%   or_2 = xX_2(frames(i),3) + des_2(3);
%   fx_2 = xX_2(frames(i),1) + des_2(1) + r(1) * cos(or_2);
%   fy_2 = xX_2(frames(i),2) + des_2(2) + r(1) * sin(or_2);
%   li_2 = [xX_2(frames(i),1) + des_2(1), xX_2(frames(i),2) + des_2(2)];
%   li_2 = [li_2; [fx_2, fy_2]];
%   plot(li_2(:,1), li_2(:,2), 'Color', 'b')
% 
%   or_3 = xX_3(frames(i),3) + des_3(3);
%   fx_3 = xX_3(frames(i),1) + des_3(1) + r(1) * cos(or_3);
%   fy_3 = xX_3(frames(i),2) + des_3(2) + r(1) * sin(or_3);
%   li_3 = [xX_3(frames(i),1) + des_3(1), xX_3(frames(i),2) + des_3(2)];
%   li_3 = [li_3; [fx_3, fy_3]];
%   plot(li_3(:,1), li_3(:,2), 'Color', 'g')
  
  
  viscircles([obs_0(1,1), obs_0(1,2)], obs_0(1,3), 'LineStyle', ':', 'EdgeColor', 'k', 'LineWidth', 1);
  viscircles([obs_0(2,1), obs_0(2,2)], obs_0(2,3), 'LineStyle', ':', 'EdgeColor', 'k', 'LineWidth', 1);
  
  %% Plot visible obstacle points
  %------------------------------------------------------------------------
  if counter == 1 || counter == 2 
    if size(visible_obstacle_points_all_2{frames(i)},1) > 0
      plot(visible_obstacle_points_all_2{frames(i)}(:,1), visible_obstacle_points_all_2{frames(i)}(:,2), '.', 'Color', 'b', 'MarkerSize', 1);
    end

    if size(visible_obstacle_points_all_3{frames(i)},1) > 0
      plot(visible_obstacle_points_all_3{frames(i)}(:,1), visible_obstacle_points_all_3{frames(i)}(:,2), '.', 'Color', 'g', 'MarkerSize', 1);
    end

    if size(visible_obstacle_points_all_1{frames(i)},1) > 0
      plot(visible_obstacle_points_all_1{frames(i)}(:,1), visible_obstacle_points_all_1{frames(i)}(:,2), '.', 'Color', 'r', 'MarkerSize', 1);
    end
  elseif counter == 3
       
    if size(visible_obstacle_points_up_2{frames(i)},1) > 0
      plot(visible_obstacle_points_up_2{frames(i)}(:,1), visible_obstacle_points_up_2{frames(i)}(:,2), '.', 'Color', 'b', 'MarkerSize', 1);
    end
    if size(visible_obstacle_points_up_1{frames(i)},1) > 0
      plot(visible_obstacle_points_up_1{frames(i)}(:,1), visible_obstacle_points_up_1{frames(i)}(:,2), '.', 'Color', 'r', 'MarkerSize', 1);
    end
    if size(visible_obstacle_points_up_3{frames(i)},1) > 0
      plot(visible_obstacle_points_up_3{frames(i)}(:,1), visible_obstacle_points_up_3{frames(i)}(:,2), '.', 'Color', 'g', 'MarkerSize', 1);
    end
    
    
    if size(visible_obstacle_points_dn_3{frames(i)},1) > 0
      plot(visible_obstacle_points_dn_3{frames(i)}(:,1), visible_obstacle_points_dn_3{frames(i)}(:,2), '.', 'Color', 'g', 'MarkerSize', 1);
    end    
    if size(visible_obstacle_points_dn_1{frames(i)},1) > 0
      plot(visible_obstacle_points_dn_1{frames(i)}(:,1), visible_obstacle_points_dn_1{frames(i)}(:,2), '.', 'Color', 'r', 'MarkerSize', 1);
    end
    if size(visible_obstacle_points_dn_2{frames(i)},1) > 0
      plot(visible_obstacle_points_dn_2{frames(i)}(:,1), visible_obstacle_points_dn_2{frames(i)}(:,2), '.', 'Color', 'b', 'MarkerSize', 1);
    end
    
  elseif counter == 4
    
    
    if size(visible_obstacle_points_up_2{frames(i)},1) > 0
      plot(visible_obstacle_points_up_2{frames(i)}(:,1), visible_obstacle_points_up_2{frames(i)}(:,2), '.', 'Color', 'b', 'MarkerSize', 1);
    end
    if size(visible_obstacle_points_up_1{frames(i)},1) > 0
      plot(visible_obstacle_points_up_1{frames(i)}(:,1), visible_obstacle_points_up_1{frames(i)}(:,2), '.', 'Color', 'r', 'MarkerSize', 1);
    end
    if size(visible_obstacle_points_up_3{frames(i)},1) > 0
      plot(visible_obstacle_points_up_3{frames(i)}(:,1), visible_obstacle_points_up_3{frames(i)}(:,2), '.', 'Color', 'g', 'MarkerSize', 1);
    end
    
    
    if size(visible_obstacle_points_dn_3{frames(i)},1) > 0
      plot(visible_obstacle_points_dn_3{frames(i)}(:,1), visible_obstacle_points_dn_3{frames(i)}(:,2), '.', 'Color', 'g', 'MarkerSize', 1);
    end    
    
    if size(visible_obstacle_points_dn_2{frames(i)},1) > 0
      plot(visible_obstacle_points_dn_2{frames(i)}(:,1), visible_obstacle_points_dn_2{frames(i)}(:,2), '.', 'Color', 'b', 'MarkerSize', 1);
    end  
    if size(visible_obstacle_points_dn_1{frames(i)},1) > 0
      plot(visible_obstacle_points_dn_1{frames(i)}(:,1), visible_obstacle_points_dn_1{frames(i)}(:,2), '.', 'Color', 'r', 'MarkerSize', 1);
    end
    

    
    
    
    
  elseif counter == 5
    if size(visible_obstacle_points_all_2{frames(i)},1) > 0
      plot(visible_obstacle_points_all_2{frames(i)}(:,1), visible_obstacle_points_all_2{frames(i)}(:,2), '.', 'Color', 'b', 'MarkerSize', 1);
    end
    if size(visible_obstacle_points_all_1{frames(i)},1) > 0
      plot(visible_obstacle_points_all_1{frames(i)}(:,1), visible_obstacle_points_all_1{frames(i)}(:,2), '.', 'Color', 'r', 'MarkerSize', 1);
    end
    if size(visible_obstacle_points_all_3{frames(i)},1) > 0
      plot(visible_obstacle_points_all_3{frames(i)}(:,1), visible_obstacle_points_all_3{frames(i)}(:,2), '.', 'Color', 'g', 'MarkerSize', 1);
    end
  elseif counter == 6
    if size(visible_obstacle_points_all_2{frames(i)},1) > 0
      plot(visible_obstacle_points_all_2{frames(i)}(:,1), visible_obstacle_points_all_2{frames(i)}(:,2), '.', 'Color', 'b', 'MarkerSize', 1);
    end

    if size(visible_obstacle_points_all_3{frames(i)},1) > 0
      plot(visible_obstacle_points_all_3{frames(i)}(:,1), visible_obstacle_points_all_3{frames(i)}(:,2), '.', 'Color', 'g', 'MarkerSize', 1);
    end

    if size(visible_obstacle_points_all_1{frames(i)},1) > 0
      plot(visible_obstacle_points_all_1{frames(i)}(:,1), visible_obstacle_points_all_1{frames(i)}(:,2), '.', 'Color', 'r', 'MarkerSize', 1);
    end
  elseif counter == 7 || counter == 8
    if size(visible_obstacle_points_all_3{frames(i)},1) > 0
      plot(visible_obstacle_points_all_3{frames(i)}(:,1), visible_obstacle_points_all_3{frames(i)}(:,2), '.', 'Color', 'g', 'MarkerSize', 1);
    end

    if size(visible_obstacle_points_all_1{frames(i)},1) > 0
      plot(visible_obstacle_points_all_1{frames(i)}(:,1), visible_obstacle_points_all_1{frames(i)}(:,2), '.', 'Color', 'r', 'MarkerSize', 1);
    end
    
    if size(visible_obstacle_points_all_2{frames(i)},1) > 0
      plot(visible_obstacle_points_all_2{frames(i)}(:,1), visible_obstacle_points_all_2{frames(i)}(:,2), '.', 'Color', 'b', 'MarkerSize', 1);
    end
  end
  
  
 
  

  counter = counter + 1;
end


% matlab2tikz('trajectory_d_OFF_3_2.tex')