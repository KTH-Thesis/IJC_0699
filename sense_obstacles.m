function [visible_obstacles, visible_obstacle_points] = sense_obstacles(pose_i, r_i, b_i, obs, obs_c)

  % The agent has a limited spatial sensing range b_i. That is the c_1 condition.
  % The visible_obstacle_points matrix contains also points that are not
  % supposed to be visible to the agent. Discard them by choosing only points
  % whose distance to the agent is less than the distance between the robot and
  % the point where the line connecting the agent and the tangent to the circle
  % meets the agent. That is the c_2 condition.
  visible_obstacle_points = [];
  for i = 1:size(obs,2)
    for j = 1:size(obs{i},2)
      c_1 = (pose_i(1) - obs{i}(1,j))^2 + (pose_i(2) - obs{i}(2,j))^2 <= b_i^2;
      c_2 = (pose_i(1) - obs{i}(1,j))^2 + (pose_i(2) - obs{i}(2,j))^2 <= ...
        (pose_i(1) - obs_c(i,1))^2 + (pose_i(2) - obs_c(i,2))^2 - obs_c(i,3)^2;
      if c_1 && c_2
        visible_obstacle_points = [visible_obstacle_points, obs{i}(:,j)];
      end
    end
  end
  
  visible_obstacle_points = visible_obstacle_points';

  circles = [];
  if size(visible_obstacle_points, 1) > 2

    % Cutoff at at least 2*r_i
    T = clusterdata(visible_obstacle_points , 'linkage', 'centroid', 'criterion', 'distance', 'cutoff', 2*r_i+0.1);
    num_clusters = max(T);

    % init structure. Each cluster will be a circular obstacle
    clusters = cell(1, num_clusters);

    for i = 1:size(T)
      clusters{T(i)} = [clusters{T(i)}, [visible_obstacle_points(i,1); visible_obstacle_points(i,2)]];
    end

%     close all
%     figure
%     hold on
%     grid
%     axis equal
%     for i = 1:num_clusters
%       plot(clusters{i}(1,:), clusters{i}(2,:), 'X')
%       plot(pose_i(1), pose_i(2), 'O')
%       viscircles([pose_i(1), pose_i(2)], b_i, 'EdgeColor', 'r');
%       viscircles([pose_i(1), pose_i(2)], r_i, 'EdgeColor', 'k');
%
%       pause()
%     end

    for i = 1:num_clusters
      [x_c,y_c,r_c] = circle_fit(clusters{i}(1,:), clusters{i}(2,:));
      circle = [x_c,y_c,r_c];
      circles = [circles; circle];
    end
  end

  unique(circles,'rows');

  visible_obstacles = circles;

end
