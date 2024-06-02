% Bi-directional RRT* in 2D

% Clear environment
clear;
clc;
close all;

% Parameters
start = [10, 10];
goal = [90, 90];
step_size = 2;
goal_sample_rate = 0.05;
max_iter = 500;
x_lim = [0, 100];
y_lim = [0, 100];
radius = 5;
delay_time = 0.05; % delay time in seconds

% Obstacle parameters
num_obstacles = 20;
obstacle_width = 10;
obstacle_height = 5;

% Node structure
node_start = struct('pos', start, 'parent', [], 'cost', 0);
node_goal = struct('pos', goal, 'parent', [], 'cost', 0);

% Initialize trees
tree_start = node_start;
tree_goal = node_goal;

% Plotting setup
figure;
hold on;
xlim(x_lim);
ylim(y_lim);
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Generate obstacles
obstacles = struct('center', {}, 'width', {}, 'height', {});
for i = 1:num_obstacles
    obstacle_valid = false;
    while ~obstacle_valid
        new_obstacle_center = [rand * (x_lim(2) - x_lim(1)), rand * (y_lim(2) - y_lim(1))];
        % Check if the new obstacle is not too close to start, goal, or existing obstacles
        if dist(new_obstacle_center, start) > 2 * max(obstacle_width, obstacle_height) && ...
           dist(new_obstacle_center, goal) > 2 * max(obstacle_width, obstacle_height) && ...
           ~any(arrayfun(@(obs) dist(new_obstacle_center, obs.center) <= 2 * max(obstacle_width, obstacle_height), obstacles))
            obstacles(i).center = new_obstacle_center;
            obstacles(i).width = obstacle_width;
            obstacles(i).height = obstacle_height;
            rectangle('Position', [obstacles(i).center(1) - obstacle_width / 2, obstacles(i).center(2) - obstacle_height / 2, obstacle_width, obstacle_height], 'EdgeColor', 'k', 'FaceColor', 'k');
            obstacle_valid = true;
        end
    end
end

% Main loop
path_found = false;
for iter = 1:max_iter
    % Display iteration number
    title(['Iteration: ', num2str(iter)]);
    
    % Random sampling
    if rand < goal_sample_rate
        rand_point = goal;
    else
        rand_point_valid = false;
        while ~rand_point_valid
            rand_point = [rand * (x_lim(2) - x_lim(1)), rand * (y_lim(2) - y_lim(1))];
            % Check if the random point is not within obstacles
            if ~in_collision(rand_point, obstacles)
                rand_point_valid = true;
            end
        end
    end
    
    % Extend start tree
    idx_start = nearest(tree_start, rand_point);
    new_point_start = steer(tree_start(idx_start).pos, rand_point, step_size);
    
    if ~in_collision(new_point_start, obstacles)
        if within_radius(tree_start(idx_start).pos, new_point_start, step_size)
            new_node_start = struct('pos', new_point_start, 'parent', idx_start, 'cost', tree_start(idx_start).cost + dist(tree_start(idx_start).pos, new_point_start));
            tree_start(end + 1) = new_node_start;
            if ~path_found
                plot([tree_start(idx_start).pos(1), new_point_start(1)], [tree_start(idx_start).pos(2), new_point_start(2)], 'b');
                drawnow;
            end
        end
    end
    
    % Extend goal tree
    idx_goal = nearest(tree_goal, rand_point);
    new_point_goal = steer(tree_goal(idx_goal).pos, rand_point, step_size);
    
    if ~in_collision(new_point_goal, obstacles)
        if within_radius(tree_goal(idx_goal).pos, new_point_goal, step_size)
            new_node_goal = struct('pos', new_point_goal, 'parent', idx_goal, 'cost', tree_goal(idx_goal).cost + dist(tree_goal(idx_goal).pos, new_point_goal));
            tree_goal(end + 1) = new_node_goal;
            if ~path_found
                plot([tree_goal(idx_goal).pos(1), new_point_goal(1)], [tree_goal(idx_goal).pos(2), new_point_goal(2)], 'r');
                drawnow;
            end
        end
    end
    
    % Check for connection
    if ~isempty(tree_start) && ~isempty(tree_goal)
        idx_start_new = nearest(tree_start, new_point_goal);
        idx_goal_new = nearest(tree_goal, new_point_start);
        if within_radius(tree_start(idx_start_new).pos, tree_goal(idx_goal_new).pos, step_size)
            disp('Path found!');
            path_found = true;

            % Trace path from start to goal
            path_start = [tree_start(idx_start_new).pos];
            path_goal = [tree_goal(idx_goal_new).pos];

            idx = idx_start_new;
            while ~isempty(tree_start(idx).parent)
                path_start = [tree_start(idx).pos; path_start];
                idx = tree_start(idx).parent;
            end

            idx = idx_goal_new;
            while ~isempty(tree_goal(idx).parent)
                path_goal = [path_goal; tree_goal(idx).pos];
                idx = tree_goal(idx).parent;
            end

            path = [path_start; flipud(path_goal)];
            plot(path(:,1), path(:,2), 'g', 'LineWidth', 2);
            drawnow;

            break;
        end
    end
    
    % Add a delay between iterations
    pause(delay_time);
end

if ~path_found
    disp('Max iterations reached. Path not found.');
end

hold off;

% Function to calculate Euclidean distance
function d = dist(p1, p2)
    d = sqrt(sum((p1 - p2).^2));
end

% Function to find nearest node in the tree
function idx = nearest(tree, point)
    dists = arrayfun(@(node) dist(node.pos, point), tree);
    [~, idx] = min(dists);
end

% Function to steer towards a point
function new_point = steer(from_point, to_point, step_size)
    direction = to_point - from_point;
    distance = norm(direction);
    new_point = from_point + step_size * (direction / distance);
    if distance < step_size
        new_point = to_point;
    end
end

% Function to check if two points are within radius
function result = within_radius(p1, p2, radius)
    result = dist(p1, p2) <= radius;
end

% Function to check if a point is in collision with any obstacles
function result = in_collision(point, obstacles)
    result = any(arrayfun(@(obstacle) in_rectangle(point, obstacle.center, obstacle.width, obstacle.height), obstacles));
end

% Function to check if a point is inside a rectangle
function result = in_rectangle(point, center, width, height)
    result = point(1) >= center(1) - width / 2 && point(1) <= center(1) + width / 2 && ...
             point(2) >= center(2) - height / 2 && point(2) <= center(2) + height / 2;
end
