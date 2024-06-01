% Bi-directional RRT* in 2D

% Clear environment
clear;
clc;
close all;

% Parameters
start = [10, 10];
goal = [90, 90];
step_size = 5;
goal_sample_rate = 0.05;
max_iter = 500;
x_lim = [0, 100];
y_lim = [0, 100];
radius = 10;
delay_time = 0.05; % delay time in seconds

% Obstacle parameters
num_obstacles = 10;
obstacle_radius = 5;

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
obstacles = struct('center', {}, 'radius', {});
for i = 1:num_obstacles
    obstacles(i).center = [rand * (x_lim(2) - x_lim(1)), rand * (y_lim(2) - y_lim(1))];
    obstacles(i).radius = obstacle_radius;
    viscircles(obstacles(i).center, obstacles(i).radius, 'EdgeColor', 'k');
end

% Main loop
for i = 1:max_iter
    % Display iteration number
    title(['Iteration: ', num2str(i)]);
    
    % Random sampling
    if rand < goal_sample_rate
        rand_point = goal;
    else
        rand_point = [rand * (x_lim(2) - x_lim(1)), rand * (y_lim(2) - y_lim(1))];
    end
    
    % Extend start tree
    idx_start = nearest(tree_start, rand_point);
    new_point_start = steer(tree_start(idx_start).pos, rand_point, step_size);
    
    % Check for collisions with obstacles
    if ~in_collision(new_point_start, obstacles)
        % Extend goal tree
        idx_goal = nearest(tree_goal, new_point_start);
        new_point_goal = steer(tree_goal(idx_goal).pos, new_point_start, step_size);
        
        if ~in_collision(new_point_goal, obstacles)
            % Add new nodes to trees
            if within_radius(tree_start(idx_start).pos, new_point_start, step_size)
                new_node_start = struct('pos', new_point_start, 'parent', idx_start, 'cost', tree_start(idx_start).cost + dist(tree_start(idx_start).pos, new_point_start));
                tree_start(end + 1) = new_node_start;
                plot([tree_start(idx_start).pos(1), new_point_start(1)], [tree_start(idx_start).pos(2), new_point_start(2)], 'b');
                drawnow;
            end
            
            if within_radius(tree_goal(idx_goal).pos, new_point_goal, step_size)
                new_node_goal = struct('pos', new_point_goal, 'parent', idx_goal, 'cost', tree_goal(idx_goal).cost + dist(tree_goal(idx_goal).pos, new_point_goal));
                tree_goal(end + 1) = new_node_goal;
                plot([tree_goal(idx_goal).pos(1), new_point_goal(1)], [tree_goal(idx_goal).pos(2), new_point_goal(2)], 'r');
                drawnow;
            end
            
            % Check for connection
            if within_radius(new_point_start, new_point_goal, step_size)
                disp('Path found!');
                
                % Trace path from start to goal
                path_start = [new_point_start];
                path_goal = [new_point_goal];
                
                idx = length(tree_start);
                while ~isempty(tree_start(idx).parent)
                    path_start = [tree_start(idx).pos; path_start];
                    idx = tree_start(idx).parent;
                end
                
                idx = length(tree_goal);
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
    end
    
    % Add a delay between iterations
    pause(delay_time);
end

if i == max_iter
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
    result = any(arrayfun(@(obstacle) dist(point, obstacle.center) <= obstacle.radius, obstacles));
end
