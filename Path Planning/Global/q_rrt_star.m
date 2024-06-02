function qrrt_star
    % Parameters
    max_iterations = 2000;
    step_size = 1.0;
    goal_radius = 1.0;
    goal_sample_rate = 0.1;
    
    % Define start and goal locations
    start = [0, 0];
    goal = [90, 90];
    
    % Define obstacles (each row is [x_min, y_min, width, height])
    obstacles = [
        20, 20, 10, 10;
        40, 40, 10, 10;
        60, 60, 10, 10;
        20, 60, 10, 10;
        60, 20, 10, 10;
        10, 40, 10, 10;
        80, 40, 10, 10
    ];
    
    % Initialize the tree
    tree.vertices = start;
    tree.edges = [];
    tree.costs = 0;
    
    figure;
    hold on;
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Goal
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
    axis([-5, 105, -5, 105]);
    grid on;
    
    % Plot obstacles
    for k = 1:size(obstacles, 1)
        rectangle('Position', obstacles(k, :), 'FaceColor', [0, 0, 0]);
    end
    
    % RRT* algorithm
    goal_reached = false;
    for i = 1:max_iterations
        % Display iteration number
        title(['Iteration: ', num2str(i)]);
        
        % Random sampling
        if rand < goal_sample_rate
            sample = goal;
        else
            sample = [rand*100, rand*100];
        end
        
        % Find nearest vertex
        nearest_idx = nearest_vertex(tree.vertices, sample);
        nearest_vertex_pos = tree.vertices(nearest_idx, :);
        
        % Steer towards sample
        new_vertex = steer(nearest_vertex_pos, sample, step_size);
        
        % Check for obstacles
        if ~in_collision(nearest_vertex_pos, new_vertex, obstacles)
            % Add new vertex to the tree
            tree.vertices = [tree.vertices; new_vertex];
            new_idx = size(tree.vertices, 1);
            
            % Find near vertices
            near_indices = near_vertices(tree.vertices, new_vertex, step_size*10);
            
            % Choose parent with minimum cost
            min_cost = inf;
            best_near_idx = nearest_idx;
            for j = 1:length(near_indices)
                near_idx = near_indices(j);
                if near_idx <= length(tree.costs)
                    cost = tree.costs(near_idx) + distance(tree.vertices(near_idx, :), new_vertex);
                    if cost < min_cost && ~in_collision(tree.vertices(near_idx, :), new_vertex, obstacles)
                        min_cost = cost;
                        best_near_idx = near_idx;
                    end
                end
            end
            
            % Add edge
            tree.edges = [tree.edges; best_near_idx, new_idx];
            tree.costs = [tree.costs; min_cost];
            
            % Rewire near vertices
            for j = 1:length(near_indices)
                near_idx = near_indices(j);
                if near_idx <= length(tree.costs)
                    cost = min_cost + distance(tree.vertices(near_idx, :), new_vertex);
                    if cost < tree.costs(near_idx) && ~in_collision(tree.vertices(near_idx, :), new_vertex, obstacles)
                        % Update edge
                        tree.edges(tree.edges(:,2) == near_idx, :) = [];
                        tree.edges = [tree.edges; new_idx, near_idx];
                        tree.costs(near_idx) = cost;
                    end
                end
            end
            
            % Draw the new edge
            plot([nearest_vertex_pos(1), new_vertex(1)], [nearest_vertex_pos(2), new_vertex(2)], 'b');
            drawnow;
            
            % Check if goal is reached
            if distance(new_vertex, goal) < goal_radius
                goal_reached = true;
                break;
            end
        end
    end
    
    if goal_reached
        disp('Goal reached!');
        highlight_path(tree, goal);
    else
        disp('Goal not reached within the maximum number of iterations.');
    end
end

function d = distance(p1, p2)
    d = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);
end

function nearest_idx = nearest_vertex(vertices, sample)
    distances = sqrt(sum((vertices - sample).^2, 2));
    [~, nearest_idx] = min(distances);
end

function new_vertex = steer(nearest_vertex, sample, step_size)
    direction = sample - nearest_vertex;
    length = norm(direction);
    direction = direction / length; % normalize
    if length < step_size
        new_vertex = sample;
    else
        new_vertex = nearest_vertex + step_size * direction;
    end
end

function near_indices = near_vertices(vertices, new_vertex, radius)
    distances = sqrt(sum((vertices - new_vertex).^2, 2));
    near_indices = find(distances < radius);
end

function collision = in_collision(p1, p2, obstacles)
    % Check if the line between p1 and p2 intersects any obstacle
    collision = false;
    for k = 1:size(obstacles, 1)
        obstacle = obstacles(k, :);
        x_min = obstacle(1);
        y_min = obstacle(2);
        x_max = x_min + obstacle(3);
        y_max = y_min + obstacle(4);
        
        if line_rect_intersection(p1, p2, [x_min, y_min], [x_max, y_max])
            collision = true;
            return;
        end
    end
end

function intersect = line_rect_intersection(p1, p2, rect_min, rect_max)
    % Liang-Barsky algorithm for line-rectangle intersection
    x1 = p1(1); y1 = p1(2);
    x2 = p2(1); y2 = p2(2);
    x_min = rect_min(1); y_min = rect_min(2);
    x_max = rect_max(1); y_max = rect_max(2);
    
    dx = x2 - x1;
    dy = y2 - y1;
    
    p = [-dx, dx, -dy, dy];
    q = [x1 - x_min, x_max - x1, y1 - y_min, y_max - y1];
    
    t0 = 0;
    t1 = 1;
    
    for i = 1:4
        if p(i) == 0
            if q(i) < 0
                intersect = false;
                return;
            end
        else
            t = q(i) / p(i);
            if p(i) < 0
                if t > t1
                    intersect = false;
                    return;
                end
                if t > t0
                    t0 = t;
                end
            else
                if t < t0
                    intersect = false;
                    return;
                end
                if t < t1
                    t1 = t;
                end
            end
        end
    end
    
    intersect = true;
end

function highlight_path(tree, goal)
    % Highlight the path from start to goal
    goal_idx = nearest_vertex(tree.vertices, goal);
    path = goal_idx;
    while path(1) ~= 1 % while not the start vertex
        parent = tree.edges(tree.edges(:,2) == path(1), 1);
        path = [parent; path];
    end
    
    % Plot the path
    for k = 1:length(path) - 1
        v1 = tree.vertices(path(k), :);
        v2 = tree.vertices(path(k + 1), :);
        plot([v1(1), v2(1)], [v1(2), v2(2)], 'r', 'LineWidth', 2);
    end
end
