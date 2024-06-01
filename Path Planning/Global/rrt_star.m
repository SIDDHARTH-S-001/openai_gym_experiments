function RRTstar
    % Parameters
    start = [10, 10];
    goal = [90, 90];
    x_max = 100;
    y_max = 100;
    max_iter = 1000;
    step_size = 2;
    goal_radius = 5;
    search_radius = 10;
    num_obstacles = 20; % Number of obstacles
    obstacle_size = 5; % Size of each obstacle

    % Generate random obstacles
    obstacles = generateObstacles(num_obstacles, obstacle_size, x_max, y_max);

    % Initialize the tree with the start node
    tree.nodes = start;
    tree.cost = 0;
    tree.parent = 0;

    figure;
    hold on;
    xlim([0, x_max]);
    ylim([0, y_max]);
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Plot obstacles
    for i = 1:num_obstacles
        rectangle('Position', obstacles(i, :), 'FaceColor', [0, 0, 0]);
    end

    for i = 1:max_iter
        % Sample random point
        rand_point = [rand * x_max, rand * y_max];

        % Find nearest node in the tree
        [nearest_node, nearest_idx] = nearestNode(tree.nodes, rand_point);

        % Steer towards the random point
        new_point = steer(nearest_node, rand_point, step_size);

        % Check if the new point is within the search radius and closer to the goal
        if ~isCollision(nearest_node, new_point, obstacles)
            % Find neighbors within the search radius
            neighbors = findNeighbors(tree.nodes, new_point, search_radius);

            % Choose the parent node which results in minimum cost
            [min_cost, min_idx] = chooseParent(neighbors, tree, nearest_idx, new_point);

            % Add the new node to the tree
            tree.nodes = [tree.nodes; new_point];
            tree.cost = [tree.cost; min_cost];
            tree.parent = [tree.parent; min_idx];

            % Rewire the tree
            tree = rewire(tree, neighbors, new_point, search_radius);

            % Plot the new point and edge
            plot([nearest_node(1), new_point(1)], [nearest_node(2), new_point(2)], 'b');
            plot(new_point(1), new_point(2), 'bo');
        end

        % Check if goal is reached
        if norm(new_point - goal) <= goal_radius
            disp('Goal reached!');
            break;
        end
    end

    % Trace the path from goal to start
    plotPath(tree, goal, goal_radius);
end

function obstacles = generateObstacles(num_obstacles, obstacle_size, x_max, y_max)
    obstacles = rand(num_obstacles, 2) .* (repmat([x_max, y_max] - obstacle_size, num_obstacles, 1));
    obstacles = [obstacles, repmat([obstacle_size, obstacle_size], num_obstacles, 1)];
end

function [nearest_node, nearest_idx] = nearestNode(nodes, point)
    dists = sqrt(sum((nodes - point).^2, 2));
    [~, nearest_idx] = min(dists);
    nearest_node = nodes(nearest_idx, :);
end

function new_point = steer(from, to, step_size)
    direction = to - from;
    distance = norm(direction);
    if distance > step_size
        direction = direction / distance * step_size;
    end
    new_point = from + direction;
end

function collision = isCollision(from, to, obstacles)
    collision = false;
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);
        if lineIntersectsRect(from, to, obs)
            collision = true;
            return;
        end
    end
end

function intersects = lineIntersectsRect(p1, p2, rect)
    % Check if the line segment from p1 to p2 intersects the rectangle
    intersects = ...
        lineIntersectsLine(p1, p2, rect(1:2), rect(1:2) + [rect(3), 0]) || ...
        lineIntersectsLine(p1, p2, rect(1:2), rect(1:2) + [0, rect(4)]) || ...
        lineIntersectsLine(p1, p2, rect(1:2) + [rect(3), 0], rect(1:2) + rect(3:4)) || ...
        lineIntersectsLine(p1, p2, rect(1:2) + [0, rect(4)], rect(1:2) + rect(3:4));
end

function intersects = lineIntersectsLine(p1, p2, q1, q2)
    % Check if the line segment from p1 to p2 intersects the line segment from q1 to q2
    d = (q2(2) - q1(2)) * (p2(1) - p1(1)) - (q2(1) - q1(1)) * (p2(2) - p1(2));
    if d == 0
        intersects = false;
        return;
    end
    ua = ((q2(1) - q1(1)) * (p1(2) - q1(2)) - (q2(2) - q1(2)) * (p1(1) - q1(1))) / d;
    ub = ((p2(1) - p1(1)) * (p1(2) - q1(2)) - (p2(2) - p1(2)) * (p1(1) - q1(1))) / d;
    intersects = (ua >= 0 && ua <= 1) && (ub >= 0 && ub <= 1);
end

function neighbors = findNeighbors(nodes, point, radius)
    dists = sqrt(sum((nodes - point).^2, 2));
    neighbors = find(dists <= radius);
end

function [min_cost, min_idx] = chooseParent(neighbors, tree, nearest_idx, new_point)
    min_cost = tree.cost(nearest_idx) + norm(tree.nodes(nearest_idx, :) - new_point);
    min_idx = nearest_idx;
    for i = 1:length(neighbors)
        n = neighbors(i);
        cost = tree.cost(n) + norm(tree.nodes(n, :) - new_point);
        if cost < min_cost
            min_cost = cost;
            min_idx = n;
        end
    end
end

function tree = rewire(tree, neighbors, new_point, radius)
    new_idx = size(tree.nodes, 1);
    for i = 1:length(neighbors)
        n = neighbors(i);
        if n == new_idx
            continue;
        end
        cost = tree.cost(new_idx) + norm(tree.nodes(new_idx, :) - tree.nodes(n, :));
        if cost < tree.cost(n)
            tree.cost(n) = cost;
            tree.parent(n) = new_idx;
            % Update the tree plot
            plot([tree.nodes(n, 1), tree.nodes(tree.parent(n), 1)], ...
                 [tree.nodes(n, 2), tree.nodes(tree.parent(n), 2)], 'w');
            plot([tree.nodes(n, 1), tree.nodes(new_idx, 1)], ...
                 [tree.nodes(n, 2), tree.nodes(new_idx, 2)], 'b');
        end
    end
end

function plotPath(tree, goal, goal_radius)
    dists = sqrt(sum((tree.nodes - goal).^2, 2));
    [~, goal_idx] = min(dists);
    if dists(goal_idx) <= goal_radius
        node = goal_idx;
        while node ~= 0
            parent = tree.parent(node);
            if parent == 0
                break;
            end
            plot([tree.nodes(node, 1), tree.nodes(parent, 1)], ...
                 [tree.nodes(node, 2), tree.nodes(parent, 2)], 'r', 'LineWidth', 2);
            node = parent;
        end
    end
end
