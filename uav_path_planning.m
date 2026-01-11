function [best_path, full_path, best_cost, cost_history] = uav_path_planning(obstacles, start_point, end_point, params)
    if nargin < 4
        params = struct();
    end
    
    fprintf('=== UAV Path Planning with Genetic Algorithm ===\n');
    fprintf('Start Point: [%.2f, %.2f, %.2f]\n', start_point);
    fprintf('End Point: [%.2f, %.2f, %.2f]\n', end_point);
    fprintf('Number of Obstacles: %d\n', size(obstacles, 1));
    fprintf('\n');
    
    [best_path, best_cost, best_generation, cost_history] = genetic_algorithm(params, obstacles, start_point, end_point);
    
    num_waypoints = length(best_path) / 3;
    waypoints = reshape(best_path, 3, num_waypoints)';
    full_path = [start_point; waypoints; end_point];
    
    fprintf('\n=== Optimization Complete ===\n');
    fprintf('Best Generation: %d\n', best_generation);
    fprintf('Best Cost: %.4f\n', best_cost);
    fprintf('Path Length: %.4f\n', calculate_path_distance(full_path));
    fprintf('Number of Waypoints: %d\n', num_waypoints);
end

function distance = calculate_path_distance(path)
    distance = 0;
    for i = 1:size(path, 1)-1
        distance = distance + norm(path(i+1, :) - path(i, :));
    end
end