clear;
close all;
clc;

fprintf('========================================\n');
fprintf('UAV Path Planning with Genetic Algorithm\n');
fprintf('========================================\n\n');

start_point = [10, 10, 10];
end_point = [90, 90, 40];

num_obstacles = 8;
obstacles = generate_obstacles(num_obstacles);

params = struct();
params.pop_size = 50;
params.max_gen = 100;
params.mutation_rate = 0.1;
params.crossover_rate = 0.8;
params.num_waypoints = 10;
params.bounds = [0, 100; 0, 100; 0, 50];
params.weight_distance = 1.0;
params.weight_smoothness = 0.5;
params.weight_safety = 2.0;

fprintf('Running path planning...\n\n');

[best_path, full_path, best_cost] = uav_path_planning(obstacles, start_point, end_point, params);

fprintf('\nVisualizing results...\n');
visualize_path(full_path, obstacles, start_point, end_point, cost_history);

fprintf('\n========================================\n');
fprintf('Path planning completed successfully!\n');
fprintf('========================================\n');

function obstacles = generate_obstacles(num_obstacles)
    obstacles = zeros(num_obstacles, 4);
    for i = 1:num_obstacles
        center_x = unifrnd(20, 80);
        center_y = unifrnd(20, 80);
        center_z = unifrnd(10, 40);
        radius = unifrnd(5, 10);
        obstacles(i, :) = [center_x, center_y, center_z, radius];
    end
end