clear;
close all;
clc;

fprintf('========================================\n');
fprintf('UAV Path Planning with Genetic Algorithm\n');
fprintf('========================================\n\n');

start_point = [10, 10, 10];
end_point = [90, 90, 40];

num_city_clusters = 4;
obstacles_per_cluster = 20;
obstacles = generate_city_obstacles(num_city_clusters, obstacles_per_cluster);

params = struct();
params.pop_size = 50;
params.max_gen = 500;
params.mutation_rate = 0.1;
params.crossover_rate = 0.8;
params.num_waypoints = 10;
params.bounds = [0, 100; 0, 100; 0, 50];
params.weight_distance = 1.0;
params.weight_smoothness = 0.5;
params.weight_safety = 2.0;

fprintf('Running path planning...\n\n');

[best_path, full_path, best_cost, cost_history] = uav_path_planning(obstacles, start_point, end_point, params);

fprintf('\nVisualizing results...\n');
visualize_path(full_path, obstacles, start_point, end_point, cost_history);

fprintf('\n========================================\n');
fprintf('Path planning completed successfully!\n');
fprintf('========================================\n');

function obstacles = generate_city_obstacles(num_clusters, obstacles_per_cluster)
    total_obstacles = num_clusters * obstacles_per_cluster;
    obstacles = zeros(total_obstacles, 4);
    
    city_centers = [
        25, 25, 20;
        75, 25, 25;
        25, 75, 30;
        75, 75, 35
    ];
    
    cluster_std = [15, 15, 8];
    
    obstacle_idx = 1;
    for cluster = 1:num_clusters
        cluster_center = city_centers(cluster, :);
        
        for i = 1:obstacles_per_cluster
            center_x = normrnd(cluster_center(1), cluster_std(1));
            center_y = normrnd(cluster_center(2), cluster_std(2));
            center_z = normrnd(cluster_center(3), cluster_std(3));
            
            center_x = max(5, min(95, center_x));
            center_y = max(5, min(95, center_y));
            center_z = max(5, min(45, center_z));
            
            radius = normrnd(8, 3);
            radius = max(3, min(15, radius));
            
            obstacles(obstacle_idx, :) = [center_x, center_y, center_z, radius];
            obstacle_idx = obstacle_idx + 1;
        end
    end
    
    fprintf('Generated %d city clusters with %d obstacles each\n', num_clusters, obstacles_per_cluster);
    fprintf('Total obstacles: %d\n', total_obstacles);
end