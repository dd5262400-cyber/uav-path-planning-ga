clear;
close all;
clc;

fprintf('========================================\n');
fprintf('UAV Path Planning with Genetic Algorithm\n');
fprintf('========================================\n\n');

start_point = [10, 10, 10];
end_point = [90, 90, 40];

num_cities = 3;
buildings_per_city = 30;
buildings = generate_city_buildings(num_cities, buildings_per_city);

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

[best_path, full_path, best_cost, cost_history] = uav_path_planning(buildings, start_point, end_point, params);

fprintf('\nVisualizing results...\n');
visualize_path(full_path, buildings, start_point, end_point, cost_history);

fprintf('\n========================================\n');
fprintf('Path planning completed successfully!\n');
fprintf('========================================\n');

function buildings = generate_city_buildings(num_cities, buildings_per_city)
    total_buildings = num_cities * buildings_per_city;
    buildings = zeros(total_buildings, 7);
    
    city_centers = [
        25, 30, 0;
        50, 50, 0;
        75, 70, 0
    ];
    
    cluster_std = [12, 12];
    base_height_mean = 15;
    base_height_std = 8;
    
    building_idx = 1;
    for city = 1:num_cities
        city_center = city_centers(city, 1:2);
        
        for i = 1:buildings_per_city
            center_x = normrnd(city_center(1), cluster_std(1));
            center_y = normrnd(city_center(2), cluster_std(2));
            
            center_x = max(10, min(90, center_x));
            center_y = max(10, min(90, center_y));
            
            width = normrnd(8, 3);
            depth = normrnd(8, 3);
            width = max(4, min(15, width));
            depth = max(4, min(15, depth));
            
            height = normrnd(base_height_mean, base_height_std);
            height = max(5, min(40, height));
            
            buildings(building_idx, :) = [center_x, center_y, 0, width, depth, height, height];
            building_idx = building_idx + 1;
        end
    end
    
    fprintf('Generated %d cities with %d buildings each\n', num_cities, buildings_per_city);
    fprintf('Total buildings: %d\n', total_buildings);
end