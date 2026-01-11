function [best_path, best_cost, best_generation, cost_history] = genetic_algorithm(params, obstacles, start_point, end_point)
    params = validate_params(params);
    
    pop_size = params.pop_size;
    max_gen = params.max_gen;
    mutation_rate = params.mutation_rate;
    crossover_rate = params.crossover_rate;
    num_waypoints = params.num_waypoints;
    bounds = params.bounds;
    
    population = initialize_population(pop_size, num_waypoints, bounds);
    
    best_cost = Inf;
    best_path = [];
    best_generation = 0;
    
    cost_history = zeros(max_gen, 1);
    
    for gen = 1:max_gen
        costs = evaluate_population(population, obstacles, start_point, end_point, params);
        
        [current_best_cost, best_idx] = min(costs);
        current_best_path = population(best_idx, :);
        
        if current_best_cost < best_cost
            best_cost = current_best_cost;
            best_path = current_best_path;
            best_generation = gen;
        end
        
        cost_history(gen) = best_cost;
        
        if mod(gen, 10) == 0
            fprintf('Generation %d: Best Cost = %.4f\n', gen, best_cost);
        end
        
        selected = selection(population, costs, params);
        offspring = crossover(selected, crossover_rate, params);
        population = mutate(offspring, mutation_rate, bounds);
        
        population(1, :) = best_path;
    end
    
    if nargout > 3
        assignin('base', 'cost_history', cost_history);
    end
end

function params = validate_params(params)
    defaults = struct(...
        'pop_size', 50, ...
        'max_gen', 100, ...
        'mutation_rate', 0.1, ...
        'crossover_rate', 0.8, ...
        'num_waypoints', 10, ...
        'bounds', [0, 100; 0, 100; 0, 50], ...
        'weight_distance', 1.0, ...
        'weight_smoothness', 0.5, ...
        'weight_safety', 2.0);
    
    fields = fieldnames(defaults);
    for i = 1:length(fields)
        if ~isfield(params, fields{i})
            params.(fields{i}) = defaults.(fields{i});
        end
    end
end

function population = initialize_population(pop_size, num_waypoints, bounds)
    population = zeros(pop_size, num_waypoints * 3);
    for i = 1:pop_size
        for j = 1:num_waypoints
            idx = (j-1) * 3 + 1;
            population(i, idx:idx+2) = [...
                unifrnd(bounds(1,1), bounds(1,2)), ...
                unifrnd(bounds(2,1), bounds(2,2)), ...
                unifrnd(bounds(3,1), bounds(3,2))];
        end
    end
end

function costs = evaluate_population(population, obstacles, start_point, end_point, params)
    pop_size = size(population, 1);
    costs = zeros(pop_size, 1);
    
    for i = 1:pop_size
        costs(i) = fitness_function(population(i, :), obstacles, start_point, end_point, params);
    end
end

function cost = fitness_function(path, obstacles, start_point, end_point, params)
    num_waypoints = length(path) / 3;
    waypoints = reshape(path, 3, num_waypoints)';
    
    full_path = [start_point; waypoints; end_point];
    
    cost_distance = calculate_path_distance(full_path);
    cost_smoothness = calculate_smoothness(full_path);
    cost_safety = calculate_safety_cost(full_path, obstacles);
    
    cost = params.weight_distance * cost_distance + ...
           params.weight_smoothness * cost_smoothness + ...
           params.weight_safety * cost_safety;
end

function distance = calculate_path_distance(path)
    distance = 0;
    for i = 1:size(path, 1)-1
        distance = distance + norm(path(i+1, :) - path(i, :));
    end
end

function smoothness = calculate_smoothness(path)
    smoothness = 0;
    for i = 2:size(path, 1)-1
        v1 = path(i, :) - path(i-1, :);
        v2 = path(i+1, :) - path(i, :);
        angle = acos(dot(v1, v2) / (norm(v1) * norm(v2) + eps));
        smoothness = smoothness + angle;
    end
end

function safety_cost = calculate_safety_cost(path, obstacles)
    safety_cost = 0;
    for i = 1:size(path, 1)-1
        segment_start = path(i, :);
        segment_end = path(i+1, :);
        
        for j = 1:size(obstacles, 1)
            obstacle_center = obstacles(j, 1:3);
            obstacle_radius = obstacles(j, 4);
            
            dist = point_to_segment_distance(obstacle_center, segment_start, segment_end);
            
            if dist < obstacle_radius
                safety_cost = safety_cost + (obstacle_radius - dist)^2 * 100;
            end
        end
    end
end

function dist = point_to_segment_distance(point, seg_start, seg_end)
    v = seg_end - seg_start;
    w = point - seg_start;
    
    c1 = dot(w, v);
    if c1 <= 0
        dist = norm(point - seg_start);
        return;
    end
    
    c2 = dot(v, v);
    if c2 <= c1
        dist = norm(point - seg_end);
        return;
    end
    
    b = c1 / c2;
    pb = seg_start + b * v;
    dist = norm(point - pb);
end

function selected = selection(population, costs, params)
    pop_size = size(population, 1);
    selected = zeros(pop_size, size(population, 2));
    
    fitness = 1 ./ (costs + eps);
    total_fitness = sum(fitness);
    probabilities = fitness / total_fitness;
    
    for i = 1:pop_size
        r = rand;
        cumulative_prob = 0;
        for j = 1:pop_size
            cumulative_prob = cumulative_prob + probabilities(j);
            if r <= cumulative_prob
                selected(i, :) = population(j, :);
                break;
            end
        end
    end
end

function offspring = crossover(selected, crossover_rate, params)
    pop_size = size(selected, 1);
    offspring = selected;
    num_waypoints = params.num_waypoints;
    
    for i = 1:2:pop_size-1
        if rand < crossover_rate
            crossover_point = randi(num_waypoints - 1);
            idx = crossover_point * 3;
            
            temp = offspring(i, idx+1:end);
            offspring(i, idx+1:end) = offspring(i+1, idx+1:end);
            offspring(i+1, idx+1:end) = temp;
        end
    end
end

function mutated = mutate(offspring, mutation_rate, bounds)
    [pop_size, num_genes] = size(offspring);
    mutated = offspring;
    
    for i = 1:pop_size
        for j = 1:num_genes
            if rand < mutation_rate
                dim = mod(j-1, 3) + 1;
                mutated(i, j) = unifrnd(bounds(dim, 1), bounds(dim, 2));
            end
        end
    end
end