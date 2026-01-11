function visualize_path(path, obstacles, start_point, end_point, cost_history)
    figure('Name', 'UAV Path Planning Visualization', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);
    
    subplot(1, 2, 1);
    plot_3d_path(path, obstacles, start_point, end_point);
    
    if nargin >= 5 && ~isempty(cost_history)
        subplot(1, 2, 2);
        plot_cost_history(cost_history);
    end
end

function plot_3d_path(path, obstacles, start_point, end_point)
    hold on;
    grid on;
    view(3);
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D UAV Path Planning');
    
    plot3(start_point(1), start_point(2), start_point(3), 'go', 'MarkerSize', 12, 'LineWidth', 2);
    plot3(end_point(1), end_point(2), end_point(3), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
    
    plot3(path(:, 1), path(:, 2), path(:, 3), 'b-', 'LineWidth', 2);
    plot3(path(:, 1), path(:, 2), path(:, 3), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    
    if ~isempty(obstacles)
        for i = 1:size(obstacles, 1)
            center = obstacles(i, 1:3);
            radius = obstacles(i, 4);
            plot_sphere(center, radius);
        end
    end
    
    legend('Start Point', 'End Point', 'Path', 'Waypoints', 'Obstacles', 'Location', 'best');
    
    axis equal;
    xlim([0, 100]);
    ylim([0, 100]);
    zlim([0, 50]);
    
    hold off;
end

function plot_sphere(center, radius)
    [x, y, z] = sphere(20);
    x = x * radius + center(1);
    y = y * radius + center(2);
    z = z * radius + center(3);
    surf(x, y, z, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', 'r');
end

function plot_cost_history(cost_history)
    plot(cost_history, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Generation');
    ylabel('Best Cost');
    title('Cost Convergence History');
    
    set(gca, 'FontSize', 12);
end

function visualize_population(population, obstacles, start_point, end_point, generation)
    figure('Name', ['Population Generation ', num2str(generation)], 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
    
    hold on;
    grid on;
    view(3);
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(['Population at Generation ', num2str(generation)]);
    
    plot3(start_point(1), start_point(2), start_point(3), 'go', 'MarkerSize', 12, 'LineWidth', 2);
    plot3(end_point(1), end_point(2), end_point(3), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
    
    pop_size = size(population, 1);
    for i = 1:pop_size
        path = population(i, :);
        num_waypoints = length(path) / 3;
        waypoints = reshape(path, 3, num_waypoints)';
        full_path = [start_point; waypoints; end_point];
        
        alpha = 0.3;
        plot3(full_path(:, 1), full_path(:, 2), full_path(:, 3), 'b-', 'LineWidth', 0.5, 'Color', [0, 0, 1, alpha]);
    end
    
    if ~isempty(obstacles)
        for i = 1:size(obstacles, 1)
            center = obstacles(i, 1:3);
            radius = obstacles(i, 4);
            plot_sphere(center, radius);
        end
    end
    
    legend('Start Point', 'End Point', 'Population Paths', 'Obstacles', 'Location', 'best');
    
    axis equal;
    xlim([0, 100]);
    ylim([0, 100]);
    zlim([0, 50]);
    
    hold off;
end