# UAV Path Planning with Genetic Algorithm

基于遗传算法的无人机三维路径规划MATLAB实现。

## 项目简介

本项目实现了使用遗传算法进行无人机三维路径规划的完整系统。该系统能够在存在障碍物的三维空间中找到从起点到终点的最优路径，同时考虑路径长度、平滑度和安全性。

## 功能特性

- 基于遗传算法的三维路径优化
- 多目标优化（距离、平滑度、安全性）
- 障碍物避障功能
- 三维可视化展示
- 收敛曲线分析
- 可配置的算法参数

## 算法原理

### 遗传算法

遗传算法是一种模拟自然选择和遗传机制的优化算法，主要步骤包括：

1. **初始化种群**：随机生成初始路径种群
2. **适应度评估**：计算每条路径的适应度（成本）
3. **选择**：根据适应度选择优秀个体
4. **交叉**：通过交叉操作生成新个体
5. **变异**：通过变异操作增加种群多样性
6. **迭代**：重复上述步骤直到满足终止条件

### 适应度函数

适应度函数综合考虑以下因素：

- **路径距离**：路径的总长度
- **平滑度**：路径的平滑程度（角度变化）
- **安全性**：与障碍物的最小距离

```
Cost = w1 * distance + w2 * smoothness + w3 * safety
```

## 项目结构

```
uav-path-planning-ga/
├── genetic_algorithm.m    # 遗传算法核心实现
├── uav_path_planning.m    # 无人机路径规划主程序
├── visualize_path.m       # 可视化函数
├── main.m                 # 主测试脚本
├── README.md              # 项目说明文档
└── .gitignore             # Git忽略文件配置
```

## 快速开始

### 环境要求

- MATLAB R2016b 或更高版本
- Optimization Toolbox（可选）

### 运行示例

1. 克隆项目到本地
2. 在MATLAB中打开项目目录
3. 运行主脚本：

```matlab
main
```

### 自定义配置

可以通过修改 `main.m` 中的参数来自定义算法：

```matlab
params = struct();
params.pop_size = 50;              % 种群大小
params.max_gen = 100;              % 最大迭代次数
params.mutation_rate = 0.1;        % 变异率
params.crossover_rate = 0.8;       % 交叉率
params.num_waypoints = 10;         % 路径点数量
params.weight_distance = 1.0;      % 距离权重
params.weight_smoothness = 0.5;    % 平滑度权重
params.weight_safety = 2.0;        % 安全性权重
```

## 使用说明

### 基本用法

```matlab
% 定义起点和终点
start_point = [10, 10, 10];
end_point = [90, 90, 40];

% 生成障碍物
obstacles = generate_obstacles(8);

% 设置参数
params = struct();
params.pop_size = 50;
params.max_gen = 100;

% 运行路径规划
[best_path, full_path, best_cost] = uav_path_planning(obstacles, start_point, end_point, params);

% 可视化结果
visualize_path(full_path, obstacles, start_point, end_point, cost_history);
```

### 自定义障碍物

障碍物格式：每行表示一个障碍物，包含中心坐标和半径

```matlab
obstacles = [
    30, 30, 20, 8;   % 障碍物1: 中心(30,30,20), 半径8
    50, 50, 30, 10;  % 障碍物2: 中心(50,50,30), 半径10
    70, 70, 25, 6;   % 障碍物3: 中心(70,70,25), 半径6
];
```

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| pop_size | 整数 | 50 | 种群大小 |
| max_gen | 整数 | 100 | 最大迭代次数 |
| mutation_rate | 浮点数 | 0.1 | 变异率 (0-1) |
| crossover_rate | 浮点数 | 0.8 | 交叉率 (0-1) |
| num_waypoints | 整数 | 10 | 路径点数量 |
| bounds | 矩阵 | [0,100;0,100;0,50] | 搜索空间边界 |
| weight_distance | 浮点数 | 1.0 | 距离权重 |
| weight_smoothness | 浮点数 | 0.5 | 平滑度权重 |
| weight_safety | 浮点数 | 2.0 | 安全性权重 |

## 可视化

程序提供两种可视化方式：

1. **3D路径可视化**：展示最优路径、起点、终点和障碍物
2. **收敛曲线**：展示算法迭代过程中的成本变化

## 性能优化建议

1. **种群大小**：增加种群大小可以提高解的质量，但会增加计算时间
2. **迭代次数**：根据问题复杂度调整迭代次数
3. **权重调整**：根据应用场景调整距离、平滑度和安全性的权重
4. **路径点数量**：增加路径点数量可以提高路径精度，但会增加计算复杂度

## 应用场景

- 无人机自主导航
- 机器人路径规划
- 自动驾驶路线优化
- 空域规划
- 救援路径规划

## 算法优势

- 全局搜索能力强
- 能够处理复杂约束
- 易于并行化
- 适用于多目标优化

## 算法局限

- 收敛速度较慢
- 参数调优需要经验
- 对初始种群敏感
- 可能陷入局部最优

## 未来改进方向

- [ ] 添加其他优化算法（A*、粒子群等）对比
- [ ] 实现动态障碍物处理
- [ ] 添加多无人机协同规划
- [ ] 优化算法性能
- [ ] 添加GUI界面
- [ ] 支持实时路径更新

## 贡献指南

欢迎提交问题和拉取请求！

## 许可证

MIT License

## 作者

UAV Path Planning Team

## 联系方式

如有问题或建议，请通过GitHub Issues联系。

---

创建日期：2026-01-11