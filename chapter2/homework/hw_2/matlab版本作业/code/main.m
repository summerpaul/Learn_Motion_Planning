% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
addpath('A_star')

% Environment map in 2D space 
% se
% 搜索的起点
xStart = 1.0;
yStart = 1.0;
% 搜索的目标点
xTarget = 9;

yTarget = 9;
% 栅格地图的最大尺寸
MAX_X = 10;
MAX_Y = 10;
% 构造有障碍物的地图
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);

% Waypoint Generator Using the A* 
% A星算法的结果
path = A_star_search(map, MAX_X,MAX_Y);

% visualize the 2D grid map
visualize_map(map, path, []);

% save map
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
