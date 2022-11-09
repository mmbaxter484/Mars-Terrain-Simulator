% Mars_Surface.m
% Created by: Michael Baxter
% Required files: 
% Required functions: stl_to_grid.m, segment.m, segment_inter.m

clear all; clc; close all

stl_file = 'Aram3mpp.stl';  % specify Mars stl terrain file

terrain = stlread(stl_file);  % load stl file
terrain_points = terrain.Points;

% check info about area of Mars
dim.x = max(terrain_points(:,1)) - min(terrain_points(:,1));  % x dimension (m)
dim.y = max(terrain_points(:,2)) - min(terrain_points(:,2));  % y dimension (m)
dim.z = max(terrain_points(:,3)) - min(terrain_points(:,3));  % elevation change (m)
resolution = terrain_points(2,2) - terrain_points(1,2);       % resolution of grid (m per point)

% define desired resolution for full DTM - do not reccomend lower than 2.5mpp
resolution_desired = 4;

% define size of terrain grid
size_x = dim.x/resolution_desired;
size_y = dim.y/resolution_desired;

% function to generate interpolated grid and point cloud
grid = stl_to_grid(stl_file, size_x, size_y);
resolution_grid = grid.X(2) - grid.X(1);

% % plot original data
figure(1)
clf
trimesh(terrain)
axis equal
title('Original Terrain - Grid')
subtitle(sprintf('Resolution: %d mpp',resolution))
xlabel 'X-Direction (m)'
ylabel 'Y-Direction (m)'
zlabel 'Elevation (m)'
c = colorbar;
c.Label.String = 'Elevation (m)';
% % % 
% % % plot interpolated data - grid overlay needs work
% figure(2)
% clf
% mesh(grid.X, grid.Y, grid.Z')
% axis equal
% title('Interpolated Terrain - Grid')
% subtitle(sprintf('Resolution: %d mpp',resolution_grid))
% xlabel 'X-Direction (m)'
% ylabel 'Y-Direction (m)'
% zlabel 'Elevation (m)'
% % colormap(copper)
% % c = colorbar;
% % c.Label.String = 'Elevation (m)';
%  
% % plot point cloud
figure(3)
clf
pcshow(grid.pc)
title('Interpolated Terrain - Point Cloud')
subtitle(sprintf('Resolution: %d mpp',resolution_grid))
xlabel 'X-Direction (m)'
ylabel 'Y-Direction (m)'; zlabel 'Elevation (m)'
% colormap(copper)
% c = colorbar;
% c.Label.String = 'Elevation (m)';

% figure(4)
% subplot(1,2,1)
% trimesh(terrain)
% axis equal
% title('Original Terrain - Grid')
% subtitle(sprintf('Resolution: %d mpp',resolution))
% xlabel 'X-Direction (m)'
% ylabel 'Y-Direction (m)'
% zlabel 'Elevation (m)'
% subplot(1,2,2)
% mesh(grid.X, grid.Y, grid.Z')
% axis equal
% title('Interpolated Terrain - Grid')
% subtitle(sprintf('Resolution: %d mpp',resolution_grid))
% xlabel 'X-Direction (m)'
% ylabel 'Y-Direction (m)'
% zlabel 'Elevation (m)'

% % choose desired segment square dimension (m)
prompt = ('Segment square side length: ');
segment_size = input(prompt);
% segment_size = 100;
segments_x = dim.x/segment_size;  % no of x segments
segments_y = dim.y/segment_size;  % no of y segments


% choose x-axis segment
% prompt = ['Number of x segments = ' num2str(segments_x) ', x-dimension segment choice: '];
% segment_x_selection = input(prompt);
prompt = ('Desired x-coordinate: ');
segment_x_selection = ceil(((dim.x/2) + input(prompt))/segment_size);

% choose y-axis segment
% prompt = ['Number of y segments = ' num2str(segments_y) ', y-dimension segment choice: '];
% segment_y_selection = input(prompt);
prompt = ('Desired y-coordinate: ');
segment_y_selection = ceil(((dim.y/2) + input(prompt))/segment_size);
 
% % function to extract segment data
split = segment(grid,segment_size,resolution_desired,segment_x_selection,segment_y_selection);

% function to interpolate for chosen segment at desired resolution
prompt = ['Resolution options (mpp):\n0 - ' num2str(resolution_grid) '\n1 - ' num2str(resolution_grid/2) '\n2 - ' num2str(resolution_grid/4)...
    '\n3 - ' num2str(resolution_grid/8) '\n4 - ' num2str(resolution_grid/16) '\n5 - ' num2str(resolution_grid/32) '\nResolution choice: '];
segment_resolution_selection = input(prompt);
res = segment_resolution_selection;  % will only take integers - specifies how many times the points are halved between
seg_grid = segment_inter(split, res);

% check new resolution (metres per point)
res_seg = segment_size/(size(seg_grid.X,2)-1);
res_segx = seg_grid.X(3)-seg_grid.X(2);
res_segy = seg_grid.Y(2)-seg_grid.Y(1);

% plot original segment data
% figure(4)
% clf
% mesh(split.x, split.y, split.z')
% axis equal
% title('Original Interpolation - Chosen Segment - Grid')
% subtitle(sprintf('Resolution: %d mpp',resolution_grid))
% xlabel 'X-Direction(m)'
% ylabel 'Y-Direction(m)'
% zlabel 'Elevation (m)'
% % colormap(copper)
% c = colorbar;
% c.Label.String = 'Elevation (m)';


% plot interpolated segment data
figure(5)
clf
mesh(seg_grid.X, seg_grid.Y,seg_grid.inter_seg')
% hold on
% mesh(seg_grid_r.X, seg_grid_r.Y,seg_grid_r.inter_seg')
% surf(seg_grid.X, seg_grid.Y, seg_grid.inter_seg','EdgeColor','none')
axis equal
set(gca,'FontSize',15)
% title('Further Interpolation - Chosen Segment - Grid')
% subtitle(sprintf('Resolution: %d mpp',res_seg))
xlabel 'X-Direction (m)'
ylabel 'Y-Direction (m)'
zlabel 'Elevation (m)'
% colormap(copper)
c = colorbar;
c.Label.String = 'Elevation (m)';

% plot segment point cloud
figure(6)
clf
pcshow(seg_grid.pc)
axis equal
title('Further Interpolation - Chosen Segment - Point Cloud')
subtitle(sprintf('Resolution: %d mpp',res_seg))
xlabel 'X-Direction(m)'
ylabel 'Y-Direction(m)'
zlabel 'Elevation (m)'
% colormap(copper)
c = colorbar;
c.Label.String = 'Elevation (m)';
 
% % flip y reference to match rover
seg_grid.Y = flip(seg_grid.Y);

