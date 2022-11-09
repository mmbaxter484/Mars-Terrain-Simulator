function grid = stl_to_grid(stl_file, x_size, y_size)

terrain_stl_data = stlread(stl_file);  % load in mars terrain stl file
terrain_points = terrain_stl_data.Points;  % show number of points in file  

x = terrain_points(:,1);  % obtain x data
y = terrain_points(:,2);  % obtain y data
z = terrain_points(:,3);  % obtain z data

inter = scatteredInterpolant(x,y,z,'natural');  % create interpolant

grid.X = linspace(min(x), max(x), x_size);  % set up x vector grid
grid.Y = linspace(min(y), max(y), y_size);  % set up y vector grid
grid.Z = inter({grid.X, grid.Y});  % generate interpolated z points

% check new resolution from grid function
grid.resx = grid.X(2) - grid.X(1); 
grid.resy = grid.Y(2) - grid.Y(1); 

[X_pc, Y_pc] = ndgrid(grid.X, grid.Y);  % generate grid for point cloud
grid.pc = pointCloud([X_pc(:) Y_pc(:) grid.Z(:)]); % generate point cloud of interpolated points
