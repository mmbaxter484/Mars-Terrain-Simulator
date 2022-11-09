function seg_grid = segment_inter(split, n)

% n = 1; % specifies how many times the interpolated points are halved ()

seg_grid.inter_seg = interpn(split.z,n,'spline');  

seg_grid.X = linspace(min(split.x), max(split.x), size(seg_grid.inter_seg,1));  % set up x vector grid
seg_grid.Y = linspace(min(split.y), max(split.y), size(seg_grid.inter_seg,2));  % set up y vector grid

[seg_X_pc, seg_Y_pc] = ndgrid(seg_grid.X, seg_grid.Y);  % generate grid for point cloud
seg_grid.pc = pointCloud([seg_X_pc(:) seg_Y_pc(:) seg_grid.inter_seg(:)]);  % generate point cloud for chosen segment
