function split = segment(grid,segment_size,resolution_desired,segment_x_selection,segment_y_selection)

no_points = segment_size/resolution_desired;

idx.start_x = no_points*segment_x_selection-no_points+1;
idx.end_x = no_points*segment_x_selection+1;
idx.start_y = no_points*segment_y_selection-no_points+1;
idx.end_y = no_points*segment_y_selection+1;

split.x = grid.X(idx.start_x:idx.end_x);
split.y = grid.Y(idx.start_y:idx.end_y);
split.z = grid.Z(idx.start_x:idx.end_x,idx.start_y:idx.end_y);