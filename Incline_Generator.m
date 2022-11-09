clc; clear all; close all
nrow = 100; ncol = 100; angle = 0; xsize = 100; ysize = 100; 

dx = xsize/ncol;  dy = ysize/nrow;
angle = deg2rad(angle);

inter_seg=zeros(nrow,ncol);
for j=1:nrow
    for i=1:ncol
        seg_grid.inter_seg(i,j) = (i-1)*dx*tan(angle);  % Switch i & j around for roll/pitch
    end
end

seg_grid.X = linspace(0,xsize,length(inter_seg));
seg_grid.Y = linspace(0,ysize,height(inter_seg));

surf(seg_grid.X,seg_grid.Y,seg_grid.inter_seg');
axis equal
xlabel 'x-direction (m)', ylabel 'y-direction (m)',zlabel 'Elevation (m)'

seg_grid.Y = flip(seg_grid.Y);