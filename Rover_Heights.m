function [height, new_l] = Rover_Heights(coord,seg_grid,l)

% INPUT: Coordinates of rover centre and wheels
%        Terrain segment
%        Distance betwen centre of rover and wheels
%
% OUTPUT: Height of rover centre and wheels
%         New top down distance between centre of rover and wheels


% =========================================================================
% Centre

% show what two cell numbers the rover is between on x/y axis
x_co = find(seg_grid.X <= coord(1,1), 1, 'last');
y_co = find(seg_grid.Y <= coord(1,2), 1, 'first');

x_co = [x_co x_co+1];
y_co = [y_co y_co-1];

% find coordinates of grid segment rover is in & height values
x1 = seg_grid.X(x_co(1)); 
x2 = seg_grid.X(x_co(2));
y1 = seg_grid.Y(y_co(1));
y2 = seg_grid.Y(y_co(2));
Z11 = seg_grid.inter_seg(x_co(1),y_co(1));
Z12 = seg_grid.inter_seg(x_co(1),y_co(2));
Z21 = seg_grid.inter_seg(x_co(2),y_co(1));
Z22 = seg_grid.inter_seg(x_co(2),y_co(2));

% interpolate to find height at specific point

xy1 = ((x2-coord(1,1))/(x2-x1))*Z11 + ((coord(1,1)-x1)/(x2-x1))*Z21;
xy2 = ((x2-coord(1,1))/(x2-x1))*Z12 + ((coord(1,1)-x1)/(x2-x1))*Z22;
height(1) = ((y2-coord(1,2))/(y2-y1))*xy1 + ((coord(1,2)-y1)/(y2-y1))*xy2;

% =========================================================================
% Front left
x_co_fl = find(seg_grid.X <= coord(2,1), 1, 'last');
y_co_fl = find(seg_grid.Y <= coord(2,2), 1, 'first');

x_co_fl = [x_co_fl x_co_fl+1];
y_co_fl = [y_co_fl y_co_fl-1];

x1_fl = seg_grid.X(x_co_fl(1)); 
x2_fl = seg_grid.X(x_co_fl(2));
y1_fl = seg_grid.Y(y_co_fl(1));
y2_fl = seg_grid.Y(y_co_fl(2));
Z11_fl = seg_grid.inter_seg(x_co_fl(1),y_co_fl(1));
Z12_fl = seg_grid.inter_seg(x_co_fl(1),y_co_fl(2));
Z21_fl = seg_grid.inter_seg(x_co_fl(2),y_co_fl(1));
Z22_fl = seg_grid.inter_seg(x_co_fl(2),y_co_fl(2));

xy1_fl = ((x2_fl-coord(2,1))/(x2_fl-x1_fl))*Z11_fl + ((coord(2,1)-x1_fl)/(x2_fl-x1_fl))*Z21_fl;
xy2_fl = ((x2_fl-coord(2,1))/(x2_fl-x1_fl))*Z12_fl + ((coord(2,1)-x1_fl)/(x2_fl-x1_fl))*Z22_fl;
height(2) = ((y2_fl-coord(2,2))/(y2_fl-y1_fl))*xy1_fl + ((coord(2,2)-y1_fl)/(y2_fl-y1_fl))*xy2_fl;

opposite_fl = height(2) - height(1);
angle_fl = atan(opposite_fl/l);
oppositex_fl = l*sin(angle_fl);
height(2) = height(1) + oppositex_fl;
new_l(1) = l*cos(angle_fl);

% =========================================================================
% Front right
x_co_fr = find(seg_grid.X <= coord(3,1), 1, 'last');
y_co_fr = find(seg_grid.Y <= coord(3,2), 1, 'first');

x_co_fr = [x_co_fr x_co_fr+1];
y_co_fr = [y_co_fr y_co_fr-1];

x1_fr = seg_grid.X(x_co_fr(1)); 
x2_fr = seg_grid.X(x_co_fr(2));
y1_fr = seg_grid.Y(y_co_fr(1));
y2_fr = seg_grid.Y(y_co_fr(2));
Z11_fr = seg_grid.inter_seg(x_co_fr(1),y_co_fr(1));
Z12_fr = seg_grid.inter_seg(x_co_fr(1),y_co_fr(2));
Z21_fr = seg_grid.inter_seg(x_co_fr(2),y_co_fr(1));
Z22_fr = seg_grid.inter_seg(x_co_fr(2),y_co_fr(2));

xy1_fr = ((x2_fr-coord(3,1))/(x2_fr-x1_fr))*Z11_fr + ((coord(3,1)-x1_fr)/(x2_fr-x1_fr))*Z21_fr;
xy2_fr = ((x2_fr-coord(3,1))/(x2_fr-x1_fr))*Z12_fr + ((coord(3,1)-x1_fr)/(x2_fr-x1_fr))*Z22_fr;
height(3) = ((y2_fr-coord(3,2))/(y2_fr-y1_fr))*xy1_fr + ((coord(3,2)-y1_fr)/(y2_fr-y1_fr))*xy2_fr;

opposite_fr = height(3) - height(1);
angle_fr = atan(opposite_fr/l);
oppositex_fr = l*sin(angle_fr);
height(3) = height(1) + oppositex_fr;
new_l(2) = l*cos(angle_fr);

% =========================================================================
% Back left
x_co_bl = find(seg_grid.X <= coord(4,1), 1, 'last');
y_co_bl = find(seg_grid.Y <= coord(4,2), 1, 'first');

x_co_bl = [x_co_bl x_co_bl+1];
y_co_bl = [y_co_bl y_co_bl-1];

x1_bl = seg_grid.X(x_co_bl(1)); 
x2_bl = seg_grid.X(x_co_bl(2));
y1_bl = seg_grid.Y(y_co_bl(1));
y2_bl = seg_grid.Y(y_co_bl(2));
Z11_bl = seg_grid.inter_seg(x_co_bl(1),y_co_bl(1));
Z12_bl = seg_grid.inter_seg(x_co_bl(1),y_co_bl(2));
Z21_bl = seg_grid.inter_seg(x_co_bl(2),y_co_bl(1));
Z22_bl = seg_grid.inter_seg(x_co_bl(2),y_co_bl(2));

xy1_bl = ((x2_bl-coord(4,1))/(x2_bl-x1_bl))*Z11_bl + ((coord(4,1)-x1_bl)/(x2_bl-x1_bl))*Z21_bl;
xy2_bl = ((x2_bl-coord(4,1))/(x2_bl-x1_bl))*Z12_bl + ((coord(4,1)-x1_bl)/(x2_bl-x1_bl))*Z22_bl;
height(4) = ((y2_bl-coord(4,2))/(y2_bl-y1_bl))*xy1_bl + ((coord(4,2)-y1_bl)/(y2_bl-y1_bl))*xy2_bl;

opposite_bl = height(4) - height(1);
angle_bl = atan(opposite_bl/l);
oppositex_bl = l*sin(angle_bl);
height(4) = height(1,1) + oppositex_bl;
new_l(3) = l*cos(angle_bl);

% =========================================================================
% Back right
x_co_br = find(seg_grid.X <= coord(5,1), 1, 'last');
y_co_br = find(seg_grid.Y <= coord(5,2), 1, 'first');

x_co_br = [x_co_br x_co_br+1];
y_co_br = [y_co_br y_co_br-1];

x1_br = seg_grid.X(x_co_br(1)); 
x2_br = seg_grid.X(x_co_br(2));
y1_br = seg_grid.Y(y_co_br(1));
y2_br = seg_grid.Y(y_co_br(2));
Z11_br = seg_grid.inter_seg(x_co_br(1),y_co_br(1));
Z12_br = seg_grid.inter_seg(x_co_br(1),y_co_br(2));
Z21_br = seg_grid.inter_seg(x_co_br(2),y_co_br(1));
Z22_br = seg_grid.inter_seg(x_co_br(2),y_co_br(2));

xy1_br = ((x2_br-coord(5,1))/(x2_br-x1_br))*Z11_br + ((coord(5,1)-x1_br)/(x2_br-x1_br))*Z21_br;
xy2_br = ((x2_br-coord(5,1))/(x2_br-x1_br))*Z12_br + ((coord(5,1)-x1_br)/(x2_br-x1_br))*Z22_br;
height(5) = ((y2_br-coord(5,2))/(y2_br-y1_br))*xy1_br + ((coord(5,2)-y1_br)/(y2_br-y1_br))*xy2_br;

opposite_br = height(5) - height(1);
angle_br = atan(opposite_br/l);
oppositex_br = l*sin(angle_br);
height(5) = height(1) + oppositex_br;
new_l(4) = l*cos(angle_br);