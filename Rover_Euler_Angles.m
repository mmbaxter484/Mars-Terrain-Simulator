function [theta, phi, psi] = Rover_Euler_Angles(height, coord)

    height_f = (height(2) + height(3))/2;
    height_b = (height(4) + height(5))/2;
    height_l = (height(2) + height(4))/2;
    height_r = (height(3) + height(5))/2;

    ave_x_f = (coord(2,1)+coord(3,1))/2;
    ave_y_f = (coord(2,2)+coord(3,2))/2;
    ave_x_b = (coord(4,1)+coord(5,1))/2;
    ave_y_b = (coord(4,2)+coord(5,2))/2;
    d_p = sqrt((ave_x_f-ave_x_b)^2+(ave_y_f-ave_y_b)^2);
    theta = (atan((height_f-height_b)/d_p));
    
    ave_x_l = (coord(2,1)+coord(4,1))/2;
    ave_y_l = (coord(2,2)+coord(4,2))/2;
    ave_x_r = (coord(3,1)+coord(5,1))/2;
    ave_y_r = (coord(3,2)+coord(5,2))/2;
    d_r = sqrt((ave_x_r-ave_x_l)^2+(ave_y_r-ave_y_l)^2);
    phi = atan((height_l-height_r)/d_r);
    
    d_y = sqrt((ave_x_f-ave_x_b)^2+(ave_y_f-ave_y_b)^2);
    psi = atan((ave_y_f-ave_y_b)/(ave_x_f-ave_x_b));
    