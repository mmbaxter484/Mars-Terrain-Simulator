% Mars_Rover_Main.m
% Created by: Michael Baxter
% Partner script: Mars_Surface.m or Incline_Generator.m
% Required functions: Mars_Rover_Dynamics.m, Rover_Motor_Model_v1.m, Rover_Heights.m, Rover_Euler_Angles.m, terramechanics_model.m 
clearvars -except seg_grid grid
close all; clc;


% ----------------------------------------------------------------------
% Initialise variables
x_mod = zeros(12,1); % Current position and velocity variables

I_mod = zeros(1,4);     % Initialise electrical current vector
Omega_mod = zeros(1,4); % Intialise wheel rotational velocity vector

unmatched = zeros(3,1);

simtime = 0;                % Simulation timer, s
maxtime = 20;               % Upper simulation timer limit, s
dt = 0.01;                  % Simulation time segments, s
i = 1;

% ----------------------------------------------------------------------
% Rover dimensions
front_length = 0.32;       % Front length of rover
side_length = 0.285;          % Side length of rover
h = 0.125;                   % Height of rover 
r = h/2;                     % Radius of wheel
b = 0.065;                   % Wheel thickness
% l = sqrt((front_length/2)^2 + (side_length/2)^2);  % Distance between centre and four corners of rover
% ang = atan(front_length/side_length);    % Angle between centre and corners of rover

front_length_wheel = front_length/2 - b/2;  % Distance between front centre of rover to centre of wheel
side_length_wheel = side_length/2 - r;  % Distance between side centre of rover to centre of wheel
l = sqrt(front_length_wheel^2 + side_length_wheel^2);  % Distance between centre of rover and centre of wheels
ang = atan(front_length_wheel/side_length_wheel);    % Angle between centre and wheel

% Set rover angle constraints
tipping_angle_pitch = deg2rad(90) - atan(h/side_length);  % Angle rover will tip over length-ways
tipping_angle_roll = deg2rad(90) - atan(h/front_length);  % Angle rover will tip over side-ways
stop_angle_regolith = deg2rad(28);  % Pitch angle rover will get stuck for regolith
stop_angle_sand = deg2rad(7);  % Pitch angle rover will get stuck for regolith

% ----------------------------------------------------------------------
% Initialise output matrices
Vout(1,:) = zeros(1,4);
time(1) = simtime;

torquesout_mod(1,:) = zeros(1,4);
xout_mod(1,:) = x_mod;
xdotout_mod(1,:) = zeros(12,1);
omegaout_mod(1,:) = zeros(1,4);
omegadotout_mod(1,:) = zeros(1,4);
Iout_mod(1,:) = zeros(1,4);
Idotout_mod(1,:) = zeros(1,4);

psi = 0;
theta = 0;
phi = 0;

F_x = 0;

total_distance = 0;
longitudinal_distance = 0;
latitudinal_distance = 0;
elevation_distance = 0;
distance_time_step = 0;
long_distance_time_step = 0;
lat_distance_time_step = 0;
elev_distance_time_step = 0;

% -----------------------------------------------------------------------
% Plot segment to choose rover starting point %% RECOMMENDED TO COPY AND PASTE INTO COMMAND WINDOW
% RATHER THAN USING PROMPTS
% figure(3)
% clf
% surf(seg_grid.X, seg_grid.Y, seg_grid.inter_seg','EdgeColor','none')
% % mesh(seg_grid.X, seg_grid.Y, seg_grid.inter_seg')
% grid on
% axis equal
% xlabel 'x-direction (m)'
% ylabel 'y-direction (m)'
% set(gca,'Ydir','reverse')
% colormap(copper)
% 
% % Choose rover starting x/y position
% prompt = ['Initial x-axis position: '];
% x_mod(7) = input(prompt);
% prompt = ['Initial y-axis position: '];
% x_mod(8) = input(prompt);


x_mod(7) = 0;    % initial x
x_mod(8) = 0;    % initial y


% Choose terrain type and terramechanics parameters
prompt = ('Which terrain model: \n1. Friction\n2. Terramechanics\nChoice: ');
terr_choice = input(prompt);
if terr_choice == 2
    prompt = ('Which terrain type: \n1. Regolith\n2. Sand\nChoice: ');
    terr_params = input(prompt);
end

% ----------------------------------------------------------------------
%%

for simtime = dt:dt:maxtime

V = [7.2 7.2 0 7.2];  % Input voltage values

% =========================================================================
% Run the rover model
    
    % MODEL    
    % Run the motor model
    [torques_mod,Idot_mod,Omegadot_mod] = Rover_Motor_Model_v1(V,I_mod,Omega_mod);
    
    torquesout_mod(i,:) = torques_mod;
    
% =========================================================================
% Terramechanics model
% % 
    if terr_choice == 2
        [F_x(1), s_fl] = terramechanics_model(Omega_mod(1), x_mod(1), terr_params); % fl wheel
        [F_x(2), s_bl] = terramechanics_model(Omega_mod(2), x_mod(1), terr_params); % bl wheel
        [F_x(3), s_fr] = terramechanics_model(Omega_mod(3), x_mod(1), terr_params); % fr wheel
        [F_x(4), s_br] = terramechanics_model(Omega_mod(4), x_mod(1), terr_params); % br wheel
        slip(i,:) = [s_fl s_bl s_fr s_br];
        force(i,:) = [F_x(1) F_x(2) F_x(3) F_x(4)];
    end
  
% =========================================================================
    
    % Run model for dynamic/kinematic response of rover system
    [xdot_mod, x_mod, grav, surge] = Mars_Rover_Dynamics(x_mod,torques_mod,unmatched,theta,phi,terr_choice,F_x);
   
% =========================================================================
% Establish outputs

    % MODEL Output Matrices
    xout_mod(i,:) = x_mod;
    psi = xout_mod(i,12);
    xdotout_mod(i,:) = xdot_mod;
    omegaout_mod(i,:) = Omega_mod;
    omegadotout_mod(i,:) = Omegadot_mod;
    Iout_mod(i,:) = I_mod;
    Idotout_mod(i,:) = Idot_mod;
    Vout(i,:) = V;
    time(i) = simtime;
    distance = xout_mod(:,7);

% =========================================================================
% Establish new current variables
    x_mod = x_mod + xdot_mod*dt;
    I_mod = I_mod + Idot_mod*dt;
    Omega_mod = Omega_mod + Omegadot_mod.*dt;
   
% =========================================================================
% Define geometry of wheels in relation to centre of rover

    fl(i,1) = xout_mod(i,7) + l*cos(-xout_mod(i,12)+ang);  % front left wheel x
    fl(i,2) = xout_mod(i,8) - l*sin(-xout_mod(i,12)+ang);  % front left wheel y
    
    fr(i,1) = xout_mod(i,7) + l*cos(xout_mod(i,12)+ang);  % front right wheel x
    fr(i,2) = xout_mod(i,8) + l*sin(xout_mod(i,12)+ang);  % front right wheel y
    
    bl(i,1) = xout_mod(i,7) - l*cos(xout_mod(i,12)+ang);  % back left wheel x
    bl(i,2) = xout_mod(i,8) - l*sin(xout_mod(i,12)+ang);  % back left wheel y
    
    br(i,1) = xout_mod(i,7) - l*cos(-xout_mod(i,12)+ang);  % back right wheel x
    br(i,2) = xout_mod(i,8) + l*sin(-xout_mod(i,12)+ang);  % back right wheel y
  
% =========================================================================
% Define where on terrain grid the rover is

    coord(1,:) = [xout_mod(i,7), xout_mod(i,8)];  % centre of rover
    coord(2,:) = [fl(i,1), fl(i,2)];  % top left wheel  
    coord(3,:) = [fr(i,1), fr(i,2)];  % top right wheel  
    coord(4,:) = [bl(i,1), bl(i,2)];  % back left wheel  
    coord(5,:) = [br(i,1), br(i,2)];  % back right wheel  

% Function to determine height of rover centre and each wheel
    [height_out, new_l] = Rover_Heights(coord,seg_grid,l);
    height(:,i) = height_out;

% Update x/y positions of wheels depending on height

    fl(i,1) = xout_mod(i,7) + new_l(1)*sin(deg2rad(90) + xout_mod(i,12) - ang);  % front left wheel x
    fl(i,2) = xout_mod(i,8) - new_l(1)*cos(deg2rad(90) + xout_mod(i,12) - ang);  % front left wheel y
    
    fr(i,1) = xout_mod(i,7) + new_l(2)*sin(deg2rad(90) - xout_mod(i,12) - ang);  % front right wheel x
    fr(i,2) = xout_mod(i,8) + new_l(2)*cos(deg2rad(90) - xout_mod(i,12) - ang);  % front right wheel y
    
    bl(i,1) = xout_mod(i,7) - new_l(3)*sin(deg2rad(90) - xout_mod(i,12) - ang);  % back left wheel x
    bl(i,2) = xout_mod(i,8) - new_l(3)*cos(deg2rad(90) - xout_mod(i,12) - ang);  % back left wheel y
    
    br(i,1) = xout_mod(i,7) - new_l(4)*sin(deg2rad(90) + xout_mod(i,12) - ang);  % back right wheel x
    br(i,2) = xout_mod(i,8) + new_l(4)*cos(deg2rad(90) + xout_mod(i,12) - ang);  % back right wheel y
    
    coord(1,:) = [xout_mod(i,7), xout_mod(i,8)];  % centre of rover
    coord(2,:) = [fl(i,1), fl(i,2)];  % front left wheel  
    coord(3,:) = [fr(i,1), fr(i,2)];  % front right wheel  
    coord(4,:) = [bl(i,1), bl(i,2)];  % back left wheel  
    coord(5,:) = [br(i,1), br(i,2)];  % back right wheel  

% =========================================================================
% Define Euler angles

    [theta, phi, psi] = Rover_Euler_Angles(height_out,coord);

    yaw(i) = psi;
    psi_deg = rad2deg(psi);
    pitch(i) = theta;
    theta_deg = rad2deg(theta);
    roll(i) = phi;
    phi_deg = rad2deg(phi);


% =========================================================================
% Animation projection

    xyz = [0 0 h]';  % top of wheel 
    % initial wheel coordinates relative to rover centre
    fl_xyz = [l*cos(-ang) l*sin(-ang) 0]';
    fr_xyz = [l*cos(ang) l*sin(ang) 0]';
    bl_xyz = [l*cos(deg2rad(-180)+ang) l*sin(deg2rad(-180)+ang) 0]';
    br_xyz = [l*cos(deg2rad(180)-ang) l*sin(deg2rad(180)-ang) 0]';
    
    % rotation matrices
    yaw_rot = [cos(psi) -sin(psi) 0 ; sin(psi) cos(psi) 0 ; 0 0 1];
    pitch_rot = [cos(theta) 0 -sin(theta) ; 0 1 0 ; sin(theta) 0 cos(theta)];
    roll_rot = [1 0 0 ; 0 cos(phi) sin(phi) ; 0 -sin(phi) cos(phi)];
    
    % rotation matrix multiplication
    XYZ = yaw_rot*pitch_rot*roll_rot*xyz;
    fl_XYZ = yaw_rot*pitch_rot*roll_rot*fl_xyz;
    fr_XYZ = yaw_rot*pitch_rot*roll_rot*fr_xyz;
    bl_XYZ = yaw_rot*pitch_rot*roll_rot*bl_xyz;
    br_XYZ = yaw_rot*pitch_rot*roll_rot*br_xyz;
    
    % coordinate output for top and bottom of rover wheels
    fl_X(i) = xout_mod(i,7)+fl_XYZ(1); fl_Y(i) = xout_mod(i,8)+fl_XYZ(2); fl_Z(i) = height(1,i)+fl_XYZ(3);
    fr_X(i) = xout_mod(i,7)+fr_XYZ(1); fr_Y(i) = xout_mod(i,8)+fr_XYZ(2); fr_Z(i) = height(1,i)+fr_XYZ(3);
    bl_X(i) = xout_mod(i,7)+bl_XYZ(1); bl_Y(i) = xout_mod(i,8)+bl_XYZ(2); bl_Z(i) = height(1,i)+bl_XYZ(3);
    br_X(i) = xout_mod(i,7)+br_XYZ(1); br_Y(i) = xout_mod(i,8)+br_XYZ(2); br_Z(i) = height(1,i)+br_XYZ(3);
    

    fl_vertX(i) = fl_X(i) + XYZ(1); fl_vertY(i) = fl_Y(i) + XYZ(2); fl_vertZ(i) = fl_Z(i) + XYZ(3);
    fr_vertX(i) = fr_X(i) + XYZ(1); fr_vertY(i) = fr_Y(i) + XYZ(2); fr_vertZ(i) = fr_Z(i) + XYZ(3);
    bl_vertX(i) = bl_X(i) + XYZ(1); bl_vertY(i) = bl_Y(i) + XYZ(2); bl_vertZ(i) = bl_Z(i) + XYZ(3);
    br_vertX(i) = br_X(i) + XYZ(1); br_vertY(i) = br_Y(i) + XYZ(2); br_vertZ(i) = br_Z(i) + XYZ(3);


    coord_ver(1,:) = [fl_vertX(i), fl_vertY(i), fl_vertZ(i)];  % front left wheel  
    coord_ver(2,:) = [fr_vertX(i), fr_vertY(i), fr_vertZ(i)];  % front right wheel 
    coord_ver(3,:) = [bl_vertX(i), bl_vertY(i), bl_vertZ(i)];  % back left wheel  
    coord_ver(4,:) = [br_vertX(i), br_vertY(i), br_vertZ(i)];  % back right wheel  


% =========================================================================
% Metric counter

    if i >= 2
        distance_time_step = sqrt((xout_mod(i,7)-xout_mod(i-1,7))^2+(xout_mod(i,8)-xout_mod(i-1,8))^2+(height(1,i)-height(1,i-1))^2);
        long_distance_time_step = xout_mod(i,7) - xout_mod(i-1,7);
        lat_distance_time_step = xout_mod(i,8) - xout_mod(i-1,8);
        elev_distance_time_step = height(1,i)-height(1,i-1);
    end
    total_distance = total_distance + distance_time_step;
    longitudinal_distance = longitudinal_distance + long_distance_time_step;
    latitudinal_distance = latitudinal_distance + lat_distance_time_step;
    elevation_distance = elevation_distance + elev_distance_time_step;
    dist(i) = total_distance;
    long_dist(i) = longitudinal_distance;
    lat_dist(i) = latitudinal_distance;
    elev_dist(i) = elevation_distance;
    speed(i) = distance_time_step/dt;


% =========================================================================
% Set rover constraints

    % max incline angles
    if abs(theta) >= abs(tipping_angle_pitch) || abs(phi) >= abs(tipping_angle_roll)
        fprintf('Rover has tipped over')
        break
    end
    
    % max incline angles for terramechanics model 
    if terr_choice == 2
     if terr_params == 1 && theta >= stop_angle_regolith
        fprintf('Rover has stopped\n')
        break
     end
     if terr_params == 2 && theta >= stop_angle_sand
        fprintf('Rover has stopped\n')
        break
     end
    end

% =========================================================================
% Output counter
    i = i + 1;  
    
    
    
end


%% Outputs

% =========================================================================
% PLOTS

% figure (2)
% clf
% subplot(4,2,1)
% plot(time,xout_mod(:,1))
% xlabel('time [s]')
% ylabel('surge [m/s]')
% subplot(4,2,2)
% plot(time,xout_mod(:,2))
% xlabel('time [s]')
% ylabel('sway [m/s]')
% subplot(4,2,3)
% plot(time,xout_mod(:,6)*180/pi)
% xlabel('time [s]')
% ylabel('yaw rate [m/s]')
% subplot(4,2,4)
% plot(time,xout_mod(:,12)*180/pi)
% xlabel('time [s]')
% ylabel('psi [m/s]')
% subplot(4,4,9)
% plot(time,Vout(:,1))
% xlabel('time [s]')
% ylabel('V1 [V]')
% subplot(4,4,10)
% plot(time,Vout(:,2))
% xlabel('time [s]')
% ylabel('V2 [V]')
% subplot(4,4,13)
% plot(time,Vout(:,3))
% xlabel('time [s]')
% ylabel('V3 [V]')
% subplot(4,4,14)
% plot(time,Vout(:,4))
% xlabel('time [s]')
% ylabel('V4 [V]')
% subplot(2,2,4)
% plot(xout_mod(:,8),xout_mod(:,7))
% hold on
% grid on
% plot(fl(:,2),fl(:,1))
% plot(fr(:,2),fr(:,1))
% plot(bl(:,2),bl(:,1))
% plot(br(:,2),br(:,1))
% legend('centre','front left','front right')
% xlabel('y-pos [m]')
% ylabel('x-pos [m]')

% % Plot 3D view of rover path over terrain
% figure(3)
% clf
% plot3(xout_mod(:,7),xout_mod(:,8),height(1,:),'r','LineWidth',4)
% hold on
% surf(seg_grid.X, seg_grid.Y, seg_grid.inter_seg','EdgeColor','none')
% % mesh(seg_grid.X, seg_grid.Y, seg_grid.inter_seg')
% grid on
% axis equal
% ylim([407 700])
% set(gca,'FontSize',15)
% xlabel 'x-direction (m)'
% ylabel 'y-direction (m)'
% zlabel 'Elevation (m)'
% % legend('Rover centre','Rover fl wheel','Rover fr wheel','Rover bl wheel','Rover br wheel')
% legend('Rover Path')
% set(gca,'Ydir','reverse')
% % colormap(copper)
% c = colorbar;
% c.Label.String = 'Elevation (m)';
% 
% 
% % Plot change in speed of rover along with Euler angles
% figure(4)
% clf
% subplot(2,2,1)
% plot(time,speed,'LineWidth',5)
% grid on
% set(gca,'FontSize',20)
% xlim([0 maxtime])
% xlabel 'time (s)'
% ylabel 'velocity (m/s)'
% title('Rover Forward Speed')
% 
% subplot(2,2,3)
% plot(time,dist,'LineWidth',3)
% hold on
% plot(time,long_dist,'LineWidth',3)
% plot(time,lat_dist,'LineWidth',3)
% plot(time,elev_dist,'Linewidth',3)
% grid on
% set(gca,'FontSize',20)
% xlim([0 maxtime])
% % ylim([-50 400])
% xlabel 'time (s)'
% ylabel 'distance (m)'
% title('Distance Travelled')
% legend('Total Distance', 'Longitudinal Distance', 'Latitudinal Distance','Elevation Distance')
% 
% subplot(3,2,2)
% plot(time,rad2deg(roll),'r','LineWidth',5)
% set(gca,'FontSize',13)
% grid on
% xlim([0 maxtime])
% % ylim([9 22])
% xlabel('time (s)')
% ylabel('phi (deg)')
% title('Roll')
% 
% subplot(3,2,4)
% plot(time,rad2deg(pitch),'b','LineWidth',5)
% set(gca,'FontSize',13)
% grid on
% xlim([0 maxtime])
% % ylim([11 22])
% xlabel('time (s)')
% ylabel('theta (deg)')
% title('Pitch')
% 
% subplot(3,2,6)
% plot(time,rad2deg(yaw),'g','LineWidth',5)
% set(gca,'FontSize',13)
% grid on
% xlim([0 maxtime])
% % ylim([1 7])
% xlabel('time (s)')
% ylabel('psi (deg)')
% title('Yaw')
% 
% % Plot 2D views of rover
% figure(5)
% clf
% subplot(2,1,1)
% plot(xout_mod(:,7),height(1,:),'LineWidth',5)
% set(gca,'FontSize',18)
% grid on
% xlim([-2150 -1778])
% ylim([-5 65])
% xlabel('x-axis (m)')
% ylabel('z-axis (m)')
% title('Rover traverse side view')
% subplot(2,1,2)
% plot(xout_mod(:,7),xout_mod(:,8),'LineWidth',5)
% set(gca,'FontSize',18)
% grid on
% xlim([-2150 -1778])
% ylim([540 560])
% xlabel('x-axis (m)')
% ylabel('y-axis (m)')
% title('Rover traverse top down view')

% =========================================================================
% DATA

fprintf('Total distance travelled: %fm\n',total_distance);
fprintf('Longitudinal distance: %fm\n',longitudinal_distance);
fprintf('Latitudinal distance: %fm\n',latitudinal_distance);
fprintf('Elevation span: %fm\n',max(elev_dist)-min(elev_dist));
fprintf('Average speed: %fm/s\n',mean(speed));
fprintf('Top speed: %fm/s\n',max(speed));
fprintf('Max roll angle: %fdeg\n',rad2deg(max(roll)));
fprintf('Max pitch angle: %fdeg\n',rad2deg(max(pitch)));
fprintf('Max yaw angle: %fdeg\n',rad2deg(max(yaw)));
fprintf('Min roll angle: %fdeg\n',rad2deg(min(roll)));
fprintf('Min pitch angle: %fdeg\n',rad2deg(min(pitch)));
fprintf('Min yaw angle: %fdeg\n',rad2deg(min(yaw)));

% % =========================================================================
% Choose to run animation

prompt = ('Run animation?\n1. Yes\n2. No\n');
run_animation = input(prompt);


if run_animation == 1
for i = 1:length(time)
    
    
 coord_ver(1,:) = [fl_vertX(i), fl_vertY(i), fl_vertZ(i)];  % front left wheel  
 coord_ver(2,:) = [fr_vertX(i), fr_vertY(i), fr_vertZ(i)];  % front right wheel 
 coord_ver(3,:) = [bl_vertX(i), bl_vertY(i), bl_vertZ(i)];  % back left wheel  
 coord_ver(4,:) = [br_vertX(i), br_vertY(i), br_vertZ(i)];  % back right wheel  
 
pts = [fr_X(i) fr_Y(i) fr_Z(i) ; fl_X(i) fl_Y(i) fl_Z(i)];  % f
pts2 = [br_X(i) br_Y(i) br_Z(i) ; bl_X(i) bl_Y(i) bl_Z(i)];  % b
pts3 = [fr_X(i) fr_Y(i) fr_Z(i) ; br_X(i) br_Y(i) br_Z(i)];  % r
pts4 = [fl_X(i) fl_Y(i) fl_Z(i) ; bl_X(i) bl_Y(i) bl_Z(i)];  % l
pts_x = [coord_ver(2,1) coord_ver(2,2) coord_ver(2,3) ; coord_ver(1,1) coord_ver(1,2) coord_ver(1,3)];  % f+
pts2_x = [coord_ver(4,1) coord_ver(4,2) coord_ver(4,3) ; coord_ver(3,1) coord_ver(3,2) coord_ver(3,3)];  % b+
pts3_x = [coord_ver(2,1) coord_ver(2,2) coord_ver(2,3) ; coord_ver(4,1) coord_ver(4,2) coord_ver(4,3)];  % r+
pts4_x = [coord_ver(1,1) coord_ver(1,2) coord_ver(1,3) ; coord_ver(3,1) coord_ver(3,2) coord_ver(3,3)];  % l+
pts_y = [fr_X(i) fr_Y(i) fr_Z(i) ; coord_ver(2,1) coord_ver(2,2) coord_ver(2,3)];  % fr vertical
pts2_y = [fl_X(i) fl_Y(i) fl_Z(i) ; coord_ver(1,1) coord_ver(1,2) coord_ver(1,3)];  % fl vertical
pts3_y = [br_X(i) br_Y(i) br_Z(i) ; coord_ver(4,1) coord_ver(4,2) coord_ver(4,3)];  % br vertical
pts4_y = [bl_X(i) bl_Y(i) bl_Z(i) ; coord_ver(3,1) coord_ver(3,2) coord_ver(3,3)];  % bl vertical

 
figure(1)
clf
set(gcf, 'Position', get(0, 'Screensize'));
plot3(pts(:,1), pts(:,2), pts(:,3),'b','LineWidth',1.5)
hold on
plot3(pts2(:,1), pts2(:,2), pts2(:,3),'r','LineWidth',1.5)
plot3(pts3(:,1), pts3(:,2), pts3(:,3),'r','LineWidth',1.5)
plot3(pts4(:,1), pts4(:,2), pts4(:,3),'r','LineWidth',1.5)
plot3(pts_x(:,1), pts_x(:,2), pts_x(:,3),'b','LineWidth',1.5)
plot3(pts2_x(:,1), pts2_x(:,2), pts2_x(:,3),'b','LineWidth',1.5)
plot3(pts3_x(:,1), pts3_x(:,2), pts3_x(:,3),'b','LineWidth',1.5)
plot3(pts4_x(:,1), pts4_x(:,2), pts4_x(:,3),'b','LineWidth',1.5)
plot3(pts_y(:,1), pts_y(:,2), pts_y(:,3),'g','LineWidth',1.5)
plot3(pts2_y(:,1), pts2_y(:,2), pts2_y(:,3),'g','LineWidth',1.5)
plot3(pts3_y(:,1), pts3_y(:,2), pts3_y(:,3),'g','LineWidth',1.5)
plot3(pts4_y(:,1), pts4_y(:,2), pts4_y(:,3),'g','LineWidth',1.5)
% surf(seg_grid.X, seg_grid.Y, seg_grid.inter_seg','EdgeColor','none')
mesh(seg_grid.X, seg_grid.Y, seg_grid.inter_seg','LineWidth',1.5)
axis([xout_mod(i,7)-0.75 xout_mod(i,7)+0.75 xout_mod(i,8)-0.75 xout_mod(i,8)+0.75 height(1,i)-0.75 height(1,i)+0.75])
% axis equal
xlabel 'x-direction (m)'
ylabel 'y-direction (m)'
zlabel 'Elevation (m)'
set(gca,'Ydir','reverse','FontSize',20)
grid on
% colormap(copper)


end
end
    
