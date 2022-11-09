% Inputs: Wheel rotation velocity, Rover velocity, Parameter choice
% Outputs: x-axis force, slip ratio

function [F_x, s] = terramechanics_model(omega, speed, choice)


% Define soil parameters

% Lunar Simulant
beta = deg2rad(5);
n = 1;  % Sinkage exponent
k_phi = 8.14e5;  % Frictional sinkage modulus
k_c = 1.37e3;  % Cohesive sinkage modulus
c = 0.8e3;  % Soil cohesion stress
phi = deg2rad(37.2);  % Angle of internal friction
rho = 1600;  % %%%%%%Need value for this
k_x = 0.043*beta+0.036;
k_y = 0.02;
lambda = 0.9;
a0 = 0.4;
a1 = 0.15;


if choice ==1
% Mars regolith
n0 = 1.4;  % Sinkage exponent
n1 = 0.354;
k_phi_rw = 677.5;  % Frictional sinkage modulus
k_c_rw = 212.2;  % Cohesive sinkage modulus
c = 0.6e3;  % Soil cohesion stress
phi = deg2rad(35);  % Angle of internal friction
rho = 1550;  % Soil density
k_x = 0.0146;  % Soil deformation
k_y = 0.0146;  
lambda = 0.07;  % Sinkage ratio
a0 = 0.365;
a1 = 0.503;

elseif choice ==2

% Mars sand
n0 = 1.4;  % Sinkage exponent
n1 = 0.45;
k_phi_rw = 500.8;  % Frictional sinkage modulus
k_c_rw = 9.1;  % Cohesive sinkage modulus
c = 0.2e3;  % Soil cohesion stress
phi = deg2rad(30);  % Angle of internal friction
rho = 1650;  % Soil density
k_x = 0.029;  % Soil deformation
k_y = 0.029;
lambda = 0.05;   % Sinkage ratio
a0 = 0.33;
a1 = 0.11;

end

% Define rover parameters
m = 2.148;
g = 3.721;
No_wheels = 4;
r = 0.0625;
b = 0.065;


% Calculate slip ratio
if abs(r*omega) >= abs(speed)  % Driving
    s = (r*omega-speed)/(r*omega);
elseif abs(r*omega) < abs(speed)  % Braking
    s = (r*omega-speed)/speed;
end

% Slip constraints
if s > 1
    s = 1;
end
if s < -1
    s = -1;
end

if speed == 0 && omega ~=0
    s = 1;
end

if speed < 0 && omega > 0
    s = 1;
end

if speed == 0 && omega == 0
    s = 0;
end

% if s == -1
%     s = 1;
% end

% Equations not reliant on loops

% Ks_b = k_c/b + k_phi;
Ks_rw = c*k_c_rw + rho*g*b*k_phi_rw;
F_n = (m*g)/(No_wheels);
X_c = deg2rad(45)-phi/2;
D1 = cot(X_c)+tan(X_c+phi);
D2 = cot(X_c)+(cot(X_c).^2)/cot(phi);
n = n0 + n1*s;

% norm_stress_bekker = r^n*Ks_b;
norm_stress_rw = (r/b)^n*Ks_rw;

% Initial sinkage estimate

% z_b_guess = (F_n/(sqrt(2*r)*b*Ks_b*(3-n)*(1-n/3)))^(2/(2*n+1));
z_b_guess = 0.00001;
z_b = 0;

% Define step sizes

theta_step = 0.0002;  % angle step size
sink_step = 0.00001;  % sinkage step size

for z_b = z_b_guess:sink_step:r  % cycle from initial sinkage guess until F_z = F_n

% Clear forces each loop    
F_x = 0;
F_y = 0;
F_z = 0;

    
% Calculate contact angles
theta_f = acos(1-z_b/r);  % entry angle
theta_r = -acos(1-((lambda*z_b)/r));  % exit angle
theta_m = (a0+a1*s)*theta_f;  % angle of max stress
theta = theta_r:theta_step:theta_f;
num =  (theta_f-theta_r)/theta_step;  % number of points

% cycle through theta values
    for k = 1:(num+1)
        
        % calculate normal stress at each theta value
        if theta(k) >= theta_m && theta(k) < theta_f
%             sigma(k) = norm_stress_bekker*(cos(theta(k)) - cos(theta_f))^n;
            sigma(k) = norm_stress_rw*(cos(theta(k)) - cos(theta_f))^n;
        elseif theta(k) >= theta_r && theta(k) < theta_m
%             sigma(k) = norm_stress_bekker*(cos(theta_f - ((theta(k) - theta_r)/(theta_m - theta_r))*(theta_f - theta_m)) - cos(theta_f))^n;
            sigma(k) = norm_stress_rw*(cos(theta_f - ((theta(k) - theta_r)/(theta_m - theta_r))*(theta_f - theta_m)) - cos(theta_f))^n;
        end 
        
        % calculate shear stress at each theta value
        j_x(k) = r*(theta_f-theta(k)-(1-s)*(sin(theta_f)-sin(theta(k))));
        j_y(k) = r*(1-s)*(theta_f-theta(k))*tan(beta);
        tau_x(k) = (c + sigma(k)*tan(phi))*(1 - exp(-j_x(k)/k_x));
        tau_y(k) = (c+sigma(k)*tan(phi))*(1-exp(-j_y(k)/k_y));

        R_b = D1*(c*z_b + D2*(rho*z_b^2/2));

        % calculate forces
        F_x = F_x + b*r*(tau_x(k)*cos(theta(k))-sigma(k)*sin(theta(k)))*theta_step;
        F_y = F_y + R_b*(r - z_b*cos(theta(k)))*theta_step;
        F_z = F_z + b*r*(sigma(k)*cos(theta(k))+tau_x(k)*sin(theta(k)))*theta_step;
      
    end
    
% Set vertical force accuracy boundary    
if F_z <= F_n + F_n*0.06 && F_z >= F_n - F_n*0.06
    break
end

end

if omega == 0
    F_x = 0;

end

