% this runs error calculations assuming that tension is each tether is kept
% constant at every point

clear
close all
clc

% everything is is feet

% tether floor anchor points
% tether_pts = [2, 0, 0;
%               -2, 0, 0];
tether_pts = [0,           2,           0;
              2*cosd(210), 2*sind(210), 0;
              2*cosd(330), 2*sind(330), 0;];



% tallest
% center of mass
COM_init_height = 3;
% mass (pounds)
mass = 200; 

% % shortest
% % center of mass
% COM_init_height = 2.5;
% % mass (pounds)
% mass = 113; 

% max allowable oscillation distance and frequency each direction
x_A = COM_init_height*tand(10);
x_f = 0;
y_A = COM_init_height*tand(10);
y_f = 1;
z_A = 0.82021; % 0.25 meters
z_f = 0;

% center of mass location and rate of change
COM_x = @(t) x_A*sin(2*pi*x_f*t);
COM_y = @(t) y_A*sin(2*pi*y_f*t);
COM_z = @(t) z_A*sin(2*pi*z_f*t);

t = linspace(0, 2, 1000)';

COM = [COM_x(t), COM_y(t), COM_z(t) + COM_init_height];

expected_force_vec = [0, 0, -mass];

% calculate equilibrium tensions
teth1_vec_eq = tether_pts(1,:) - COM(1,:);
teth2_vec_eq = tether_pts(2,:) - COM(1,:);
teth3_vec_eq = tether_pts(3,:) - COM(1,:);
teth1_hat_eq = teth1_vec_eq/norm(teth1_vec_eq);
teth2_hat_eq = teth2_vec_eq/norm(teth2_vec_eq);
teth3_hat_eq = teth3_vec_eq/norm(teth3_vec_eq);
M1 = [teth1_hat_eq(1), teth2_hat_eq(1), teth3_hat_eq(1);
      teth1_hat_eq(2), teth2_hat_eq(2), teth3_hat_eq(2);
      teth1_hat_eq(3), teth2_hat_eq(3), teth3_hat_eq(3)];
M2 = [0;0;-mass];
f_eq = M1\M2;

f = zeros(length(t), size(tether_pts, 1));
ang_err = zeros(1, length(t));
act_force_mag = zeros(1, length(t));
for i = 1:length(t)
    teth1_vec = tether_pts(1,:) - COM(i,:);
    teth2_vec = tether_pts(2,:) - COM(i,:);
    teth3_vec = tether_pts(3,:) - COM(i,:);
    teth1_hat = teth1_vec/norm(teth1_vec);
    teth2_hat = teth2_vec/norm(teth2_vec);
    teth3_hat = teth3_vec/norm(teth3_vec);

    teth1_vec = f_eq(1)*teth1_hat;
    teth2_vec = f_eq(1)*teth2_hat;
    teth3_vec = f_eq(1)*teth3_hat;

    act_force_vec = teth1_vec + teth2_vec + teth3_vec;
    ang_err(i) = acosd(dot(act_force_vec, expected_force_vec)/(norm(act_force_vec)*norm(expected_force_vec)));
    act_force_mag(i) = norm(act_force_vec);
    
end


figure(1)
plot(t, ang_err)
title("angular error with constant tether tension")
xlabel("time (s)")
ylabel("angle error (deg)")

figure(2)
plot(t, act_force_mag)
hold on
plot(t, mass*ones(1, length(t)))
title("force error with constant tether tension")
xlabel("time (s)")
ylabel("force error (lbf)")