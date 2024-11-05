clear
close all
clc
% simulating the tether speed required assuming an immediate changing in
% tether length to follow center of mass oscilations

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
y_f = 0;
z_A = 0.82021*2; % 0.25 meters
z_f = 1;

% center of mass location and rate of change
COM_x = @(t) x_A*sin(2*pi*x_f*t);
COM_y = @(t) y_A*sin(2*pi*y_f*t);
COM_z = @(t) z_A*sin(2*pi*z_f*t);

t = linspace(0, 2, 1000)';

COM = [COM_x(t), COM_y(t), COM_z(t) + COM_init_height];

% initialize tether matrices
tether_L = zeros(length(t), size(tether_pts, 1));
tether_L_dot = zeros(length(t)-1, size(tether_pts, 1));

for i = 1:size(tether_pts, 1)
    % magnitude of COM-tether_anchor_point
    tether_L(:,i) = sqrt(sum((COM - tether_pts(i,:)).^2,2));
    % numerically differentiated tether retraction and extension speed
    tether_L_dot(:,i) = diff(tether_L(:,i))./diff(t);
end


% do force calculations for different modes
f = zeros(length(t), size(tether_pts, 1));
if(size(tether_pts, 1) == 2)
    % 1 dimensional oscillations for a two tether setup
    for i = 1:length(t)
        theta1 = acosd(norm(tether_pts(1,:))/tether_L(i,1));
        theta2 = acosd(norm(tether_pts(2,:))/tether_L(i,2));
        M1 = [sind(theta1), sind(theta2);
              cosd(theta1), -cosd(theta2)];
        M2 = [mass; 0];
        f(i,:) = M1\M2;
    end
elseif (size(tether_pts, 1) == 3)
    for i = 1:length(t)
        teth1_vec = tether_pts(1,:) - COM(i,:);
        teth2_vec = tether_pts(2,:) - COM(i,:);
        teth3_vec = tether_pts(3,:) - COM(i,:);
        teth1_hat = teth1_vec/norm(teth1_vec);
        teth2_hat = teth2_vec/norm(teth2_vec);
        teth3_hat = teth3_vec/norm(teth3_vec);
        M1 = [teth1_hat(1), teth2_hat(1), teth3_hat(1);
              teth1_hat(2), teth2_hat(2), teth3_hat(2);
              teth1_hat(3), teth2_hat(3), teth3_hat(3)];
        M2 = [0;0;mass];
        f(i,:) = M1\M2;

    end
end


% for the legend
for i = 1:size(tether_pts, 1)
    leg(i) = "tether " + i;
end

figure(1)
plot(t, tether_L)
title("tether length")
xlabel("time (s)")
ylabel("tether length (ft)")
legend(leg)

figure(2)
plot(t(1:end-1), tether_L_dot)
title("tether extension/retraction speed")
xlabel("time (s)")
ylabel("tether speed (ft/s)")
legend(leg)

if(size(tether_pts, 1) == 2)
    figure(3)
    plot(t, f(:,1))
    hold on
    plot(t, f(:,2))
    title("tether forces")
    xlabel("time (s)")
    ylabel("tether force")
    legend(leg)
end
if(size(tether_pts, 1) == 3)
    figure(3)
    plot(t, f(:,1))
    hold on
    plot(t, f(:,2))
    hold on
    plot(t, f(:,3))
    title("tether forces")
    xlabel("time (s)")
    ylabel("tether force")
    legend(leg)
end

