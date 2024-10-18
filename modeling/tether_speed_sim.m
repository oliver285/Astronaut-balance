clear
close all
clc
% simulating the tether speed required assuming an immediate changing in
% tether length to follow center of mass oscilations
% tether forces is implemented for 2 tethers but not three yet

% everything is is feet

% tether floor anchor points
tether_pts = [2, 0, 0;
              -2, 0, 0];
% tether_pts = [2, 0, 0;
%               -2, 0, 0;
%               0, 2, 0;
%               0, -2, 0];

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
z_A = 0.82021; % 0.25 meters
z_f = 0;

% center of mass location and rate of change
COM_x = @(t) x_A*sin(2*pi*x_f*t);
COM_y = @(t) y_A*sin(2*pi*y_f*t);
COM_z = @(t) z_A*sin(2*pi*z_f*t);

t = linspace(0, 5, 1000)';

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

