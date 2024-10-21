
%% Find the force applied at each tether to achieve a resultant force equal to the user's weight
% in the +Z direction
% Assumes tethers all connect directly to the CG of the user
% Assumes tethers are mounted 120° from one another

clear
clc
close all

mounting_radius=2*12*25.4/1000; % 2 foot mounting radius in meters
cg_location=[0 0 0]; % initial CG location of user is origin of coord system
user_height=6*12*25.4/1000; % 6 feet in meters
user_weight=200*4.448; % 200 lbf in N
tether_1_base=[0 mounting_radius user_height/2]; % assume tether 1 is located on the user's right along the +y axis
tether_2_base=[mounting_radius*cosd(210) mounting_radius*sind(210) user_height/2]; % other tethers are 120 deg apart
tether_3_base=[mounting_radius*cosd(330) mounting_radius*sind(330) user_height/2]; % other tethers are 120 deg apart

desired_force=[0;0;user_weight];

% vary cg position in x, y

x_range=-.25:.01:.25;
y_range=-.25:.01:.25;

i=1;
j=1;

applied_force=cell(length(x_range),length(y_range)); % preallocate cell array to store compntns of forces and CG location

for x_coord=x_range % vary cg positiion over specified x range
    j=1;
    for y_coord=y_range % vary cg positiion over specified y range

        cg_location=[x_coord, y_coord, 0];
        tether_1_vector=tether_1_base-cg_location;
        tether_2_vector=tether_2_base-cg_location;
        tether_3_vector=tether_3_base-cg_location;
        
        unit_vec_f1=tether_1_vector/norm(tether_1_vector);
        unit_vec_f2=tether_2_vector/norm(tether_2_vector);
        unit_vec_f3=tether_3_vector/norm(tether_3_vector);

        k_matrix=[unit_vec_f1(1) unit_vec_f2(1) unit_vec_f3(1);...
                  unit_vec_f1(2) unit_vec_f2(2) unit_vec_f3(2);... 
                  unit_vec_f1(3) unit_vec_f2(3) unit_vec_f3(3)]; % compnt of tether force in each direction is unit vec*F
        
        applied_force{i,j}(1:3)=k_matrix\desired_force; % solve system of equations for tether forces
        applied_force{i,j}(4:6)=cg_location; % store cg position with each set of forces for ease of use later
        j=j+1;

    end

    i=i+1;

end

% Plot forces for sway in pure X and pure Y

% pure y sway
for k=1:51
    t1_forces_y(k)=applied_force{26,k}(1);
    t2_forces_y(k)=applied_force{26,k}(2);
    t3_forces_y(k)=applied_force{26,k}(3);
    y_cg(k)=applied_force{26,k}(5);
end
figure(1);
hold on;
plot(y_cg,t1_forces_y);
plot(y_cg,t2_forces_y);
plot(y_cg,t3_forces_y);
legend('T1 Tension','T2 Tension','T3 Tension');
title('Tether Tension vs. CG Y Coordinate');
xlabel('Y coordinate');
ylabel('Force (N)');


% pure x sway
for k=1:51
    t1_forces_x(k)=applied_force{k,26}(1);
    t2_forces_x(k)=applied_force{k,26}(2);
    t3_forces_x(k)=applied_force{k,26}(3);
    x_cg(k)=applied_force{k,26}(4);
end
figure(2);
hold on;
plot(x_cg,t1_forces_x);
plot(x_cg,t2_forces_x);
plot(x_cg,t3_forces_x);
legend('T1 Tension','T2 Tension','T3 Tension');
title('Tether Tension vs. CG X Coordinate');
xlabel('X coordinate');
ylabel('Force (N)');

% Determine max force required of a single actuator

for b=1:51
    for c=1:51
        max_force(b,c)=max(applied_force{b,c});
    end
end

actual_max_force=max(max(max_force));
actual_max_force_lbf=actual_max_force/4.448;

