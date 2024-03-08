%% Homework 2
%% Jacob Faile

%% PART A

la=1;
lb=0.2;
lc=1;

%% theta values (uncomment the ones that you want to process)
theta1_val = 0;
theta2_val = -90;
theta3_val = 90;

% theta1_val = 45;
% theta2_val = -60;
% theta3_val = 120;
% 
% theta1_val = 45;
% theta2_val = 60;
% theta3_val = -120;
% 
% theta1_val = 90;
% theta2_val = 45;
% theta3_val = 90;
% 
% theta1_val = 180;
% theta2_val = -30;
% theta3_val = -60;

%% joint angles
theta1 = theta1_val/360*2*pi;
theta2 = theta2_val/360*2*pi;
theta3 = theta3_val/360*2*pi;

%% Redefined homogeneous transformation as functions of joint angles: theta1,2,3=0,-90,90

T01=[    cos(theta1)     -sin(theta1)     0     0;
     sin(theta1)     cos(theta1)     0     0;
     0     0     1     0;
     0     0     0     1;
     ];

T12=[   cos(theta2)   -sin(theta2) 0  0;
         0         0    1         0;
    -sin(theta2)   -cos(theta2)         0         0;
         0         0         0    1;
         ];

T23=[     cos(theta3)   -sin(theta3)         0    la;
    sin(theta3)    cos(theta3)         0         0;
         0         0    1         0;
         0         0         0    1;
         ];

T02 = T01*T12;
T03 = T01*T12*T23;

%% end-point location

%Endpoint wrt Frame 3
r3=[lc;0;lb;1];
r0=T03*r3;


%% extract joint locations and draw manipulator

%Oo-O3-Om-O4
%Om is between O3 and O4
rm=T03*[0;0;lb;1];

endpoint = [r0(1), r0(2), r0(3)];

figure(1)
hold on
plot3([0.1 -0.1 -0.1 0.1 0.1],[0.1 0.1 -0.1 -0.1 0.1],[0 0 0 0 0])  %base square
plot3([0 T03(1,4) rm(1) r0(1)],[0 T03(2,4) rm(2) r0(2)],[0 T03(3,4) rm(3) r0(3)])
plot3(r0(1),r0(2),r0(3),'o');
axis equal
%axis([-1 1 -1 1 -0.5 1.5])

grid
hold off
view(-30,30)


%% PART B
%% inverse kinematics solutions
rx = 0.3;
ry = 0.3;
rz = 0.1;
r0 = [rx; ry; rz; 1];

%% Define equations

k = ((rx^2 + ry^2 + rz^2 - lb^2 + la^2 + lc^2)^2 - 2*((rx^2 + ry^2 + rz^2 - lb^2)^2 + la^4 + lc^4))^(1/2);

inv_theta1_plus = atan2(-rx, ry) + atan2(sqrt(rx^2 + ry^2 - lb^2), lb);
inv_theta1_minus = atan2(-rx, ry) - atan2(sqrt(rx^2 + ry^2 - lb^2), lb);

inv_theta2_plus_top = atan2(-rz, sqrt(rx^2 + ry^2 - lb^2)) + atan2(k, rx^2 + ry^2 + rz^2 - lb^2 + la^2 - lc^2);
inv_theta2_minus_top = atan2(-rz, sqrt(rx^2 + ry^2 - lb^2)) - atan2(k, rx^2 + ry^2 + rz^2 - lb^2 + la^2 - lc^2);

inv_theta2_plus_bot = atan2(-rz, -sqrt(rx^2 + ry^2 - lb^2)) + atan2(k, rx^2 + ry^2 + rz^2 - lb^2 - la^2 - lc^2);
inv_theta2_minus_bot = atan2(-rz, -sqrt(rx^2 + ry^2 - lb^2)) - atan2(k, rx^2 + ry^2 + rz^2 - lb^2 - la^2 - lc^2);

inv_theta3_plus = atan2(k, rx^2 + ry^2 + rz^2 - lb^2 - la^2 - lc^2);
inv_theta3_minus = -atan2(k, rx^2 + ry^2 + rz^2 - lb^2 - la^2 - lc^2);

%% Theta values (uncomment the ones that you want to process)

theta1 = inv_theta1_plus;
theta2 = inv_theta2_minus_top;
theta3 = inv_theta3_plus;

% theta1 = inv_theta1_plus;
% theta2 = inv_theta2_minus_bot;
% theta3 = inv_theta3_plus;

% theta1 = inv_theta1_minus;
% theta2 = inv_theta2_plus_top;
% theta3 = inv_theta3_minus;

% theta1 = inv_theta1_minus;
% theta2 = inv_theta2_plus_bot;
% theta3 = inv_theta3_minus;

theta1_deg = rad2deg(theta1);
theta2_deg = rad2deg(theta2);
theta3_deg = rad2deg(theta3);

thetas = [theta1_deg; theta2_deg; theta3_deg];

T01=[    cos(theta1)     -sin(theta1)     0     0;
     sin(theta1)     cos(theta1)     0     0;
     0     0     1     0;
     0     0     0     1;
     ];

T12=[   cos(theta2)   -sin(theta2) 0  0;
         0         0    1         0;
    -sin(theta2)   -cos(theta2)         0         0;
         0         0         0    1;
         ];

T23=[     cos(theta3)   -sin(theta3)         0    la;
    sin(theta3)    cos(theta3)         0         0;
         0         0    1         0;
         0         0         0    1;
         ];

T02 = T01*T12;
T03 = T01*T12*T23;

rm=T03*[0;0;lb;1];

figure(2)
hold on
plot3([0.1 -0.1 -0.1 0.1 0.1],[0.1 0.1 -0.1 -0.1 0.1],[0 0 0 0 0])  %base square
plot3([0 T03(1,4) rm(1) r0(1)],[0 T03(2,4) rm(2) r0(2)],[0 T03(3,4) rm(3) r0(3)])
plot3(r0(1),r0(2),r0(3),'o');
axis equal
%axis([-1 1 -1 1 -0.5 1.5])

grid
hold off
view(-30,30)


%% PART C
%% Circular trajectory plotting and theta calculation

% Initialize k values
k_values = 0:12;

% Define T(k) function
T = @(k) 2*k*pi/12;

% Create a figure
figure(3);

% Iterate through k values
for k_val = k_values
    
    % Calculate T(k)
    Tk = T(k_val);

    xk = 0.5 + 0.2* cos(Tk);
    yk = 0.2 * sin(Tk);
    zk = 0;
    
    rx = xk;
    ry = yk;
    rz = zk;
    r0 = [rx; ry; rz; 1];
    
    %% Define equations
    
    k = ((rx^2 + ry^2 + rz^2 - lb^2 + la^2 + lc^2)^2 - 2*((rx^2 + ry^2 + rz^2 - lb^2)^2 + la^4 + lc^4))^(1/2);
    
    inv_theta1_plus = atan2(-rx, ry) + atan2(sqrt(rx^2 + ry^2 - lb^2), lb);
    inv_theta1_minus = atan2(-rx, ry) - atan2(sqrt(rx^2 + ry^2 - lb^2), lb);
    
    inv_theta2_plus_top = atan2(-rz, sqrt(rx^2 + ry^2 - lb^2)) + atan2(k, rx^2 + ry^2 + rz^2 - lb^2 + la^2 - lc^2);
    inv_theta2_minus_top = atan2(-rz, sqrt(rx^2 + ry^2 - lb^2)) - atan2(k, rx^2 + ry^2 + rz^2 - lb^2 + la^2 - lc^2);
    
    inv_theta2_plus_bot = atan2(-rz, -sqrt(rx^2 + ry^2 - lb^2)) + atan2(k, rx^2 + ry^2 + rz^2 - lb^2 - la^2 - lc^2);
    inv_theta2_minus_bot = atan2(-rz, -sqrt(rx^2 + ry^2 - lb^2)) - atan2(k, rx^2 + ry^2 + rz^2 - lb^2 - la^2 - lc^2);
    
    inv_theta3_plus = atan2(k, rx^2 + ry^2 + rz^2 - lb^2 - la^2 - lc^2);
    inv_theta3_minus = -atan2(k, rx^2 + ry^2 + rz^2 - lb^2 - la^2 - lc^2);
    
    %% Theta values (uncomment the ones that you want to process)
    
    % just using one inverse kinematic solution
    theta1 = inv_theta1_plus;
    theta2 = inv_theta2_minus_top;
    theta3 = inv_theta3_plus;
    
    theta1_deg = rad2deg(theta1);
    theta2_deg = rad2deg(theta2);
    theta3_deg = rad2deg(theta3);
    
    theta_rads = [theta1; theta2; theta3];
    thetas = [theta1_deg; theta2_deg; theta3_deg];
    
    T01=[    cos(theta1)     -sin(theta1)     0     0;
         sin(theta1)     cos(theta1)     0     0;
         0     0     1     0;
         0     0     0     1;
         ];
    
    T12=[   cos(theta2)   -sin(theta2) 0  0;
             0         0    1         0;
        -sin(theta2)   -cos(theta2)         0         0;
             0         0         0    1;
             ];
    
    T23=[     cos(theta3)   -sin(theta3)         0    la;
        sin(theta3)    cos(theta3)         0         0;
             0         0    1         0;
             0         0         0    1;
             ];
    
    T02 = T01*T12;
    T03 = T01*T12*T23;
    
    rm=T03*[0;0;lb;1];
    
    hold on
    plot3([0.1 -0.1 -0.1 0.1 0.1],[0.1 0.1 -0.1 -0.1 0.1],[0 0 0 0 0])  %base square
    plot3([0 T03(1,4) rm(1) r0(1)],[0 T03(2,4) rm(2) r0(2)],[0 T03(3,4) rm(3) r0(3)])
    plot3(r0(1),r0(2),r0(3),'o');
    axis equal
    %axis([-1 1 -1 1 -0.5 1.5])

    grid
    hold off
    view(-30,30)

    
    % Wait for key press to continue
    % pause;
end



%% Problem 2

T03=[ 0  -1  0  0;
      0   0  1  0;
     -1   0  0 -4;
      0   0  0  1;
    ];

a = [1; 0; 2; 1];
b = [-1; 0; 2; 1];
c = [1; 2; 0; 1];
d = [-1; 2; 0; 1];
e = [1; 0; 0; 1];
f = [-1; 0; 0; 1];

a_new = T03 * a;
b_new = T03 * b;
c_new = T03 * c;
d_new = T03 * d;
e_new = T03 * e;
f_new = T03 * f;

figure(4)

% Original points
original_points = [a, b, c, d, e, f];

% Transformed points
transformed_points = [a_new, b_new, c_new, d_new, e_new, f_new];

% Plot original points as dots in red
hold on;
plot3(original_points(1,:), original_points(2,:), original_points(3,:), 'ro', 'MarkerSize', 5);

% Plot transformed points as dots in blue
plot3(transformed_points(1,:), transformed_points(2,:), transformed_points(3,:), 'bo', 'MarkerSize', 5);

% Connect original points with lines in red
% line(original_points(1,:), original_points(2,:), original_points(3,:), 'Color', 'r');
line([original_points(1,1), original_points(1,3), original_points(1,5), original_points(1,1)], ...
    [original_points(2,1), original_points(2,3), original_points(2,5), original_points(2,1)], ...
    [original_points(3,1), original_points(3,3), original_points(3,5), original_points(3,1)], 'Color', 'r');

line([original_points(1,2), original_points(1,4), original_points(1,6), original_points(1,2)], ...
    [original_points(2,2), original_points(2,4), original_points(2,6), original_points(2,2)], ...
    [original_points(3,2), original_points(3,4), original_points(3,6), original_points(3,2)], 'Color', 'r');

line([original_points(1,1), original_points(1,2)], ...
    [original_points(2,1), original_points(2,2)], ...
    [original_points(3,1), original_points(3,2)], 'Color', 'r');

line([original_points(1,3), original_points(1,4)], ...
    [original_points(2,3), original_points(2,4)], ...
    [original_points(3,3), original_points(3,4)], 'Color', 'r');

line([original_points(1,5), original_points(1,6)], ...
    [original_points(2,5), original_points(2,6)], ...
    [original_points(3,5), original_points(3,6)], 'Color', 'r');

% Connect transformed points with lines in blue
line([transformed_points(1,1), transformed_points(1,3), transformed_points(1,5), transformed_points(1,1)], ...
    [transformed_points(2,1), transformed_points(2,3), transformed_points(2,5), transformed_points(2,1)], ...
    [transformed_points(3,1), transformed_points(3,3), transformed_points(3,5), transformed_points(3,1)], 'Color', 'b');

line([transformed_points(1,2), transformed_points(1,4), transformed_points(1,6), transformed_points(1,2)], ...
    [transformed_points(2,2), transformed_points(2,4), transformed_points(2,6), transformed_points(2,2)], ...
    [transformed_points(3,2), transformed_points(3,4), transformed_points(3,6), transformed_points(3,2)], 'Color', 'b');

line([transformed_points(1,1), transformed_points(1,2)], ...
    [transformed_points(2,1), transformed_points(2,2)], ...
    [transformed_points(3,1), transformed_points(3,2)], 'Color', 'b');

line([transformed_points(1,3), transformed_points(1,4)], ...
    [transformed_points(2,3), transformed_points(2,4)], ...
    [transformed_points(3,3), transformed_points(3,4)], 'Color', 'b');

line([transformed_points(1,5), transformed_points(1,6)], ...
    [transformed_points(2,5), transformed_points(2,6)], ...
    [transformed_points(3,5), transformed_points(3,6)], 'Color', 'b');


% Label points
labels = {'a', 'b', 'c', 'd', 'e', 'f'};
text(original_points(1,:), original_points(2,:), original_points(3,:), labels, ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'r');
text(transformed_points(1,:), transformed_points(2,:), transformed_points(3,:), labels, ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'Color', 'b');

% Add legend
legend('Frame 0: End-effector Points', 'Frame 3: End-effector Points', 'Location', 'best');

% Customize plot
title('End Effector Rotations and Translation');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;
axis equal;
hold off;
view(-30,30)
