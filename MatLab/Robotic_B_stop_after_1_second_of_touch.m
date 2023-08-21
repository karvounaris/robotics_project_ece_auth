% Aristotle University of Thessaloniki
% Robotics
% Panagiotis Karvounaris
% AEM 10193

%% Section Setup

% Set simulation time period and time step
t_final = 10;
step = 0.002;
t = 0:step:t_final;

% Create the robot
robot = mdl_ur10e();

% Create object for the ball movement simulation
wspace = Wspace();

% Set joint angles for starting position
q_position = [-0.140 -1.556 -1.359 1.425 -1.053 -1.732];

% Initialize a matrix to store ball's position, velocity and angular
% velocity
p_cb_matrix = zeros(3, length(t));
v_cb_matrix = zeros(3, length(t));
w_cb_matrix = zeros(3, length(t));

% Initialize parameters
p_oc = [0.4; 0; 0.2];
p_be = [0; 0; 0.45];
p_be_matrix = zeros(3, length(t));
R_be = [-1 0 0;
        0 1 0;
        0 0 -1];
theta_i = -0.362; % start value of theta, helps to calculate g_d
R_oc = eye(3);
K = 100; % Gain
q_dot = zeros(6,1); % velocity of the joints in current control cycle
q_dot_previous = zeros(6,1); % velocity of the joints in previous control cycle
q_double_dot = zeros(6,1); % acceleration of the joints in current control cycle
temp = 0; % parameter that helps with the 1 sec time keeping at the end
time = 0; % parameter that helps with the 1 sec time keeping at the end
velocity_hybrid = zeros(6); % help to calculate the desired hybrid velocity
velocity_hybrid_d = zeros(6); % desired hybrid velocity

% This parameter controls how fast does the arm approach the ball
decrease_step = 0.0002;

% Initialize storage matrix for plotting reasons
% Robot's position
p_matrix = zeros(3, length(t)); % robot's position
p_d_matrix = zeros(3, length(t)); % desired robot's position
error_p_matrix = zeros(3, length(t)); % robot's position error

% % Robot's quaternion
% quat_matrix = zeros(4, length(t)); % robot's quaternion
% quat_d_matrix = zeros(4, length(t)); % desired robot's quaternion
% error_o_matrix = zeros(3, length(t)); % robot's orientation error

% Robot's orientation
theta_e_matrix = zeros(length(t));
k_matrix = zeros(3, length(t));
error_o_matrix = zeros(3, length(t)); % robot's orientation error

% Joint's position
q_position_matrix = zeros(6, length(t)); % joint's position

% Joint's angular velocity
q_dot_matrix = zeros(6, length(t)); % joint's angular velocity

% Joint's angular acceleration
q_double_dot_matrix = zeros(6, length(t)); % joint's angular acceleration

% Angle error
theta_i_matrix = zeros(length(t));

% Robot's hybrid velocity
velocity_v_matrix = zeros(3,length(t)); % robot velocity
angular_velocity_w_matrix = zeros(3,length(t)); % robot angular velocity

%% Section Simulation

% Run the simulation
for i=1:1:length(t)
    % Compute the forward kinematics and extract the position and orientation
    T = robot.fkine(q_position);
    p = T.t; % robot_position
    p_matrix(:,i) = p;
    R_e = T.R; % robot_orientation as rotation matrix
%     quat = rotm2quat(R_e); % robot_orientation as quaternion
%     quat = quat / norm(quat);
%     quat_matrix(:, i) = quat;
    
    % Calculate position, velocity and orientation from ball to camera
    [p_cb, v_cb, w_cb] = wspace.sim_ball(step);
    p_cb_matrix(:,i) = p_cb;
    v_cb_matrix(:,i) = v_cb;
    w_cb_matrix(:,i) = w_cb;
    
    % Calculate the homogenous transformation g_d
    theta_i = theta_i + w_cb(1) * step; % use w_cd to calculate theta
    theta_i_matrix(i) = theta_i;
    R_cb = [1 0 0;
            0 cos(theta_i) -sin(theta_i);
            0 sin(theta_i) cos(theta_i)]; % use theta to calculate R_cd
    R_d = R_oc*R_cb*R_be;
    p_d = R_oc*R_cb*p_be+R_oc*p_cb+p_oc;
    p_d_matrix(:, i) = p_d;
    g_d = [R_d(1,1) R_d(1,2) R_d(1,3) p_d(1);
           R_d(2,1) R_d(2,2) R_d(2,3) p_d(2);
           R_d(3,1) R_d(3,2) R_d(3,3) p_d(3);
           0 0 0 1];
    
    % Calculate the desired velocity and angular velocity from b to 0 
    p_oc_antisymmetric = [0 -p_oc(3) p_oc(2);
                          p_oc(3) 0 -p_oc(1);
                          -p_oc(2) p_oc(1) 0];
    help_par = p_oc_antisymmetric * R_oc;
    gamma_oc_matrix =[R_oc help_par;
                      zeros(3,3) R_oc];
    gamma_oc_matrix = inv(gamma_oc_matrix);
    velocity_hybrid = [v_cb; w_cb];
    velocity_hybrid_d = gamma_oc_matrix * velocity_hybrid;
    v_d = velocity_hybrid_d(1:3);
    w_d = velocity_hybrid_d(4:6);
   
%     % Calculate the desired unit quaternion
%     quat_d = rotm2quat(R_d);
%     quat_d_matrix(:, i) = quat_d;
    
    % Calculate errors
    error_p = p - p_d;
    error_p_matrix(:, i) = error_p;
    
    R_ed = R_e * inv(R_d);
    theta_e = acos((trace(R_ed) - 1) / 2);
    k = (1/(2*sin(theta_e))).*[R_ed(3,2) - R_ed(2,3);
                             R_ed(1,3) - R_ed(3,1);
                             R_ed(2,1) - R_ed(1,2)];
    error_o = [theta_e .* k(1);
               theta_e .* k(2);
               theta_e .* k(3)];
    error_o_matrix(:, i) = error_o;
           
%     quat_d_inverse = quatinv(quat_d);
%     quat_d_inverse = quat_d_inverse / norm(quat_d_inverse );
%     quat_error = quatmultiply(quat, quat_d_inverse);
%     quat_error = quat_error / norm(quat_error);
%     error_o = [quat_error(2);
%                quat_error(3);
%                quat_error(4)];
%     error_o_matrix(:, i) = error_o;
    
    % Calculate the new velocity and angular velocity of the robot
    u = [v_d; w_d] - K .* [error_p; error_o];
    velocity_v_matrix(:, i) = u(1:3);
    angular_velocity_w_matrix(:, i) = u(4:6);
    
    % Calculate the velocity of the joints
    inverse_jacobian = inv(robot.jacob0(q_position));
    q_dot = inverse_jacobian * u;
    
    for j=1:1:6
        % Use the saturation function to check the limits of the joint's velocity
        if (j == 1) || (j == 2)
            q_dot(j) = my_saturation(q_dot(j), -(2*pi)/3, (2*pi)/3);
        else
            q_dot(j) = my_saturation(q_dot(j), -pi, pi);
        end
        
        % Calculate the acceleration of every joint
        q_double_dot(j) = (q_dot(j) - q_dot_previous(j)) / step;
        
        % Use the saturation function to check the limits of the joint's
        % acceleration and change the velocity if needed.
        q_double_dot(j) = my_saturation(q_double_dot(j), -250, 250);
        if (q_double_dot(j) == -250) || (q_double_dot(j) == -250)
            q_dot(j) = q_dot_previous(j) + q_double_dot(j) * step;
        end
        
        % Find the new value of the joint's angles
        q_position(j) = q_position(j) + q_dot(j) * step;
        
        % Update the q_dot_previous value
        q_dot_previous(j) = q_dot(j);
        
    end
    
    % Store data
    q_position_matrix(:, i) = q_position;
    q_dot_matrix (:, i) = q_dot;
    q_double_dot_matrix (:, i) = q_double_dot;
    
    % Decrease the value of desired p_be
    if p_be(3) >= (0.06 + decrease_step)
        p_be(3) = p_be(3) - decrease_step;
        p_be_matrix(:,i) = p_be; 
    end
    
    % Hold the robot for one second on the ball to catch it
    if p_be(3) <= (0.06 + decrease_step)
        if abs(error_p(1)) <= 0.0001 && abs(error_p(2)) <= 0.0001 && abs(error_p(3)) <= 0.0001
            if abs(error_o(1)) <= 0.0001 && abs(error_o(2)) <= 0.0001 && abs(error_o(3)) <= 0.0001
                if temp == 0
                    time = t(i);
                    temp = 1;
                elseif (t(i) - time) >= 1
                    break
                end
            end
        end
    end
end

%% Section plot
 
view = [90,0];
wspace.visualize(robot, q_position_matrix, view);


% P_BE plot
figure()
tiledlayout(3,1)
nexttile
plot(t, error_p_matrix(1,:))
title('P_{BE} x axis')
xlabel('Time (s)')
ylabel('Error (m)')
nexttile
plot(t, error_p_matrix(2,:))
title('P_{BE} y axis')
xlabel('Time (s)')
ylabel('Error (m)')
nexttile
plot(t, error_p_matrix(3,:))
title('P_{BE} z axis')
xlabel('Time (s)')
ylabel('Error (m)')


% R_BE plot
figure()
tiledlayout(3,1)
nexttile
plot(t, error_o_matrix(1,:))
title('R_{BE} x axis')
xlabel('Time (s)')
ylabel('Error (rad)')
nexttile
plot(t, error_o_matrix(2,:))
title('R_{BE} y axis')
xlabel('Time (s)')
ylabel('Error (rad)')
nexttile
plot(t, error_o_matrix(3,:))
title('R_{BE} z axis')
xlabel('Time (s)')
ylabel('Error (rad)')


% Joints 1 to 3 q plot
figure()
tiledlayout(3,1)
nexttile
plot(t, q_position_matrix(1,:))
title('Position of first joint')
xlabel('Time (s)')
ylabel('Position (rad)')
nexttile
plot(t, q_position_matrix(2,:))
title('Position of second joint')
xlabel('Time (s)')
ylabel('Position (rad)')
nexttile
plot(t, q_position_matrix(3,:))
title('Position of third joint')
xlabel('Time (s)')
ylabel('Position (rad)')


% Joints 1 to 3 q plot
figure()
tiledlayout(3,1)
nexttile
plot(t, q_position_matrix(4,:))
title('Position of forth joint')
xlabel('Time (s)')
ylabel('Position (rad)')
nexttile
plot(t, q_position_matrix(5,:))
title('Position of fifth joint')
xlabel('Time (s)')
ylabel('Position (rad)')
nexttile
plot(t, q_position_matrix(6,:))
title('Position of sixth joint')
xlabel('Time (s)')
ylabel('Position (rad)')


% Joints 1 to 3 q dot plot
figure()
tiledlayout(3,1)
nexttile
plot(t, q_dot_matrix(1,:))
title('Angular velocity of first joint')
xlabel('Time (s)')
ylabel('q dot (rad/s)')
nexttile
plot(t, q_dot_matrix(2,:))
title('Angular velocity of second joint')
xlabel('Time (s)')
ylabel('q dot (rad/s)')
nexttile
plot(t, q_dot_matrix(3,:))
title('Angular velocity of third joint')
xlabel('Time (s)')
ylabel('q dot (rad/s)')


% Joints 1 to 3 q dot plot
figure()
tiledlayout(3,1)
nexttile
plot(t, q_dot_matrix(4,:))
title('Angular velocity of forth joint')
xlabel('Time (s)')
ylabel('q dot (rad/s)')
nexttile
plot(t, q_dot_matrix(5,:))
title('Angular velocity of fifth joint')
xlabel('Time (s)')
ylabel('q dot (rad/s)')
nexttile
plot(t, q_dot_matrix(6,:))
title('Angular velocity of sixth joint')
xlabel('Time (s)')
ylabel('q dot (rad/s)')


% Joints 1 to 3 q double dot plot
figure()
tiledlayout(3,1)
nexttile
plot(t, q_double_dot_matrix(1,:))
title('Angular acceleration of first joint')
xlabel('Time (s)')
ylabel('q double dot (rad/s^2)')
nexttile
plot(t, q_double_dot_matrix(2,:))
title('Angular acceleration of second joint')
xlabel('Time (s)')
ylabel('q double dot (rad/s^2)')
nexttile
plot(t, q_double_dot_matrix(3,:))
title('Angular acceleration of third joint')
xlabel('Time (s)')
ylabel('q double dot (rad/s^2)')


% Joints 4 to 6 q double dot plot
figure()
tiledlayout(3,1)
nexttile
plot(t, q_double_dot_matrix(4,:))
title('Angular acceleration of fourth joint')
xlabel('Time (s)')
ylabel('q double dot (rad/s^2)')
nexttile
plot(t, q_double_dot_matrix(5,:))
title('Angular acceleration of fifth joint')
xlabel('Time (s)')
ylabel('q double dot (rad/s^2)')
nexttile
plot(t, q_double_dot_matrix(6,:))
title('Angular acceleration of sixth joint')
xlabel('Time (s)')
ylabel('q double dot (rad/s^2)')

