function DR_result = SensorComputation
% this routine determines the trajectory of an object using dead reckoning
% navigation.

% Define constants
Define_Constants

% load data
dead_reckoning = csvread('Data/Dead_reckoning.csv');
[n,~] = size(dead_reckoning); % n is the number of epoch

% result
DR_result = zeros(n,6);
DR_result(:,1) = dead_reckoning(:,1); % store time in s

% let's use GNSScomputation to initialize the position
GNSSmeasurement = GNSScomputation; 

% the rear wheels are the driving wheels, so we can assume that the average
% of the rear wheel speeds correspond to the forward speed of the lawnmower
rear_wheel_L = dead_reckoning(:,4); % m/s
rear_wheel_R = dead_reckoning(:,5);
forward_speed = (rear_wheel_L + rear_wheel_R)/2; % m/s

% take height as calculated by GNSS
h = GNSSmeasurement(1,7);

time = dead_reckoning(:,1);

% first, we will compute the heading by correcting the gyroscope
% measurement with the compass measurement

%% Compute heading by combining gyrosope-derived heading and magnetic heading
% In this section, We will use a Gyro-Magnetometer Kalman Filter States to compute heading
dead_reckoning = csvread('Data/Dead_reckoning.csv');
gyroscope_heading_rate = dead_reckoning(:,6);
compassHeading = dead_reckoning(:,7)*deg_to_rad;

% Compute heading from gyroscope
gyroscopeHeading = zeros(n,1);
gyroscopeHeading(1) = gyroscope_heading_rate(1)*0.5;
for i=2:n
    gyroscopeHeading(i) = gyroscopeHeading(i-1)+0.5*gyroscope_heading_rate(i);
end

% store corrected heading
heading = zeros(n,1);
heading(1) = gyroscopeHeading(1); % initialize

% Initialize 2 states Kalman filter vector
h_est = zeros(2,1); %

% Initiaize state estimation error covariance matrix
sigma_gyroBias = 1*deg_to_rad; % bias standar deviation of 1 degree per second
Ph_matrix = [10^-4 0;... % noise standar deviation of 10-4 rad/s
             0 sigma_gyroBias];

for i=2:n
    
    % step 1 : Compute transition matrix
    tau_s = 0.5; % propagation time
    Phi = [1 tau_s;...
           0 1];
       
    % step 2 : compute noise covariance matrix
    S_rg = 3*10^-6; % Gyro random noise PSD, rad2/s
    S_bgd = 3*10^-6; % gyro bias variation PSD in rad2
    
    Q = [S_rg*tau_s+(1/3)*S_bgd*tau_s^3 (1/2)*S_bgd*tau_s^2;...
         (1/2)*S_bgd*tau_s^2 S_bgd*tau_s];
     
    % step 3 : propagate state estimates
    h_minus = Phi*h_est;
    
    % Step 4 : propagate error covariance matrix
    Ph_minus = Phi*Ph_matrix*Phi' + Q;
    
    % Step 5 : Compute measurement matrix
    H = [-1 0];
    
    % Step 6 : Measurement noise covariance
    R = 4*deg_to_rad; %magnetic heading noise variance
    
    % Step 7 : Kalman gain matrix
    K = Ph_minus*H'/(H*Ph_minus*H' + R);
    
    % Step 8 : Formulate measurement innovation vector
    dz = [compassHeading(i) - gyroscopeHeading(i)] -H*h_minus;
    
     
    % Step 9 : update states estimates and error covariance matrix
    h_est = h_minus + K*dz;
    Ph_matrix = (eye(2) - K*H)*Ph_minus;
    
    % store corrected heading
    heading(i) = gyroscopeHeading(i) - h_est(1);
    
end

% convert heading into degree
heading = heading*rad_to_deg;
% keep values between -180 and 180 degree
for i=1:n
   if heading(i)<-180 
       heading(i) = heading(i) + 360;
   elseif heading(i)>180
       heading(i) = heading(i) - 360;
   end
end

% convert back to rad
heading = heading*deg_to_rad;


%% Compute positions and velocity using wheel sensor and corrected heading
% Initialize geodetic latitude and longitude with GNSS measurement at first epoch
DR_result(1,2:3) = GNSSmeasurement(1,2:3)*deg_to_rad; % in rad

% Initialize velocity
v0 = forward_speed(1);
DR_result(1,4:5) = [v0*cos(heading(1)) v0*sin(heading(1))];

% compute new position for each epoch
for i=2:n
    
    % compute average velocity from epoch i-1 to epoch i
    v_N_k = (1/2)*(cos(heading(i)) + cos(heading(i-1)))*forward_speed(i);
    v_E_k = (1/2)*(sin(heading(i)) + sin(heading(i-1)))*forward_speed(i);
    
    % meridian/transverse radius of the curvature
    [R_N,R_E]= Radii_of_curvature(DR_result(i-1,2));
    
    DR_result(i,2) = DR_result(i-1,2) + (v_N_k*(time(i) - time(i-1)))/(R_N + h);
    DR_result(i,3) = DR_result(i-1,3) + (v_E_k*(time(i) - time(i-1)))/((R_E + h)*cos(DR_result(i,2)));
    
    % instantaneous velocity
    DR_result(i,4) = 2*v_N_k - DR_result(i-1,4);
    DR_result(i,5) = 2*v_E_k - DR_result(i-1,5);
    
end

% convert latitude/longitude from rad to degree
DR_result(:,2:3) = DR_result(:,2:3)*rad_to_deg;
% convert heading from rad to deg
DR_result(:,6) = heading*rad_to_deg;

end