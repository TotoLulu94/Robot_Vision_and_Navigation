% This routine compute an integrated horizontal DR/GNQQ navigation solution
% using Kalman filtering

% Define constants
Define_Constants

% Compute GNSS solution
GNSSsolutions = GNSScomputation;
[n,~] = size(GNSSsolutions); % n is the number of epoch

% Compute DR solution
DRsolution = SensorComputation;

% separate the data
time = GNSSsolutions(:,1);

GNSSlatitude = GNSSsolutions(:,2)*deg_to_rad;
GNSSlongitude = GNSSsolutions(:,3)*deg_to_rad;
GNSSheight = GNSSsolutions(:,7); 
GNSSnorth_velocity = GNSSsolutions(:,4);
GNSSeast_velocity = GNSSsolutions(:,5);

DRlatitude = DRsolution(:,2)*deg_to_rad;
DRlongitude = DRsolution(:,3)*deg_to_rad;
DRnorth_velocity = DRsolution(:,4);
DReast_velocity = DRsolution(:,5);

% initialize latitude with DR measurement
latitude = DRlatitude;

% Since it's a lawnmower, we assume that height is 0
height = zeros(n,1);

% Store result
correctedResults = zeros(n, 6);
correctedResults(:,1) = time;
correctedResults(1,:) = DRsolution(1,:); % initalization of latitude, longitude, velocity, heading and height

% Initialize 4-state Kalman filter vector
x_est = zeros(4,1);

% Initialise state estimation error covariance matrix
P_matrix = zeros(4);
sigma_v = 0.02;
sigma_r = 10;
[R_N,R_E]= Radii_of_curvature(latitude(1));
P_matrix(1,1) = sigma_v^2;
P_matrix(2,2) = sigma_v^2;
P_matrix(3,3) = sigma_r^2/(R_N + height(1))^2;
P_matrix(4,4) = sigma_r^2/(R_E + height(1))^2*cos(latitude(1))^2;
 
for i=2:n
    %% Step 1 : Compute transition matrix
    tau_s = 0.5; % propagation time in s
    [R_N,R_E]= Radii_of_curvature(latitude(i-1));
    Phi = eye(4);
    Phi(3,1) = tau_s/(R_N + height(i-1));
    Phi(4,2) = tau_s/((R_E + height(i-1))*cos(latitude(i-1)));
    
    %% Step 2 : Compute system noise covriance matrix
    S_DR = 0.01; % DR PSD
    Q = [S_DR*tau_s 0 (1/2)*(S_DR*tau_s^2)/(R_N + height(i-1)) 0 ;...
         0 S_DR*tau_s 0 (1/2)*(S_DR*tau_s^2)/((R_E + height(i-1))*cos(latitude(i-1)));...
         (1/2)*(S_DR*tau_s^2)/(R_N + height(i-1)) 0 (1/3)*(S_DR*tau_s^3)/(R_N+height(i-1)) 0; ...
         0 (1/2)*(S_DR*tau_s^2)/((R_E + height(i-1))*cos(latitude(i-1))) 0 (1/3)*(S_DR*tau_s^3)/((R_E+height(i-1))*cos(latitude(i-1))^2)];
    
    %% Step 3 : Propagate state estimates and error covariance matrix
    x_minus = Phi*x_est;
    P_minus = Phi*P_matrix*Phi' + Q;
    
    %% Step 4 : Compute measurement matrix
    H = [0 0 -1 0;...
         0 0 0 -1;...
         -1 0 0 0;...
         0 -1 0 0];
     
    %% Step 5 : Compute measurement noise covariance matrix
    sigma_Gr = 10; % Gnss position error standard deviation
    sigma_Gv = 0.05; % GNSS velocity error standard deviation
    [R_N,R_E]= Radii_of_curvature(latitude(i));
    R = [ sigma_Gr^2/(R_N+height(i))^2 0 0 0;...
          0 sigma_Gr^2/((R_E + height(i))^2*cos(latitude(i))^2) 0 0;...
          0 0 sigma_Gv^2 0;...
          0 0 0 sigma_Gv^2];
      
     
    %% Step 6 : Computer Kalman gain matrix
    K = P_minus*H'/(H*P_minus*H' + R);
    
    %% Step 7 : Formulate measurement innovation vector
    dz = [GNSSlatitude(i) - DRlatitude(i);...
          GNSSlongitude(i) - DRlongitude(i);...
          GNSSnorth_velocity(i) - DRnorth_velocity(i);...
          GNSSeast_velocity(i) - DReast_velocity(i)];
    
      dz = dz - H*x_minus;
      
    %% Step 8 : Update state estimates
    x_est = x_minus + K*dz;
    
    %% step 9 : Update Error covariance matrix
    P_matrix = (eye(size(K*H))-K*H)*P_minus;
    
    % Correct DR solutions
    correctedResults(i,2) = DRlatitude(i) - x_est(3); % latitude
    correctedResults(i,3) = DRlongitude(i) - x_est(4); % longitude
    correctedResults(i,4) = DRnorth_velocity(i) - x_est(1); % north velocity
    correctedResults(i,5) = DReast_velocity(i) - x_est(2); % east velocity
    
end
     
% convert latitude and longitude from rad to degree
correctedResults(2:end,2:3) = correctedResults(2:end,2:3)*rad_to_deg;

% corrected heading computed in SensorComputation
correctedResults(:,6) = DRsolution(:,6);

% uncomment to save result into a csv file
% csvwrite('resultThomasLuo.csv',correctedResults)