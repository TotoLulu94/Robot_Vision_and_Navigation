% This routine determine the position of the object at all of the epochs
% using GNSS and Kalman Filter
function GNSSresults = GNSScomputation
Define_Constants

% import data for determining positions and Velocity
pseudo_ranges = csvread('Data/Pseudo_ranges.csv');
pseudo_range_rates = csvread('Data/Pseudo_range_rates.csv');

% n is the number of epoch, m the number of satellites
[n,~] = size(pseudo_ranges);

% Array to store result
GNSSresults = zeros(n-1, 7);
% 1st column is the time in seconds
GNSSresults(:,1) = pseudo_ranges(2:end,1);
% 2nd column contains Geodetic latitude in degrees
% 3rd column contains Geodetic longitude in degrees
% 4th column contains North Velocity in m/s
% 5th column contains East velocity in m/s
% 6th column contains heading in degrees
% 7th column contains height

%% Initialise the Kalman filter state vector estimate and error covariance matrix
[x_est,P_matrix] = Initialise_GNSS(pseudo_ranges, pseudo_range_rates);

% x_est correspond to the initial positions (Which will be the initialized Kalman filter state vector estimate)
% P_matrix is the initial error covariance matrix
% positions and velocities are the positions/Velocities computed without using Kalman Filter

%% Apply Kalman Filter to initial state vector/error covariance matrix

S_a_cphi = 0.01;% clock phase PSD
S_a_cf = 0.04;%clock frequency PSD
sigma_p = 2; % error standart deviation of pseudo range measurements in m
sigma_r = 0.02; % error standart deviation of pseudo range rate measurement in m/s

result = Kalman_Filter(x_est, P_matrix, pseudo_ranges, pseudo_range_rates,S_a_cphi, S_a_cf, sigma_p, sigma_r);

GNSSresults(:,2:3) = result(:,2:3);
GNSSresults(:, 4:5) = result(:,5:6);
GNSSresults(:,7) = result(:,4);
end

%% This subroutine is used to initialised the Kalman Filter
function [x_est,P_matrix] = Initialise_GNSS(pseudo_ranges, pseudo_range_rates)
    % first run Define_Constants.m
    Define_Constants

    % Extract data
    pseudo_ranges_data = pseudo_ranges(2:end, 2:end);
    pseudo_range_rates_data = pseudo_range_rates(2:end,2:end);
    [n,m] = size(pseudo_range_rates_data); % m number au satellite, n number of epoch
    

    clock = 1;
    clock_drift = 0;
    
    % Initialize user position at the center of the Earth and velocity at 0
    r_eb_e = zeros(3,1);
    v_eb_e = zeros(3,1);

    % Store satellite position and velocity for current epoch

    % store the cartesian ECEF positions
    ECEF_positions = zeros(m,4) ;
    ECEF_velocities = zeros(m,4);
    % first column contains satellite number
    ECEF_positions(:,1) = pseudo_ranges(1,2:end);
    ECEF_velocities(:,1) = pseudo_range_rates(1,2:end);

    % Fill the array
    for i=1:m
       [sat_r_es_e,sat_v_es_e] = Satellite_position_and_velocity(0,ECEF_positions(i,1));
       ECEF_positions(i,2:end) = sat_r_es_e;
       ECEF_velocities(i,2:end) = sat_v_es_e;
    end

    
    distance = 10; %(random number > 10 so that it doesn't stop at 1st epoch)

    % if the distance between 2 consecutives iterations > 1 m, then
    % continue (We consider that we have convergence when distance < 1)
    while distance > 0.1
        %% compute ranges and range rates from the user to each satellite line of sight unit vector (1x3) for each satellite

        % for storing ranges and range rates
        ranges = zeros(m,2);
        range_rates = zeros(m,2);
        % 1st column contains satelites number
        ranges(:,1) = pseudo_ranges(1,2:end);
        range_rates(:,1) = pseudo_range_rates(1,2:end);

        lineOfSightArray = zeros(m,4);
        % column 1 = satellite number
        % column 2 to 4 = unit vector
        lineOfSightArray(:,1) = pseudo_ranges(1,2:end)';

        % For each satellite, compute its range from user
        for i=1:m
            % compute range
            %Cartesian ECEF position of the satellite (3x1 vector)
            r_e_ej = ECEF_positions(i,2:end)';

            %Predicted Cartesian ECEF user  (3x1 vector)
            r_e_ea = r_eb_e;

            % Initialize Sagnac compensation matrix as Identity matrix
            C_I_e = eye(3);

            % Compute range
            r_aj = sqrt((C_I_e*r_e_ej - r_e_ea)'*(C_I_e*r_e_ej - r_e_ea));

            % recompute Sagnac compensation matrix using range
            C_I_e(2,1) = -omega_ie*r_aj/c ;
            C_I_e(1,2) = omega_ie*r_aj/c ;

            % recompute range
            r_aj = sqrt((C_I_e*r_e_ej - r_e_ea)'*(C_I_e*r_e_ej - r_e_ea));

            % Store range of the satellite
            ranges(i,2) = r_aj;

            % Compute light of sight unit vector
            u_e_aj = (C_I_e*r_e_ej - r_e_ea)/r_aj;
            lineOfSightArray(i,2:4) = u_e_aj;

            % Compute range rates
            % Cartesian ECEF velocity of the satellite
            v_e_ej = ECEF_velocities(i,2:end);

            % Cartesian ECEF velocity of the user
            v_e_ea = v_eb_e;

            r_aj_bis = u_e_aj'*(C_I_e*(v_e_ej' + Omega_ie*r_e_ej) - (v_e_ea + Omega_ie*r_e_ea));

            % Store range of the satellite
            range_rates(i,2) = r_aj_bis;
        end

        %% predicted state vector, measerement innovation vector and measurement matrix

        % predicted receiver clock offset (initialisation at 0)
        drho_a_c = clock;
        drho_a_c_bis = clock_drift;
        % predicted state vector
        X = [r_e_ea; drho_a_c];
        X_bis = [v_e_ea; drho_a_c_bis];

        % measurement innovation vector
        dz = zeros(m,1);
        dz_bis = zeros(m,1);

        % measurement matrix
        H_e_G = ones(m,4);

        for i=1:m
            % measured pseudo-range and pseudo range rates from satellite i to the user antenna
            rho_a = pseudo_ranges_data(1,i);
            rho_a_bis = pseudo_range_rates_data(1,i);

            % range r_aj and range rates r_aj_bis
            r_aj = ranges(i,2);
            r_aj_bis = range_rates(i,2);

            % compute
            dz(i,1) = rho_a - r_aj - drho_a_c;
            dz_bis(i,1) = rho_a_bis - r_aj_bis - drho_a_c_bis;

            %fill measurement matrix
            u_e_aj = lineOfSightArray(i,2:4);
            H_e_G(i,1:3) = -u_e_aj;

        end

        %% Compute the position and clock receiver clock offset using unweighted least-squares
        X_new = X + (H_e_G'*H_e_G)\H_e_G'*dz;
        X_new_bis = X_bis + (H_e_G'*H_e_G)\H_e_G'*dz_bis;

        distance = norm(X_new(1:3) - r_eb_e);

        % update
        r_eb_e = X_new(1:3);
        clock = X_new(end);
        v_eb_e = X_new_bis(1:3);
        clock_drift = X_new_bis(end);
    end

    
    % Initialise state estimates (at 1st epoch)
    x_est = [ r_eb_e; v_eb_e;...
              clock; clock_drift];

    % Initialise error covariance matrix
    P_matrix = [ 10^2*eye(3) zeros(3,5);...
             zeros(3) 0.05^2*eye(3) zeros(3,2);...
             zeros(1,6) 10^2 0;...
             zeros(1,7) 0.05^2];

end

%% This subroutine apply the Kalman Filter to all epochs to determine positions and velocity
function result = Kalman_Filter(x_est, P_matrix, pseudo_ranges, pseudo_range_rates, S_a_cphi, S_a_cf, sigma_p, sigma_r)

    % define constants
    Define_Constants;

    % n is the number of epoch, m is the number of satellites
    [n,m] = size(pseudo_ranges(2:end, 2:end));

    % Store position and velocity
    result = zeros(n,7);
    result(:,1) = pseudo_ranges(2:end,1);

    % for each epoch
    for epoch =1:n
        %% Step 1 : Compute the Transition matrix

        % propagation interval
        tau_s = 0.5;

        Phi = [eye(3) tau_s*eye(3) zeros(3,1) zeros(3,1); ...
               zeros(3) eye(3) zeros(3,1) zeros(3,1);...
               zeros(1,3) zeros(1,3) 1 tau_s;...
               zeros(1,3) zeros(1,3) 0 1];

        %% Step 2 : Compute system noise covariance matrix

        % Acceleration power spectral density
        S_a = 5; %(in m2/s3)

        Q = [(1/3)*S_a*(tau_s^3)*eye(3) (1/2)*S_a*(tau_s^2)*eye(3) zeros(3,1) zeros(3,1);...
             (1/2)*S_a*(tau_s^2)*eye(3) S_a*tau_s*eye(3) zeros(3,1) zeros(3,1);...
             zeros(1,3) zeros(1,3) S_a_cphi*tau_s+(1/3)*S_a_cf*(tau_s^3) (1/2)*S_a_cf*(tau_s^2);...
             zeros(1,3) zeros(1,3) (1/2)*S_a_cf*(tau_s^2) S_a_cf*tau_s];

        %% Step 3 : Propagate the state estimates
        X_minus = Phi*x_est;

        %% Step 4 : Propagate error covariance matrix
        P_minus = Phi*P_matrix*Phi' + Q;

        %% Compute also satellites ranges, range rates and light of sight unit vector at current epoch (needed for further steps).

        ranges = zeros(m,2);
        range_rates = zeros(m,2);
        line_of_sight = zeros(m,4);

        % 1st column is the satellite number
        ranges(:,1) = pseudo_ranges(1,2:end)';
        range_rates(:,1) = pseudo_ranges(1,2:end)';
        line_of_sight(:,1) = pseudo_ranges(1,2:end)';

        % for each satellites 
        for i=1:m

            % Compute satellite posision and velocity at current epoch
            time = pseudo_ranges(epoch+1,1);
            sat_number = pseudo_ranges(1,i+1);
            [sat_r_es_e,sat_v_es_e] = Satellite_position_and_velocity(time,sat_number);

            % Cartesian ECEF position of satellite
            r_e_ej = sat_r_es_e';

            % Cartesian ECEF velocity of satellite
            v_e_ej = sat_v_es_e';

            % Cartesian ECEF position of the user
            r_e_ea = X_minus(1:3);

             % Cartesian ECEF velocity of the user
            v_e_ea = X_minus(4:6);

            %Initialize Sagnac effect compensation matrix as Identity
            C_I_e = eye(3);

            % Compute 1st version of range
            r_aj = sqrt((C_I_e*r_e_ej - r_e_ea)'*(C_I_e*r_e_ej - r_e_ea));

            % Recompute Sagnac effect compensation matrix
            C_I_e(1,2) = omega_ie*r_aj/c;
            C_I_e(2,1) = -omega_ie*r_aj/c;

            % Recompute range
            r_aj = sqrt((C_I_e*r_e_ej - r_e_ea)'*(C_I_e*r_e_ej - r_e_ea));

            % Compute line of sight vector
            u_e_ea = (C_I_e*r_e_ej - r_e_ea)/r_aj;

            % compute range rates
            r_aj_bis = u_e_ea'*(C_I_e*(v_e_ej + Omega_ie*r_e_ej) - (v_e_ea + Omega_ie*r_e_ea));

            % Store results
            ranges(i,2) = r_aj;
            range_rates(i,2) = r_aj_bis;
            line_of_sight(i,2:end) = u_e_ea;
        end

        %% Step 5 : Compute measurement matrix

        H = [ -line_of_sight(:,2:end) zeros(m,3) ones(m,1) zeros(m,1);...
               zeros(m,3) -line_of_sight(:,2:end) zeros(m,1) ones(m,1)];

        %% Step 6 : Compute measurement noise covariance matrix
        
        [l,~] = size(H*P_minus*H');

        R = [sigma_p^2*eye(l/2) zeros(l/2);...
             zeros(l/2) sigma_r^2*eye(l/2)];

         %% Step 7 : Compute Kalman gain matrix

         K = P_minus*H'/(H*P_minus*H' + R);

         %% Step 8 : Compute measurement innovation vector

         % measured pseudo ranges from the satellites
         rho_j_a = pseudo_ranges(epoch+1,2:end)';
         % measured pseudo range rates
         rho_j_a_bis = pseudo_range_rates(epoch+1,2:end)';
         % propagated receiver clock offset estimate
         drho_a_c = X_minus(7);
         % propagated receiver clock drift estimate
         drho_a_c_bis = X_minus(8);

         dz = [rho_j_a - ranges(:,2) - drho_a_c;...
               rho_j_a_bis - range_rates(:,2) - drho_a_c_bis];

         %% Step 9 : Update the state estimate
         X_new = X_minus + K*dz;

         %% Step 10 : Update error covariance matrix
         P_new = (eye(size(K*H)) - K*H)*P_minus;

         %% Store results
         [L_b,lambda_b,h_b,v_eb_n] = pv_ECEF_to_NED(X_new(1:3),X_new(4:6));

         % convert latitude and longitude in degree
         result(epoch,2:3) = rad_to_deg*[L_b lambda_b];
         result(epoch,4) = h_b;
         result(epoch,5:7) = v_eb_n;

         % prepare data for the next epoch
         x_est = X_new;
         P_matrix = P_new;
    end
end


