classdef GPSLocalizationSystem < answers.gtsam.OdometryOnlyLocalizationSystem
   
    % This class extends the odometry system to process GPS measurements
    % when they are available.
    
    methods(Access = public)
        
        function this = GPSLocalizationSystem()
            this = this@answers.gtsam.OdometryOnlyLocalizationSystem();
        end
        
    end
    
    methods(Access = protected)        
        
        % Implement the GPS measurement.
        function handleGPSEvent(this, event)
            
            % You will need to create a suitable factor and noise model
            % insert it into the graph. You will need:
            
            % As for kalman update, we will keep the orientation
            % information obtained by odometry
            % Specify the measurement noise model, e.g.,
            gpsCovariance = zeros(3);
            gpsCovariance(1:2, 1:2) = event.covariance;
            gpsCovariance(3,3) = this.uCov(2,2); % orientation obtained by odometry
            gpsNoise = gtsam.noiseModel.Gaussian.Covariance(gpsCovariance);
            
            % get GPS measurements
            z = zeros(3,1);
            z(1:2) = event.data;
            z(3) = this.currentVehiclePose.theta; % orientation obtained by odometry
            % Construct the suitable factor
            currentPoseKey = this.currentVehiclePoseKey;
            gpsObservationFactor = gtsam.PriorFactorPose2(currentPoseKey,gtsam.Pose2(z(1), z(2), z(3)), gpsNoise);
            
            % Insert it into the graph. Use this method:
            this.addNewFactor(gpsObservationFactor);
        end
    end    
end