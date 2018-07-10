classdef GPSLocalizationSystem < answers.kalman.OdometryOnlyLocalizationSystem
   
    % This class extends the odometry system to process GPS measurements
    % when they are available.
    
    methods(Access = public)
        
        function this = GPSLocalizationSystem()
            this = this@answers.kalman.OdometryOnlyLocalizationSystem();
        end
        
    end
    
    methods(Access = protected)        
        
        % Implement the GPS measurement.
        function handleGPSEvent(this, event)
            
            % sensor handler function
            % H.h sensor function
            h = @(x) x;

            % J its jacobian
            H = @(x) eye(size(x,2));
            
            % You will need to write a Kalman filter update            
            
            % The variables this.xPred, this.PPred contain the predicted
            % mean and covariance.
            xPred = this.xPred;
            PPred = this.PPred;

            % extracting GPS measurement. Since we have no information
            % about the orientation of the vehicle, we'll keep the
            % orientation predicted by Odometry
            z = [0;0;0];
            z(1:2) = event.data;
            z(3) = xPred(3) ;
            
            % event.covariance give the measurement error 
            R = zeros(3);
            R(1:2,1:2) = event.covariance; % error on the coordinates
            R(3,3) = 0.0010 ; % covariance from odometry.
            
            % get sensor function jacobian
            J = H(xPred);
            
            % compute near-optimal kalman gain
            G = PPred*J'/(J*PPred*J' + R);
            
            % We then update the position mean and its covariance matrix :
            
            xEst = xPred + G*(z - h(xPred));
            PEst = (eye(3) - G*J)*PPred;
                      
            % Your update should produce a new estimate this.xEst and
            % this.PEst
            this.xEst = xEst;
            this.PEst = PEst;
            
            
            
            
            
            
            
        end
    end    
end