classdef KalmanFilterLocalizationSystem < minislam.localization.VehicleLocalizationSystem
    
    % This is a very minimal Kalman filter; its purpose is to let me do
    % some debugging and comparison. It is not fully featured.
    
    properties(Access = protected)
        
        
        xEst;
        PEst;
        
        xPred;
        PPred;
    end
    
    methods(Access = public)
        
        function [x,P] = getCurrentMeanAndCovarianceEstimate(this, optimizeIfNeeded)
            x = gtsam.Pose2(this.xEst(1), this.xEst(2), this.xEst(3));
            P = this.PEst;
        end
        
        function [x,P] = getCurrentLandmarkEstimates(this, optimizeIfNeeded)
            x = zeros(2, 0);
            P = zeros(2, 2, 0);
        end
        
        function recommendation = recommendOptimization(this)
            recommendation = true;
        end
        
        function optimize(this)
            % Nothing to do
        end
    end
       
    methods(Access = protected)
    
        function advanceToEventTime(this, time)

            % Work out the time step length. This is the interval we
            % will predict over.
            dT = time - this.currentTime;

            % Nothing to do if it's really close to the last time or we
            % have no odometry
            if ((abs(dT) < 1e-3) || (isempty(this.u) == true))
                this.xPred = this.xEst;
                this.PPred = this.PEst;
                return
            end

            [this.xPred, Fd, Qd] = this.predictMeanJacobianNoise(dT);

            this.PPred = Fd * this.PEst * Fd' + Qd;

            % Do this because it means this class works even if there is no
            % update in the base class
            this.xEst = this.xPred;
            this.PEst = this.PPred;
        end
        
        function handleInitialConditionEvent(this, event)
            this.xEst = event.data;
            this.PEst = event.covariance;
            this.currentTime = event.time;
            this.xPred = this.xEst;
            this.PPred = this.PEst;
            this.initialized = true;
        end
        
        function handleGPSEvent(this, event)
            error('Implement this method for part 1');
        end
        
        function handleLaserEvent(this, event)
            error('Not supported; this should only be used to implement odometry and GPS')
        end
        
        function handleCameraEvent(this, event)
            error('Not supported; this should only be used to implement odometry and GPS')
        end

    end
    
    methods(Access = protected, Abstract)
        
        [xPred, Fd, Qd] = predictMeanJacobianNoise(this, dT);
        
    end
end