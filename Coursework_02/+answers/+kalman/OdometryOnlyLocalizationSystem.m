classdef OdometryOnlyLocalizationSystem < minislam.localization.KalmanFilterLocalizationSystem
    
    % This class extends the localization system to properly handle vehicle
    % odometry
    
    methods(Access = public)
        
        % Call the base class constructor
        function this = OdometryOnlyLocalizationSystem()
            this = this@minislam.localization.KalmanFilterLocalizationSystem();
        end
        
    end
       
    methods(Access = protected)
    
                        
        
        function [xPred, Fd, Qd] = predictMeanJacobianNoise(this, dT)
            
            % Define function handler for state-transition function and its Jacobian
            f = @(x,u,dt) [ x(1) + u(1)*dt*cos(x(3) + 0.5*u(2)*dt);...
                            x(2) + u(1)*dt*sin(x(3) + 0.5*u(2)*dt);...
                            x(3) + u(2)*dt];
            % F.J its Jacobian : u constant, x variable 
            J = @(x,u,dt) [ 1 0 -u(1)*dt*sin(x(3) + 0.5*u(2)*dt);...
                            0 1 u(1)*dt*cos(x(3) + 0.5*u(2)*dt);...
                            0 0 1];
            
            % The control inputs are in this.u
            % vDT = dT * this.u(1);
            
            % The previous mean in the Kalmnan filter is this.xEst
            % xPred = this.xEst;
            % xPred(1) = xPred(1) + vDT; 
            xEst = this.xEst;
            u = this.u;
            
            
            xPred = f(xEst, u, dT);

            % The covariance prediction equation is            
            Fd = J(xEst, u, dT); % Fd is the jacobian of the state-transition function
            Qd = diag([0.2 0.1 0.1])*dT^2; % Process noise
            % PPred = Fd * PEst * Fd' + Qd;
            
        end
    end
end