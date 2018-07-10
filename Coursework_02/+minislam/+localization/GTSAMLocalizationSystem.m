% This class implements an event-based estimation system using GTSAM and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% extend this class to implement additional methods to create and insert
% new landmarks and new features.

classdef GTSAMLocalizationSystem < minislam.localization.VehicleLocalizationSystem
    
    properties(Access = protected)
                
        % The graph used for performing estimation.
        factorGraph;
        
        % The values in the graph.
        values;
        
        % The most recently created vehicle pose and associated pose key.
        % This corresponds to the vertex on the very front of the graph and
        % is the vertex which describes the currentEventTime.
        currentVehiclePose;
        
        % The key of the most recently created vertex.
        currentVehiclePoseKey;
        
        % This stores the vehicle keys for ease of looking up
        vehiclePoseKeyStore;

        % This stores the landmark keys for ease of looking up
        landmarkIDKeyStore;
    end
    
    properties(Access = private)
        
        % A counter which is used to create unambiguous keys. This must be
        % an integer.
        vehicleVertexCounter;

    end
        
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = GTSAMLocalizationSystem()
            
            % Call the base class constructor
            this = this@minislam.localization.VehicleLocalizationSystem();
            
            % Create the factor graph and the associated values. These need to be kept in "lock step" with one another. 
            this.factorGraph = gtsam.NonlinearFactorGraph();
            this.values = gtsam.Values();
            
            % Create the vehicle counter which is used to generate the vehicle verticles.
            this.vehicleVertexCounter = uint32(0);
            
            % Vehicle pose key store
            this.vehiclePoseKeyStore = minislam.utils.KeyStore();
            
            % Landmkark key store
            this.landmarkIDKeyStore = minislam.utils.KeyStore();
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. Therefore, this method returns if the
        % localization algorithm thinks optimizing is a good idea. Here we
        % always return true. This shouldn't hurt anything, but stuff can
        % run much more slowly as a result.
        function recommendation = recommendOptimization(this)
            recommendation = rem(this.stepNumber, 10) == 0;
        end
        
        % This method runs the graph optimiser and the outputs of the
        % optimisation are used as the initial conditions for the next
        % round of the graph. This makes it possible to iterative compute
        % solutions as more information becomes available over time. If the
        % marginals are requested, these big boys are computed as well.
        function optimize(this)
            
            % Set defaults if not defined
            lambdaUpperBound = 1e9;
            maxIterations = 200;
            
            % Set the optimization parameters
            params = gtsam.LevenbergMarquardtParams();
            params.setlambdaUpperBound(lambdaUpperBound);            
            params.setMaxIterations(maxIterations);
            
            % Set the debug level
%             if (this.debug == true)
%                 params.setVerbosityLM('SUMMARY');
%             end
%             
            % Run the optimizer and pull out the results.
            optimizer = gtsam.LevenbergMarquardtOptimizer(this.factorGraph, this.values, params);
            results = optimizer.optimizeSafely();
            
            % Now set the current values to the results. This is the trick
            % which makes iterative calculation possible.
            this.values = results;
            
            % Set the current pose to the optimised value
            this.currentVehiclePose = this.values.atPose2(this.currentVehiclePoseKey);
        end
                
        function [x, P] = getCurrentMeanAndCovarianceEstimate(this)
            x = this.currentVehiclePose;
            try
                marginals = gtsam.Marginals(this.factorGraph, this.values);
                P = marginals.marginalCovariance(this.currentVehiclePoseKey);
            catch e
                this.factorGraph
                keyboard
            end
        end
        
        % Return two cell arrays providing mean and covariance data
        function [x, P] = getCurrentLandmarkEstimates(this, optimizeIfNeeded)
            if (nargin == 1)
                optimizeIfNeeded = true;
            end
            
            if ((optimizeIfNeeded == true) || (isempty(this.lastOptimizedValues) == true))     
                this.optimize();
            end
            
            landmarkKeys = this.landmarkIDKeyStore.values();
            
            numLandmarks = length(landmarkKeys);
            
            marginals = gtsam.Marginals(this.factorGraph, this.values);
            
            x = zeros(3, numLandmarks);
            
            P = zeros(3, 3, numLandmarks);
            
            for n = 1 : numLandmarks
                landmarkKey = landmarkKeys{n};
                x(:, n) = this.values.atPoint3(landmarkKey).vector();
                P(:, :, n) = marginals.marginalCovariance(landmarkKey);
            end
        end

        
    end
        
    % These are the methods you will need to overload
    methods(Access = protected)
                        
        % Declare bodies all of these methods. This makes it possible to
        % instantiate this object, although it will flip out and cause an
        % error.
        
        function handleGPSEvent(this, event)
            error('Implement me!');
        end
        
        function handleLaserEvent(this, event)
            error('Implement me!');
        end
        
        function handleCameraEvent(this, event)
            error('Implement me!');
        end
        
        function [relativeTransform, relativeTransformCovariance] = computeRelativeVehicleTransformAndCovariance(this)
            relativeTransform = [2 0 0]';
            relativeTransformCovariance = diag([1 1e-6 0.1*pi/180].^2);
        end
    end
    
    methods(Access = protected)
        
        % Initialize
        function initialize(this, time)            
            % Create the first vehicle vertex. The boolean flag shows
            % whether we add a prior to it or not.
            this.createFirstVehiclePose(true);
            this.currentTime = time;
        end
        
        function addNewFactor(this, newFactor)
            this.factorGraph.add(newFactor);
        end
        
        function addNewVariable(this, newKey, initialValue)
            this.values.insert(newKey, initialValue);
        end
        
        function addNewFactorAndVariable(this, newKey, newFactor, initialValue)
            this.factorGraph.add(newFactor);
            this.values.insert(newKey, initialValue);
        end
        
        % This method does two things. First, it creates a new vehicle pose
        % associated with the current time. Second, it does other stuff.
        % Whee!
        function advanceToEventTime(this, time)            
            % Work out the time step length. This is the interval we
            % will predict over.
            dT = time - this.currentTime;
            
            % Nothing to do if it's really close to the last time
            if (abs(dT) < 1e-3)
                return
            end
            
            % The vehicle pose we are going to attach to
            newVehiclePoseKey = this.getNewVehiclePoseKey();
           
            % Get the 2D kinematic model. This has to be turned into a 3D
            % pose for GTSAM to work.
            [relativePose, relativePoseCovariance] = ...
                this.computeRelativeVehicleTransformAndCovariance(dT);

            % Pad with stabilising noise
            relativePoseCovariance(3,3) = relativePoseCovariance(3,3) + (dT *0.01*pi/180)^2;

            % If debugging is enabled, check to see that the noise
            % covariance is good
            if (this.debug == true)
                try                
                    chol(relativePoseCovariance);
                catch e
                    warning('The relativeTransformCovariance matrix is not positive semidefinite.');
                    relativePoseCovariance
                    keyboard
                end
            end

            
            % Construct the covariance matrix for the process noise
            relativePoseNoise = gtsam.noiseModel.Gaussian.Covariance(relativePoseCovariance);
            
            % Insert the odometry edge
            this.factorGraph.add(gtsam.BetweenFactorPose2(this.currentVehiclePoseKey, ...
                newVehiclePoseKey, relativePose, relativePoseNoise));
            
            % Assign the initial value to the vehicle pose. This is given
            % by predicting with the process model
            vehiclePose = this.currentVehiclePose.compose(relativePose);
            
            % Associate the newly created pose key with the newly created
            % vehicle pose.
            this.values.insert(newVehiclePoseKey, gtsam.Pose2());
            
            % Store the last vehicle pose and pose key
            this.currentVehiclePose = vehiclePose;
            this.currentVehiclePoseKey = newVehiclePoseKey;
        end
                
    end
    
    methods(Access = private)

        % This method creates the first vehicle pose in the graph. It
        % creates a Pose2 and puts it at the origin with a small prior covariance.  
        function createFirstVehiclePose(this, createPrior)
            
            % First create and insert the vehicle pose
            this.currentVehiclePoseKey = this.getNewVehiclePoseKey();
            this.currentVehiclePose = gtsam.Pose2();
            this.values.insert(this.currentVehiclePoseKey, this.currentVehiclePose);
            this.vehiclePoseKeyStore.insert(this.vehicleVertexCounter - 1, this.currentVehiclePoseKey);

            % If a prior is requested, create it and add it to the graph
            if (createPrior == true)
                priorMean = gtsam.Pose2();
                priorNoise = gtsam.noiseModel.Diagonal.Variances(1e-12*ones(3,1));
                this.factorGraph.add(gtsam.PriorFactorPose2(this.currentVehiclePoseKey, priorMean, priorNoise));
            end
        end 
       
        % This helper function serves up unique keys for the robot.
        function vehiclePoseKey = getNewVehiclePoseKey(this)
            vehiclePoseKey = gtsam.symbol('x', this.vehicleVertexCounter);
            this.vehicleVertexCounter = this.vehicleVertexCounter + 1;
        end
    end
    
end
