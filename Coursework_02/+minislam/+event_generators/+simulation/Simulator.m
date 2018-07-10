classdef Simulator < minislam.event_generators.EventGenerator
    
    % This class is a simple simulator which predicts the movement of a
    % car-like vehicle and generates result from it. For the second part of
    % the coursework, we will use a different and more general set of
    % classes.
    
    properties(Access = protected)
        
        % The current simulation time
        currentTime;
        
        % The state of the vehicle (x, y, phi)
        x;
        
        % The control inputs (speed, steer angle)
        u;
        
        % The set of landmarks
        landmarks;
        
        % The set of waypoints
        waypoints;
        
        % Index to the waypoint
        waypointIndex;
        
        % Flag indicates we're done
        carryOnRunning;
        
        % Time of next GPS event
        nextGPSTime;
        
        % Time of next range-bearing sensor event
        nextLaserTime;
        
        % Debug. Set to 0 to disable noise.
        noiseScale = 1;
        
        % Directory containing the scenario files.
        scenarioDirectory;
         
    end
    
    methods(Access = public)
        
        function this = Simulator(parameters, scenarioDirectory)
            
            this = this@minislam.event_generators.EventGenerator(parameters);
            
            % Setup default
            if (nargin == 1)
                this.scenarioDirectory = 'task12';
            else
                this.scenarioDirectory = scenarioDirectory;
            end
            
            this.start();
        end
        
        % Start the simulator
        function start(this)
            this.currentTime = 0;
            this.waypointIndex = 1;
            
            this.loadLogFiles();
            
            this.x = [0, 0, 0]';
            this.u = zeros(2, 1);
            
            % Start everything off at the same time
            this.nextGPSTime = this.currentTime;
            this.nextLaserTime = this.currentTime;
            
            this.stepNumber = 0;
            
            this.mostRecentEvents = {};
        end
        
        % Return whether the simulator has finished
        function carryOn =  keepRunning(this)
            carryOn = this.carryOnRunning;
        end
        
        % Step the simulator and return events
        function step(this)

            % Create the odometry message first. We do this because the
            % measurement is used to predict to the next time and do the
            % update
            
            this.mostRecentEvents = {};
            
            % If this is start time, create an initialisation event. If
            % odometry is enabled, we have to send an initial empty
            % odometry event. This is so the estimators know to expect
            % odometry when the first odometry event appears.
            if (this.stepNumber == 0)
                
                if (this.parameters.enableOdometry == true)
                    odometryEvent = minislam.event_types.VehicleOdometryEvent(this.currentTime, ...
                        this.u, this.parameters.ROdometry);            
                    this.mostRecentEvents = cat(1, this.mostRecentEvents, {odometryEvent});
                end
                
                initialConditionEvent = minislam.event_types.InitialConditionEvent(this.currentTime, ...
                    this.x, zeros(3));
                this.mostRecentEvents = cat(1, this.mostRecentEvents, {initialConditionEvent});
            end

            % Bump the step number
            this.stepNumber = this.stepNumber + 1;

            % Bump the time step
            this.currentTime = this.currentTime + this.parameters.DT;
            
            % Predict forwards to the next step
            vDT = this.u(1) * this.parameters.DT;
            wDT = this.u(2) * this.parameters.DT;
            phi = this.x(3) + 0.5 * wDT;
            this.x(1) = this.x(1) + vDT * cos(phi);
            this.x(2) = this.x(2) + vDT * sin(phi);            
            this.x(3) = this.x(3) + wDT;

            % Compute the GPS observation if necessary
            gpsEvents = this.simulateGPSEvents();            
            this.mostRecentEvents = cat(1, this.mostRecentEvents, gpsEvents);

            % Compute the laser observation if necessary
            laserEvents = this.simulateLaserEvents();            
            this.mostRecentEvents = cat(1, this.mostRecentEvents, laserEvents);

            % Determine the wheel speed and steer angle for the robot which
            % will be applied next time
            this.computeControlInputs();
            
            % If requested, create the odometry event
            if (this.parameters.enableOdometry == false)
                return
            end
            
            odometryMeasurement = [this.u(1); this.u(2)] + this.noiseScale * sqrtm(this.parameters.ROdometry) * randn(2, 1);
            odometryEvent = minislam.event_types.VehicleOdometryEvent(this.currentTime, ...
                odometryMeasurement, this.parameters.ROdometry);            
            this.mostRecentEvents = cat(1, this.mostRecentEvents, {odometryEvent});
        end
        
        % Get the ground truth state of the vehicle; not available in the
        % real world (alas)
        function groundTruthState = getGroundTruth(this, getFullStateInformation)
            groundTruthState = minislam.event_generators.simulation.SimulatorState();
            
            % Required information
            groundTruthState.currentTime = this.currentTime;
            groundTruthState.xTrue = this.x;
            groundTruthState.uTrue = this.u;
            
            % Optional information
            if (getFullStateInformation == true)
                groundTruthState.waypoints = this.waypoints;
                groundTruthState.mTrue = this. landmarks;
            end
        end
    end
            
    methods(Access = protected)

        function computeControlInputs(this)
            
            % Work out distance to the target waypoint
            dX = this.waypoints(:, this.waypointIndex) - this.x(1:2);
            d = norm(dX);
            
            % If sufficiently close, switch to the next waypoint;
            if (d < 1)
                this.waypointIndex = this.waypointIndex + 1;
                
                % If we've reached the end of the list of waypoints, return
                if (this.waypointIndex > size(this.waypoints, 2))
                    this.carryOnRunning = false;
                    return;
                end
                
                % Update to the new waypoint
                dX = this.x(1:2) - this.waypoints(:, this.waypointIndex);
                d = norm(dX);
            end
            
            % Compute the speed. We first clamp the acceleration, and then
            % clamp the maximum and minimum speed values.
            diffSpeed = 0.1 * d - this.u(1);
            maxDiffSpeed = this.parameters.maxAcceleration * this.parameters.DT;
            diffSpeed = min(maxDiffSpeed, max(-maxDiffSpeed, diffSpeed));
            this.u(1) = max(this.parameters.minSpeed, min(this.parameters.maxSpeed, this.u(1) + diffSpeed));

            % Compute the steer angle. We first clamp the rate of change,
            % and then clamp the maximum and minimum steer angles.
            diffDelta = minislam.utils.pi_to_pi(atan2(dX(2), dX(1)) - this.x(3) - this.u(2));
            maxDiffDelta = this.parameters.maxDiffDeltaRate * this.parameters.DT;            
            diffDelta = min(maxDiffDelta, max(-maxDiffDelta, diffDelta));
            this.u(2) = min(this.parameters.maxDelta, max(-this.parameters.maxDelta, this.u(2) + diffDelta));
            
            % Flag that we shoul d keep running
            this.carryOnRunning = true;            
        end
        
        function gpsEvents = simulateGPSEvents(this)
            
            if ((this.parameters.enableGPS == false) || (this.currentTime < this.nextGPSTime))
                gpsEvents = {};
                return
            end
                
            this.nextGPSTime = this.nextGPSTime + this.parameters.gpsMeasurementPeriod;
            gpsMeasurement = [this.x(1); this.x(2)] + this.noiseScale * sqrtm(this.parameters.RGPS) * randn(2, 1);
            gpsEvents = {minislam.event_types.GPSObservationEvent(this.currentTime, gpsMeasurement, ...
                this.parameters.RGPS)};
        end
            
        function laserEvents = simulateLaserEvents(this)
            
            laserEvents = {};
            if ((this.parameters.enableLaser== false) || (this.currentTime < this.nextLaserTime))
                return
            end
            
            this.nextLaserTime = this.nextLaserTime + this.parameters.laserMeasurementPeriod;
            
            % Find the landmarks which are in range
            
            % Compute the relative distance. Note the vehicle always sits
            % with z=0.
            dX = this.landmarks;
            dX(1, :) = dX(1, :) - this.x(1);
            dX(2, :) = dX(2, :) - this.x(2);
            
            % Squared range to each landmark
            R2 = sum(dX.^2,1);
            R = sqrt(R2);
            
            ids = find(R <= this.parameters.laserDetectionRange);
            
            % If nothing to see, return
            if (isempty(ids))
                return
            end
            
            numLandmarks = length(ids);
            
            % Create observations
            r = R(ids) + this.noiseScale * sqrt(this.parameters.RLaser(1,1)) * randn(1, numLandmarks);
            az = minislam.utils.pi_to_pi(atan2(dX(2, ids), dX(1, ids)) - this.x(3) ...
                + this.noiseScale * sqrt(this.parameters.RLaser(2,2)) * randn(1, numLandmarks));
            el = atan2(dX(3, ids), sqrt(sum(dX(1:2,ids).^2,1))) ...
                + this.noiseScale * sqrt(this.parameters.RLaser(3,3)) * randn(1, numLandmarks);
            
            % Package into a single event
            laserEvents = {minislam.event_types.LaserObservationEvent(this.currentTime, ...
                [r; az; el], this.parameters.RLaser, ids)};
        end
        
        function loadLogFiles(this)
            
            % Find the full directory
            fullDirectoryWhat = what(this.scenarioDirectory);
            
            % Get the path
            fullPath = fullDirectoryWhat.path();
            
            this.x = load(fullfile(fullPath, 'x0.txt'))';
            this.landmarks = load(fullfile(fullPath, 'lm.txt'))';
            this.waypoints = load(fullfile(fullPath, 'wp.txt'))';
        end
    end
end