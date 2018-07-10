% This class implements an event-based estimation system for building up a
% minimal, ideal SLAM system. The system is event-based and responds to a
% sequence of events which are time stamped and served in order.

classdef VehicleLocalizationSystem < handle
        
    properties(Access = protected)
        
        % Step number - how many times have we iterated?
        stepNumber;
        
        % The last time an event was processed.
        currentTime;
        
        % The vehicle control inputs. These are wheel speed and steer
        % angle. These are assumed to be held constant until the next
        % control input event comes along.
        u;
        uCov;
        
        % Flag shows if initialized or not. Initialization happens with the
        % first event
        initialized = false;
        
        % Flag to show if debugging is enabled
        debug = true;
    end
       
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = VehicleLocalizationSystem()
            
            % Set the start time to 0.
            this.currentTime = 0;
            
            this.stepNumber = 0;
            
            % Default odometry. This will stay the same until new
            % measurements are provided.
            this.u = zeros(2, 0);
            this.uCov = zeros(0, 0);
        end
        
        % Process a cell array which is a sequence of events. Each event is
        % processed in the order it appears in the array.
        function processEvents(this, events)
            
            this.stepNumber = this.stepNumber + 1;
            
            % If the event is not a cell, put it in an array for
            % convenience.
            if (iscell(events) == false)
                events = {events};
            end
            
            % Get the next event
            for k = 1 : length(events)                
                event = events{k};

                % First check it's of the right class.
                assert(isa(event, 'minislam.event_types.Event'));
                
                % Now do all the actual work
                this.processEvent(event);
            end
        end        
        
        function processEvent(this, event)
            
            event.time
            
            % If initialized, predict
            if (this.initialized == true)
                this.advanceToEventTime(event.time);   
            end
            
            this.currentTime = event.time;
            
            % Handle the event on the basis of its type. Note that if we
            % are not initialized yet, we can only handle the
            % odometry and initialization events.
            switch(event.type)
                
                case minislam.event_types.Event.INITIAL_CONDITION
                    this.handleInitialConditionEvent(event);

                case minislam.event_types.Event.VEHICLE_ODOMETRY
                    this.handleVehicleOdometry(event);
                    
                case minislam.event_types.Event.GPS
                    if (this.initialized == true)
                        this.handleGPSEvent(event);
                    end

                case minislam.event_types.Event.LASER
                    if (this.initialized == true)
                        this.handleLaserEvent(event);
                    end
 
                case minislam.event_types.Event.CAMERA
                    if (this.initialized == true)
                        this.handleCameraEvent(event);                
                    end
                    
                otherwise
                    error('Unknown observation type %d', event.type)     
            end            
        end
    end
        
    methods(Access = protected)
        
        % Handle a new odometry event. We simply change the vehicle
        % odometry to the new value.
        function handleVehicleOdometry(this, event)
            this.u = event.data;
            this.uCov = event.covariance;
        end
    end
    
    methods(Access = public, Abstract)
        
        % Get the mean and covariance of the estimate at the current time.
        % This is used for output purposes.
        [x, P] = getCurrentMeanAndCovarianceEstimate(this, optimizeIfNeeded);
        
        % Get the current landmarks estimates.
        [x, P] = getCurrentLandmarkEstimates(this, optimizeIfNeeded);
        
        % Recommend if an optimization is required
        recommendation = recommendOptimization(this);
        
        % Optimize
        optimize(this);
        
    end
    
    methods(Access = protected, Abstract)
                
        % Handle everything needed to predict to the current state.
        advanceToEventTime(this, time);
        
        % Handle a GPS measurement
        handleInitialConditionEvent(this, event);

        % Handle a GPS measurement
        handleGPSEvent(this, event);
            
        % Handle a set of laser measurements
        handleLaserEvent(this, event);
        
        % Handle a set of camera measurements
        handleCameraEvent(this, event);
    end
end
