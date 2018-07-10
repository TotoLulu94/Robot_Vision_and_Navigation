% This class stores an observation event

classdef Event < handle

    % Enumeration of observation types
    properties(Access = public, Constant = true)
        INITIAL_CONDITION = 0;
        VEHICLE_ODOMETRY = 1;
        GPS = 2;
        LASER = 3;
        CAMERA = 4;
    end
    
    properties(Access = public)
        
        % The time of the event.
        time; 
        
        % The type of the event. Must be one of the enums listed above.
        type;
        
        % The IDs (if relevant) of the data. This would be the ID of the
        % landmark, for example.
        landmarkIDs;
        
        % The event data
        data;
        
        % The noise on the event data
        covariance;
    end
    
    methods(Access = public)
        function this = Event(time, type, data, covariance, landmarkIDs)
            
            % Copy over the common values
            this.time = time;
            this.type = type;
            this.data = data;
            
            % If the noise is a vector, assume that it encodes a diagonal covariance
            % matrix. Therfore, reshape into a matrix.
            if ((size(covariance, 1) == 1) || (size(covariance, 2) == 1))
                covariance = diag(covariance);
            end            
            this.covariance = covariance;
            
            % Assign the ID if provided
            if (nargin == 5)
                this.landmarkIDs = landmarkIDs;
            end
        end
    end
    
end
