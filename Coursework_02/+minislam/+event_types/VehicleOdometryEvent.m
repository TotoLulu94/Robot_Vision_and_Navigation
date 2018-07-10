% This type of event wraps up the vehicle control inputs. These consist of
% linear speed and angular velocity.

classdef VehicleOdometryEvent < minislam.event_types.Event   
    
    methods(Access = public)        
        
        function this = VehicleOdometryEvent(time, data, covariance)
            this = this@minislam.event_types.Event(time, minislam.event_types.Event.VEHICLE_ODOMETRY, data, covariance);
        end
        
    end
end
    
