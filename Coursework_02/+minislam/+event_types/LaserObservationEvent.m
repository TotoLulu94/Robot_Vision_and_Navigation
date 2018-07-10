% This class wraps up a depth camera measurement. This consists of a set of
% measurements each of the form (d, u,v) where d is the distance and (u,v)
% are the pixel coordinaes. Each measurement has an associated landmark ID.

classdef LaserObservationEvent < minislam.event_types.Event
    
    methods(Access = public)
        
        function this = LaserObservationEvent(time, data, covariances, landmarkIDs)
            this = this@minislam.event_types.Event(time, minislam.event_types.Event.LASER, data, covariances, landmarkIDs);
        end
        
    end
end
