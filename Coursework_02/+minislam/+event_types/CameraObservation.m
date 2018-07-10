% This class wraps up a camera measurement. This consists of a set of
% measurements each of the form (u,v) which is the pixel coodinate. Each measurement 
% has an associated landmark ID.

classdef CameraObservation < event_types.Event
    
    methods(Access = public)
        
        function this = CameraObservation(time, data, covariances, landmarkIDs)
            this = this@event_types.Event(time, event_types.Event.CAMERA, data, covariances, landmarkIDs);
        end
        
    end
end
