% This class wraps up a "GPS" measurement. The "GPS" here is assumed to be
% some kind of ideal sensor which provides a suitable measurement in a
% local tangent plane (=simplest sensor possible).

classdef GPSObservationEvent < minislam.event_types.Event
    
    methods(Access = public)
        
        function this = GPSObservationEvent(time, data, covariance)
            this = this@minislam.event_types.Event(time, minislam.event_types.Event.GPS, data, covariance);
        end
        
    end
end
