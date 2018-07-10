% This class stores a dictionary of IDs to keys. It can be used to access
% registered keys in a GTSAM graph using, say, a numerical identity.

% Just to confuse things, the "key" used in the store here is the ID value
% that you'd use. The "value" is the GTSAM key you'd use to look stuff up
% in the graph. Anything else would be easy.

classdef KeyStore < handle
   
    properties(Access = protected)

        % The store of key-value pairs
        keyStore;
        
    end
    
    methods(Access = public)
        
        function this = KeyStore()
            % Create the map object. The key is a 'double' for simplicity
            this.keyStore = containers.Map('KeyType', 'double', 'ValueType', 'uint64');
        end
    
        function insert(this, id, value)
            this.keyStore(id) = value;            
        end
        
        function put(this, id, value)
            this.keyStore(id) = value;            
        end
        
        function remove(this, id)
            this.keyStore.remove(id);
        end
        
        function registered = isRegistered(this, id)
            registered = this.keyStore.isKey(id);
        end

        function registered = contains(this, id)
            registered = this.keyStore.isKey(id);
        end

        function value = get(this, id)
            value = this.keyStore(id);
        end
        
        function valueSet = values(this)
            valueSet = values(this.keyStore);
        end
    end    
end