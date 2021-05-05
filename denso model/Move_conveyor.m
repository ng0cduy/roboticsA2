classdef Move_conveyor<handle
    properties(Access=public)
    end
    methods(Access=public)
        function self=Move_conveyor(object)
            for i=-0.5:0.01:0.8
                 object.Move(transl(i,-0.08,0.32));
%                  pause(0.2);
            end
        end
    end
end