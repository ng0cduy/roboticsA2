%% class to move the goods on the conveyor
classdef Move_conveyor<handle
    properties(Access=public)
    end
    methods(Access=public)
        function self=Move_conveyor(robot,object)
            
            for i=-0.5:0.01:0.8
                 robot.checkEStop();
                 object.Move(transl(i,-0.08,0.32));
%                  pause(0.05);
            end
        end
    end
end