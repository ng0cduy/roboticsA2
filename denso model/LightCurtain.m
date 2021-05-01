classdef LightCurtain<handle
    properties(Constant)
        qz=zeros(1,6);
    end
    properties(Access =public)
         verts;
    end
    methods(Access = public)
        function self = LightCurtain()
            for i = 1:0.2:5
                for y = 1:1:4
                    hold on;
                    if y==1
                        tr = transl(i,1,1);
                    elseif y == 2
                        tr = transl(1,i,1);
                    elseif y==3
                        tr = transl(i,5,1);
                    else
                        tr = transl(5,i,1);
                    end
                    startP = tr(1:3,4)';

                    dist = 2;

                    endP = tr(1:3,4)' + dist * tr(1:3,3)';
                    line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
                    plot3(endP(1),endP(2),endP(3),'r');
                    axis equal;
%                     verts = endP;
                end
            end
        end
    end
end

