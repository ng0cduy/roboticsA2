classdef LightCurtain<handle
    properties(Constant)
%         qz=zeros(1,6);
    end
    properties(Access =public)
         verts;
         a=1;
         z=0;
    end
    methods(Access = public)
        function self = LightCurtain()
            for i = -self.a:0.1:self.a
                for y = 1:4
                    hold on;
                    if y==1
                        tr = transl(i,1,self.z);
                    elseif y == 2
                        tr = transl(1,i,self.z);
                    elseif y==3
                        tr = transl(i,-1,self.z);
                    else
                        tr = transl(-1,i,self.z);
                    end
                    startP = tr(1:3,4)';

                    dist = 1;

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

