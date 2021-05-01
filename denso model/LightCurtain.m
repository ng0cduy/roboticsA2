classdef LightCurtain<handle
    properties(Constant)
%         qz=zeros(1,6);
    end
    properties(Access =public)
         verts;
         a=2;
         z=0;
         dist=2;
    end
    methods(Access = public)
        function self = LightCurtain()
            for i = -self.a:0.04:self.a
                for y = 1:4
                    hold on;
                    if y==1
                        tr = transl(i,self.a,self.z);
                    elseif y == 2
                        tr = transl(self.a,i,self.z);
                    elseif y==3
                        tr = transl(i,-self.a,self.z);
                    else
                        tr = transl(-self.a,i,self.z);
                    end
                    startP = tr(1:3,4)';

                    endP = tr(1:3,4)' + self.dist * tr(1:3,3)';
                    line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
                    plot3(endP(1),endP(2),endP(3),'r');
                    axis equal;
%                     verts = endP;
                end
            end
        end
    end
end

