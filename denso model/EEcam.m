classdef EEcam<handle
%     extract immage from a camera mounted at the endEffector of the arm
    properties (Constant)
        qz=zeros(1,6);

    end
    
     properties(Access =public)
         EE;
         color;
     end
     methods (Access = public)
         function self = EEcam(robot)
             view(0,90)
             ee = robot.endEffector;
             ee = ee(1:3,4);
             ee(3,1) = ee(3,1) - 0.1;
             set(gca,'Projection','perspective');
             target = ee;
             target(3,1) = 0.04;
             camtarget(target);
             self.EE =ee;
             campos(self.EE);
             camva(10);
             saveas(gcf, 'color.png');
             self.IsColor();
         end
         
         function IsColor(self)
             [r,g,b] = imread('color.png');
             if(r==255) 
                 self.color = 'red';
             elseif(g==255)
                     self.color = 'green';
             else
                 self.color = 'blue';
             end
                 
             
         end
         
             
     end
end
