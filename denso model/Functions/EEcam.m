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
             self.Run(robot);
             pause(0.05);
             self.IsColor();
         end
         
         function Run(self,robot)
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
             saveas(gcf, 'gcolor.png');
         end
         
         function IsColor(self)
            img_data=0;
            img_data = imread('gcolor.png');
            [redBand,greenBand,blueBand] = imsplit(img_data);
            redThresholdLow = 175;
            redThresholdHigh = 255;
            greenThresholdLow = 170;
            greenThresholdHigh = 255;
            blueThresholdLow = 170;
            blueThresholdHigh = 180;
            redMask = (redBand >= redThresholdLow) & (redBand <= redThresholdHigh);
            greenMask = (greenBand >= greenThresholdLow) & (greenBand <= greenThresholdHigh);
            blueMask = (blueBand >= blueThresholdLow) & (blueBand <= blueThresholdHigh);
            rPixel = sum(find(redMask>0),'all');      
            gPixel = sum(find(greenMask>0),'all');
            bPixel = sum(find(blueMask>0),'all');
            if bPixel == 0 && gPixel ==0
                self.color = 'red';
            elseif bPixel == 0 && rPixel ==0
                self.color = 'green';
            else
                self.color = 'blue';
            end  
         end
         
             
     end
end
