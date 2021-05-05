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
%              [r,g,b] = imread('color.png');
%              if(r==255) 
%                  self.color = 'red';
%              elseif(g==255)
%                      self.color = 'green';
%              else
%                  self.color = 'blue';
%              end
            img_data = imread('color.png');
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
            bPixel = sum(find(blueMask>0),'all');
            gPixel = sum(find(greenMask>0),'all');
            if bPixel == 0 && gPixel ==0
                self.color = 'red';
            elseif rPixel ==0 && gPixel ==0
                self.color = 'blue';
            elseif bPixel == 0 && rPixel ==0
                self.color == 'green';
            end
%             r_mean = mean(r,'all');
%             b_mean = mean(b,'all');
%             g_mean = mean(g,'all');
%             if r_mean >=200 && b_mean <= 75 && g_mean <=75
%                 self.color = 'red';
%             elseif r_mean <=75 && b_mean <= 75 && g_mean >=200
%                 self.color = 'green';
%             elseif r_mean <= 75 && b_mean >=200 && g_mean <=75
%                  self.color = 'blue';
%             elseif r_mean <=50 && b_mean <= 50 && g_mean <=50
%                 self.color = 'black';
%             elseif r_mean <= 150 && r_mean >= 120 && g_mean <= 150 && g_mean >= 120 && b_mean <= 150 && b_mean >= 120
%                 self.color = 'gray';
%             else 
%                 self.color = 'None'
%             end     
         end
         
             
     end
end
