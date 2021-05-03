classdef ImgRead < handle
     properties
         qx=zeros(2,2);
         qy=zeros(2,2);
         qz=zeros(2,2);
         ImgName;
     end
     %% 
     methods (Access=public) %Class for adding image to workspace
         %% Function to add an Image to Workspace
         function self=ImgRead(qx,qy,qz,ImgName)
             self.ImgName=ImgName;
             self.qx=qx;
             self.qy=qy;
             self.qz=qz;
             
             self.PlotImg;
             
         end
         
         function PlotImg(self)
             surf(self.qx,self.qy,self.qz,...
             'CData',imread(self.ImgName),'FaceColor','texturemap');
         end
     end
end