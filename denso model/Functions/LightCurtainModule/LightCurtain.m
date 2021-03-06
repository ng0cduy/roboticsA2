classdef LightCurtain<handle
    properties(Constant)
%         qz=zeros(1,6);
    end
    properties(Access =public)
         verts;
         a=3;
         z=-0.5;
         dist=2;
         mode = false;
         tr;
         pose;
         collision=false;
    end
    methods(Access = public)
        function self = LightCurtain(mode,obstacle)
            self.mode = mode;
            self.Plot_LC;
            if nargin == 2
                self.detect_Obstacle(obstacle); 
                
            end
            
        end
%         Plot the light curtain
        function Plot_LC(self)
            if(self.mode == true)
                for i = -self.a:0.12:self.a
%                     for y = 1:4
%                         hold on;
%                         if y==1
%                             self.tr = transl(i,self.a,self.z);
%                         elseif y == 2
%                             self.tr = transl(self.a,i,self.z);
%                         elseif y==3
%                             self.tr = transl(i,-self.a,self.z);
%                         else
%                             self.tr = transl(-self.a,i,self.z);
%                         end
                    for j = 1:3
                        hold on;
                        if j==1
                            self.tr = transl(self.a,i,self.z);
                        elseif j == 2
                            self.tr = transl(i,-self.a,self.z);
                        else
                            self.tr = transl(-self.a,i,self.z);
                        end
                        startP = self.tr(1:3,4)';

                        endP = self.tr(1:3,4)' + self.dist * self.tr(1:3,3)';
                        line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r*');
                        plot3(endP(1),endP(2),endP(3),'r*');
%                         axis equal;
                        temp = self.tr(1:3,4)';
                        self.pose = [self.pose;temp];
                    end
                end
            else
                return
            end
        end
        
        function collisionDetect = DetectObstacle(self,obstacle)
              center =[0 0];
              [x,y,~] = obstacle.GetSize();
%               poseTemp = obstacle.UpdatePose();
              x0 = obstacle.pos_(1,4);
              y0 = obstacle.pos_(2,4);
              points= [x0 ,y0+y/2; x0+x/2,y0; x0,y0-y/2; x0-x/2,y0];  
              distFromCenter = [norm(center-points(1,:)),norm(center-points(2,:)),norm(center-points(3,:)),norm(center-points(4,:))];
            if obstacle.pos_(3,4) > self.z + self.dist || obstacle.pos_(3,4) < self.z
                self.collision = false;
            else 
%                 for i=1:size(self.pose(:,1),1) % each ray of the light curtain 
%                     if(transpose(obstacle.pos_(1:2,4)) <= self.pose(i,1:2)+0.01 |...
%                                 transpose(obstacle.pos_(1:2,4)) > self.pose(i,1:2)-0.05 )
%                         self.collision = true;
%                         return
%                     end
% 
%                 end
                    pointsInside = find(distFromCenter < self.a);
                    pointsOutside = find(distFromCenter > self.a);
                    if size(pointsInside,2) > 0 && size(pointsOutside,2) >0
                        self.collision =true;                    
                    else
                        self.collision = false;
                    end 
            end 
            if(self.collision ==true)
                    disp('collision detected');
            end
            collisionDetect = self.collision;
        end
    end
end

