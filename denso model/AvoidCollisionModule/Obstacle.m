classdef Obstacle <handle
    properties(Access = public)        
    f;
    v;
    vUpdate;
    data;
    faceNormals;
    mesh_h;
    midpoint;
    modelVertexCount;
    modelVert;
    updatePose;
    pos_ = eye(4);
    P;
    x;
    y;
    z;
    end
    methods (Access = public)
        function self = Obstacle(pos)                     
            [self.f,self.v,self.data] = plyread("Obstacle.ply",'tri'); 
            self.vUpdate = self.v + pos(1:3,4)'; 
            self.PlotModel();
            self.MoveObject(pos);
            self.Find_faceNormal();
            self.GetSize();
            self.CalculateP();
                       
        end
        function PlotModel(self)
            %compute the number of vertices
            self.modelVertexCount = size(self.v,1);
            
            %compute the middle point in the model
            self.midpoint = sum(self.v)/self.modelVertexCount;
            
            %move all the vertices a distance of the midpoint
            self.modelVert = (self.v - repmat(self.midpoint,self.modelVertexCount,1));
            
            %convert vertices color to 0-1
            vertexColours = [self.data.vertex.red, self.data.vertex.green, self.data.vertex.blue] / 255;
            
            %plot the model
            self.mesh_h = trisurf(self.f,self.modelVert(:,1),self.modelVert(:,2),self.v(:,3),'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','flat');   
        end
        function UpdatePose(self,new_pose)
            self.updatePose = [new_pose * [self.modelVert,ones(self.modelVertexCount,1)]']';
            self.mesh_h.Vertices = self.updatePose(:,1:3);
            self.pos_ = new_pose;
        end
        function MoveObject(self,pose)
            self.UpdatePose(pose);
            self.pos_ = pose;
        end
        function Find_faceNormal(self)
                self.faceNormals = zeros(size(self.f,1),3);
                for faceIndex = 1:size(self.f,1)
                    v1 = self.v(self.f(faceIndex,1)',:);
                    v2 = self.v(self.f(faceIndex,2)',:);
                    v3 = self.v(self.f(faceIndex,3)',:);
                    self.faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
                end
        end
        
        function [x,y,z] = GetSize(self)
            self.x = max(self.data.vertex.x) - min(self.data.vertex.x);
            self.y = max(self.data.vertex.y) - min(self.data.vertex.y);
            self.z = max(self.data.vertex.z) - min(self.data.vertex.z);
            x=self.x;
            y=self.y;
            z=self.z;
        end
        
        function CalculateP(self)
            x_=self.vUpdate(:,1,1);
            y_=self.vUpdate(:,2,1);
            z_=self.vUpdate(:,3,1);
            P_ = [max(x_)-0.1, max(y_)-0.1,max(z_);...
                      min(x_)+0.1,max(y_)-0.1,max(z_);...
                      min(x_)+0.1,min(y_)+0.1,max(z_);
                      max(x_)-0.1,min(y_)+0.1,max(z_);...
                      ];
                  self.P=P_';
        end
        
        function cubePoints = CreateMesh(self,option)
            xMas = max(self.data.vertex.x);
            yMas = max(self.data.vertex.y);
            zMas = max(self.data.vertex.z);
            
            xMin = min(self.data.vertex.x);
            yMin = min(self.data.vertex.y);
            zMin = min(self.data.vertex.z);   
            
            [Y,Z] = meshgrid(yMin:0.05:yMas,zMin:0.05:zMas);
            sizeMat = size(Y);
            X = repmat(xMin,sizeMat(1),sizeMat(2));
%             oneSideOfCube_h = surf(X,Y,Z);
            cubeSide1Points = [X(:),Y(:),Z(:)];
            
            cubeSide2Points = [cubeSide1Points(:,1)+self.x,cubeSide1Points(:,2:3)];
            
            [X,Y] = meshgrid(xMin:0.04:xMas,yMin:0.04:yMas);
            sizeMat = size(X);
            Z = repmat(zMin,sizeMat(1),sizeMat(2));
            cubeBottomPoints = [X(:),Y(:),Z(:)];
            
            cubeTopPoints = [cubeBottomPoints(:,1:2),cubeBottomPoints(:,3)+self.z];
            
            [X,Z] = meshgrid(xMin:0.04:xMas,zMin:0.04:zMas);
            sizeMat = size(X);
            Y = repmat(yMin,sizeMat(1),sizeMat(2));
            cubeSide3Points = [X(:),Y(:),Z(:)];
            
            cubeSide4Points = [cubeSide3Points(:,1),cubeSide3Points(:,2)+self.y,cubeSide3Points(:,3)];
            
            cubePoints = [ cubeSide1Points ...
             ; cubeSide2Points  ...
             ; cubeSide3Points ...
             ; cubeSide4Points ...
             ; cubeBottomPoints ...
             ; cubeTopPoints]; 
            
            if strcmp(option,'AtOrigin')
                return
            end
%             cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'cyan.');
            centre = self.pos_(1:3,4)';
            cubePoints = cubePoints + repmat(centre,size(cubePoints,1),1); % move the cube to the required location
            cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'cyan.');
        end
                
    end
    
end
        