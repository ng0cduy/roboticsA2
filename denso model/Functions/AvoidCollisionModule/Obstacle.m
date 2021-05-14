%% class for creating obstacle
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
    cubePoints;
    name;
    end
    methods (Access = public)
        %% contructive function
        function self = Obstacle(name,pos)         
            self.name = name;
            [self.f,self.v,self.data] = plyread(self.name,'tri'); 
            self.vUpdate = self.v + pos(1:3,4)'; 
            self.PlotModel();
            self.Move(pos);
            self.Find_faceNormal();
            self.GetSize();
            self.CalculateP();
            self.pos_ = pos;
            self.UpdatePose(pos);        
        end
        %% function to plot model
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
        %% function to update pose and animation
        function UpdatePose(self,new_pose)
            self.updatePose = [new_pose * [self.modelVert,ones(self.modelVertexCount,1)]']';
            self.mesh_h.Vertices = self.updatePose(:,1:3);
            self.pos_ = new_pose;
        end
        %% function to update pose and animation
        function Move(self,pose)
            self.UpdatePose(pose);
            self.pos_ = pose;
        end
        
        %% function to find the faces' normal vectors for the model
        function Find_faceNormal(self)
                self.faceNormals = zeros(size(self.f,1),3);
                for faceIndex = 1:size(self.f,1)
                    v1 = self.v(self.f(faceIndex,1)',:);
                    v2 = self.v(self.f(faceIndex,2)',:);
                    v3 = self.v(self.f(faceIndex,3)',:);
                    self.faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
                end
        end
        
        %% function to get the dimensions of the object
        function [x,y,z] = GetSize(self)
            self.x = max(self.data.vertex.x) - min(self.data.vertex.x);
            self.y = max(self.data.vertex.y) - min(self.data.vertex.y);
            self.z = max(self.data.vertex.z) - min(self.data.vertex.z);
            x=self.x;
            y=self.y;
            z=self.z;
        end
        %% function to determine object's P for visual servo to detect pose
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
        %% Create mesh data for collision detection using ellipsoid
        function cubePoints = CreateMesh(self,option)
            % dimensions of the object
            xMas = max(self.data.vertex.x);
            yMas = max(self.data.vertex.y);
            zMas = max(self.data.vertex.z);
            
            xMin = min(self.data.vertex.x);
            yMin = min(self.data.vertex.y);
            zMin = min(self.data.vertex.z);   
            
            % create mesh for 4 side faces and top, bottom faces
            [Y,Z] = meshgrid(yMin:0.05:yMas,zMin:0.05:zMas);
            sizeMat = size(Y);
            X = repmat(xMin,sizeMat(1),sizeMat(2));

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
            
            % accumulate all the points
            cubePoints = [ cubeSide1Points ...
             ; cubeSide2Points  ...
             ; cubeSide3Points ...
             ; cubeSide4Points ...
             ; cubeBottomPoints ...
             ; cubeTopPoints]; 
            
            % in case transforming the mesh to a different locations
            if strcmp(option,'AtOrigin')
                return
            end
            
            % in case return the mesh at the current location of the object
            centre = self.pos_(1:3,4)';
            cubePoints = cubePoints + repmat(centre,size(cubePoints,1),1); % move the cube to the required location
            self.cubePoints = cubePoints;
            cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'cyan.'); % plotting for clear visualisation
            
        end
                
    end
    
end