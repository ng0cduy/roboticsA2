classdef goods <handle
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
    nameModel;
    pos_ = eye(4);
    end
    methods (Access = public)
        function self = goods(name,pos)
            if strcmpi(name,'red') == 1
                self.nameModel = 'red.ply';
            
            elseif strcmpi(name,'blue') == 1
                self.nameModel = 'blue.ply';
              
            elseif strcmpi(name,'green') == 1
                self.nameModel ='green.ply';
            end
            [self.f,self.v,self.data] = plyread(self.nameModel,'tri'); 
            self.vUpdate = self.v + pos(1:3,4)'; 
            self.plotModel();
            self.MoveObject(pos);
            self.Find_faceNormal();
        end
        function plotModel(self)
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
        
        function [x,y,z] = getGoodsSize(self)
            x = max(self.data.vertex.x) - min(self.data.vertex.x);
            y = max(self.data.vertex.y) - min(self.data.vertex.y);
            z = max(self.data.vertex.z) - min(self.data.vertex.z);
        end
                
    end
    
end
        