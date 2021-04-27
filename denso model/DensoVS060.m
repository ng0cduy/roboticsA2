classdef DensoVS060<handle
    properties (Constant)
        qz = zeros(1,6);
    end
    properties(Access =public)
        base;
        endEffector;
        model;
        qMatrix;
        useGripper = false;   
        name;
        initialPose=zeros(1,6);
        pose
        workspace = [-1 1 -1 1 -0.3 1]; 
    end
    methods (Access = public) %% Class for DensoVS060 robot simulation
        %% Define robot Function  
        function self = DensoVS060(useGripper, base, name)
            self.useGripper = useGripper;  
            self.base = base;
            self.name = name;
            self.getDensoVS060();
            self.SetBase(base);
        end
        %% Create Denso Links
        function getDensoVS060(self)       
%             L1 = Link([pi     0       0       pi/2    1.001]);
            L1=Link('alpha',pi/2,'a',0, 'd',0.148, 'offset',pi,'qlim',[deg2rad(-170), deg2rad(170)] );
            L2=Link('alpha',0,'a',0.305, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]);
            L3=Link('alpha',-pi/2,'a',0, 'd',0, 'offset',0,'qlim',[deg2rad(-125), deg2rad(155)]);
            L4=Link('alpha',pi/2,'a',0, 'd',0.3, 'offset',0,'qlim',[deg2rad(-270), deg2rad(270)]);
            L5=Link('alpha',-pi/2,'a',0, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]);
            L6=Link('alpha',0,'a',0, 'd',0.06, 'offset',0,'qlim',[deg2rad(-360), deg2rad(360)]);
%             L1.qlim = [-0.8 0];            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',self.name);
        end
        %% Set Denso Base Location 
        function SetBase(self,base)
%             self.model.base=base*trotz(-pi/2)*trotx(pi/2);
            self.model.base = base;
            self.PlotandColorUR3();
        end
        
        %% Import PLY files to get the 3D model
        function PlotandColorUR3(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['J',num2str(linkIndex),'.ply'],'tri'); 
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
%             self.model.plot3d(zeros(1,self.model.n),'workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        %% ForWard Kinematic for UR3
        function ee_pose = FKine(self,q)
              self.endEffector = self.model.fkine(q);
              ee_pose = self.endEffector;
        end
        %% InverseKinematic for UR3
        function q_ = IKine(self,transform)
    %           self.pose=transform;
              steps = 15;
              poseArr   = cell(1,steps);
              qArr      = cell(1,steps);
              rotMat    = cell(1,steps);
              transMat  = cell(1,steps);
              errRotMat    = cell(1,steps);
              errRot    = cell(1,steps);
              errTransl  = cell(1,steps);
              errTot    = cell(1,steps);
              stepsQ = (self.model.qlim(:,2) - self.model.qlim(:,1))/steps; 
              [Rd,Td] = tr2rt(transform);
              for i = 0:1:steps                
                  testQ                         = self.model.qlim(:,1) + i* stepsQ;
                  qArr{i+1}                     = self.model.ikcon(transform,testQ');
                  poseArr{i+1}                  = self.FKine(qArr{i+1});
                  [rotMat{i+1},transMat{i+1}]   = tr2rt(poseArr{i+1});
                  errRotMat{i+1}                = Rd * (rotMat{i+1})';
                  errRot{i+1}                   = norm(errRotMat{i+1} - eye(3));
                  errTransl{i+1}                = norm (transMat{i+1} - Td);
                  errTot{i+1}                   = errRot{i+1} * errTransl{i+1};                   
              end
              [minErr,errIndex] = min([errTot{:}]);
              q_ = qArr{errIndex}; 
        end
        %% Reset the robot
        function Reset(self)
               poseNew = self.FKine(self.qz);
               if self.model.getpos() ~= self.qz
                   self.Animate(poseNew,50);
               else
                   self.model.animate(self.qz);
               end
        end
        
        %% Trajectory using Quintic Polynomial 
        function Animate(self,pose,steps,object)
            self.qMatrix = [];
            isCollision =false;
            if(nargin>3)
              qNew = self.IKine(pose);
              self.qMatrix = jtraj(self.model.getpos, qNew,steps);
%               check if the object collide with the trajectory
              for i = 1:1:steps
%                    self.model.animate(self.qMatrix(i,:));
%                    drawnow();
                   result(i) = IsCollision(self,self.qMatrix(i,:),object.f,object.vUpdate,object.faceNormals);
                   if result(i) == 1
                        disp('Intersect');
                        isCollision = true;
                        checkedTillWaypoint = 1;
                   else 
                       disp('not intersect');
                   end 
              end
%               obstacle avoid
              qWaypoints = [self.model.getpos;qNew];
              while (isCollision)
                    self.qMatrix=[];
                    startWaypoint = checkedTillWaypoint;
                    for i = startWaypoint:size(qWaypoints,1)-1
                        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
                        if ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                            self.qMatrix = [self.qMatrix; qMatrixJoin];
                            isCollision = false;
                            checkedTillWaypoint = i+1;
                            % Now try and join to the final goal (q2)
                            qMatrixJoin = InterpolateWaypointRadians([self.qMatrix(end,:); qNew],deg2rad(10));
                            if ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                                self.qMatrix = [self.qMatrix;qMatrixJoin];
                                % Reached goal without collision, so break out
                                break;
                            end
                        else
                            % Randomly pick a pose that is not in collision
                %             qRand = (2 * rand(1,6) - 1) * pi;
                            a=eye(4);
                            a(1:3,4) = self.endEffector(1:3,4);
                            qRand = self.IKine(a*transl(0,0,0.15));
                            while ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                %                 qRand = (2 * rand(1,6) - 1) * pi;
%                                 a(1:3,4) = self.endEffector(1:3,4);
                                qRand = self.IKine(a*transl(0,0,0.15));
                            end
                            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
                            isCollision = true;
                            break;
                        end
                    end
              end
%             Move the arm prior to qMatrix
              self.Plot(self.qMatrix);
            else
                qNew = self.IKine(pose);
                self.qMatrix = jtraj(self.model.getpos, qNew,steps);
                for i = 1:1:steps
                    self.model.animate(self.qMatrix(i,:));
                    drawnow();
                    pause(0.05);
                end
            end
        end
        %% Collision Detection
        function Plot(self,qMatrix)
            [row,col] = size(qMatrix);
            for i=1:1:row
                    self.model.animate(qMatrix(i,:));
                    pause(0.03);
            end
        end



    end
end