classdef DensoVS060<handle
    properties (Constant)
        qz = zeros(1,6);
    end
    properties(Access =public)
        steps = 50;
        isCollision = false;
%                         checkedTillWaypoint = 1;
        base;
        endEffector;
        model;
        qMatrix;
        eStop = 0;
        useGripper = false;   
        name;
        initialPose=zeros(1,6);
        pose;
        workspace = [-1 1 -1 1 -1 1]; 
        verts;
        line_h;
        
    end
    methods (Access = public) %% Class for DensoVS060 robot simulation
        %% Define robot Function  
        function self = DensoVS060(useGripper, base, name)
            self.useGripper = useGripper;  
            self.base = base;
            self.name = name;
            self.getDensoVS060();
            self.SetBase(base);
            self.FKine(self.qz);
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
            self.model.base = base * transl(0,0,0.202);
            base = self.model.base;
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
                   self.Animate('jtraj',poseNew,50);
               else
                   self.model.animate(self.qz);
               end
        end
        %% RMRC: q generating using RMRC
        function qMatrix = GenerateRMRC(self,pose,steps)
            q1 = self.model.getpos;
%             q2 = self.IKine(pose);
            T1 = self.FKine(q1);
            T2 = pose;
            x1 = [T1(1:3,4);tr2rpy(T1)'];
            x2 = [T2(1:3,4);tr2rpy(T2)'];
            deltaTime = 0.05;
            x = zeros(6,steps);
            s = lspb(0,1,steps);
            for i = 1:steps
                x(:,i) = x1*(1-s(i)) + s(i)*x2;
            end
            qMatrixx = nan(steps,6);
            qMatrixx(1,:) = q1;
            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaTime;
                J = self.model.jacob0(qMatrixx(i,:));
                qdot = J\xdot;
                qMatrixx(i+1,:) = qMatrixx(i,:) + deltaTime* qdot';
            end
            qMatrix = qMatrixx;
        end
        
        %% Trajectory using Quintic Polynomial 
        function Animate(self,option,pose,steps,table,object)
            self.qMatrix = [];
            self.isCollision =false;
            qNew = self.IKine(pose);
            if strcmpi(option,'jtraj') == 1
                self.qMatrix = jtraj(self.model.getpos, qNew,steps);
            elseif strcmpi(option,'rmrc') == 1
                 self.qMatrix = GenerateRMRC(self,pose,steps);
            end
            
            
            if(nargin==5)       
%               check if the trajectory collide with the table
              self.qMatrix = Check_Collision(self,qNew,table);
            end
            if(nargin>5)
%               check if the object collide with the trajectory
              self.qMatrix = Check_Collision(self,qNew,object);

%             Move the arm prior to qMatrix
              self.Plot(self.qMatrix);
%               Only move the arm without checking the collision
            else
                self.qMatrix = jtraj(self.model.getpos, qNew,steps);
%                 self.qMatrix = Check_Collision(self,qNew,table);
                self.Plot(self.qMatrix);
            end
        end
        %% Collision Detection
        function Plot(self,qMatrix)
            [row,col] = size(qMatrix);
            for i=1:1:row
                    Lidar(self,self.qMatrix(i,:));
                    self.model.animate(qMatrix(i,:));
                    pause(0.03);
            end
        end
        %% Check collision Function
        function qMatrix = Check_Collision(self,q,object)
            for i = 1:1:self.steps
                   
                   result= IsCollision(self,self.qMatrix(i,:),object.f,object.vUpdate,object.faceNormals);
                   %               Check if there is any collision with all joins
                   r = EllipCheck(self,object,self.qMatrix(i,:));
                   if (result >= 1 | r.result >= 1) 
                        disp('Intersect');
                        self.isCollision = true;
                        checkedTillWaypoint = 1;
                   else 
                       disp('not intersect');
                   end 
            end
%               obstacle avoid
              qWaypoints = [self.model.getpos;q];
              while (self.isCollision)
                    self.qMatrix=[];
                    startWaypoint = checkedTillWaypoint;
                    for i = startWaypoint:1:size(qWaypoints,1)-1
                        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
                        if ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                            self.qMatrix = [self.qMatrix; qMatrixJoin];
                            self.isCollision = false;
                            checkedTillWaypoint = i+1;
                            % Now try and join to the final goal (q2)
                            qMatrixJoin = InterpolateWaypointRadians([self.qMatrix(end,:); q],deg2rad(5));
                            if ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                                self.qMatrix = [self.qMatrix;qMatrixJoin];
                                % Reached goal without collision, so break out
                                break;
                            end
                        else
                            % Randomly pick a pose that is not in collision
                            a=eye(4);
                            a(1:2,4) = self.endEffector(1:2,4);
                            qRand = self.IKine(a*transl(.05,-.05,object.z+0.2)*troty(pi));
%                             while ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
%                                 qRand = self.IKine(a*transl(-.05,0.05,object.z+0.15)*troty(pi));
%                             end
                            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
                            break;
                        end
                    end
              end
              qMatrix = self.qMatrix;
        end

        %% Toggle estop to stop or continue doing task
        function eStop = eStopToggle(self)
            eStop = ~self.eStop;
            self.eStop = eStop;
        end

        %% 
        function Lidar(robot,qMatrix)
            pNormal = [-0.3090, 0.9511, 0];            % Create questions
            pPoint = [0,0.2,0];                          % Create questions
%             [row,col] = size(qMatrix);
%             line_h = [];
            for i=1:1:3
                if i == 1
                    tr=robot.FKine(robot.model.getpos);
                    robot.model.animate(robot.model.getpos);
                elseif i ==2
                    tr=robot.FKine(robot.model.getpos)*trotx(deg2rad(20));
                    robot.model.animate(robot.model.getpos);
                else
                    tr=robot.FKine(robot.model.getpos)*trotx(deg2rad(-20));
                end
                startP = tr(1:3,4)';
                endP = tr(1:3,4)' + 3 * tr(1:3,3)';                             % Create questions
                intersectP = LinePlaneIntersection(pNormal,pPoint,startP,endP);  % Create questions
                dist = dist2pts(startP,intersectP)  ;
                endP = tr(1:3,4)' + dist * tr(1:3,3)';
%                 self.line_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
%                 m=plot3(endP(1),endP(2),endP(3),'r*');
                if i==1
                    self.verts = endP;
                else
                    self.verts = [self.verts;endP];
                end
                pause(0.001);
            end
                triangleNormal = cross((self.verts(1,:)-self.verts(2,:)),(self.verts(2,:)-self.verts(3,:)));
                triangleNormal = triangleNormal / norm(triangleNormal);

                % Make a plane at the orgin, to rotate later
                basePlaneNormal = [-1,0,0];
                [Y,Z] = meshgrid(-1:0.1:1,-1:0.1:1  );
                sizeMat = size(Y);
                X = repmat(0,sizeMat(1),sizeMat(2));

                % Rotation axis: to rotate the base plane around
                rotationAxis = cross(triangleNormal,basePlaneNormal);
                rotationAxis = rotationAxis / norm(rotationAxis);

                % Rotation angle: how much to rotate base plane to make it match triangle plane
                rotationRadians = acos(dot(triangleNormal,basePlaneNormal));

                % Make a transform to do that rotation
                tr = makehgtform('axisrotate',rotationAxis,rotationRadians);

                % Find a central point of the triangle
                trianglePoint = sum(self.verts)/3;

                % Plot the point/normal of the triangle
%                 plot3(trianglePoint(1),trianglePoint(2),trianglePoint(3),'g*');
%                 plot3([trianglePoint(1),trianglePoint(1)+triangleNormal(1)] ...
%                      ,[trianglePoint(2),trianglePoint(2)+triangleNormal(2)] ...
%                      ,[trianglePoint(3),trianglePoint(3)+triangleNormal(3)],'b');
%                 drawnow();
%                 pause(1);

                % Transform the points on the default plane, to matches the actual triangle
                points = [X(:),Y(:),Z(:)] * tr(1:3,1:3) + repmat(trianglePoint,prod(sizeMat),1);
                X = reshape(points(:,1),sizeMat(1),sizeMat(2));
                Y = reshape(points(:,2),sizeMat(1),sizeMat(2));
                Z = reshape(points(:,3),sizeMat(1),sizeMat(2));

                % Make points where Z<0 to be = zero
%                 Z(Z<0)= 0;
%                 surf(X,Y,Z);
%                 pause(1);

                maxRange = 0.6; % meters

    %             get end-effector point in given pose
                q = qMatrix;
                tr = robot.model.fkine(q);
                robot.model.animate(q);
                startP = tr(1:3,4)';

                % Make a single scan ray as if it were from the origin (for rotating)
                rayAtOrigin = maxRange * tr(1:3,3)';
                xRotAxis = tr(1:3,1)';
                yRotAxis = tr(1:3,2)';

                % Rotate around end effector's xaxis and yaxis, populate scanData
                scanData = [];
                for xRotRads = deg2rad(-20):deg2rad(1):deg2rad(20)
                    for yRotRads = deg2rad(-20):deg2rad(1):deg2rad(20) % Note it is more efficient to make one scan block rather than having an embeded for loop as done here
                        % Make a transform to rotate the scan ray around
                        tr = makehgtform('axisrotate',xRotAxis,xRotRads) * makehgtform('axisrotate',yRotAxis,yRotRads);

                        % Determine location of ray end at max range
                        rayEnd = startP +  rayAtOrigin * tr(1:3,1:3);

                        % Check for intersection with the plane that the triangle is on
                        [intersectP,check] = LinePlaneIntersection(triangleNormal,trianglePoint,startP,rayEnd);        
                        if check == 1
                            rayEnd = intersectP;
                        end

                        % Check for intersection with floor (note: floorNormal = [0,0,1] floorPoint = origin)
                        [intersectWithFloor,check] = LinePlaneIntersection([0,0,1],[0,0,0],startP,rayEnd);
                        if check == 1 && dist2pts(startP,intersectWithFloor) < dist2pts(startP,rayEnd)
                            rayEnd = intersectWithFloor;
                        end

                        % Keep track of the scan data. It is more efficient to initialise
                        % it to the correct size, however for brevity, it is done suboptimally here
                        scanData = [scanData; rayEnd];         %#ok<AGROW>
                    end
%                 end
                end
            w = plot3(scanData(:,1),scanData(:,2),scanData(:,3),'r.');
            pause(0.02);
            try delete(w); end; 
%             try delete(m); end;
        end  
           

    end
end