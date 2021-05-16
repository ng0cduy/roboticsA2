classdef DensoVS060<handle
    properties (Constant)
        qz = zeros(1,6);
    end
    properties(Access =public)
        steps = 50;
        isCollision = false;
        base;
        endEffector;
        model;
        qMatrix=[];
        useGripper = false;   
        name;
        initialPose=zeros(1,6);
        pose;
        workspace = [-4 4 -4 3 -0.5 2]; 
        verts;
        line_h;
        
        Estop_state;
        lcState = false;
        eStopState = 0;
        resumeState = 0;

        
    end
    methods (Access = public) %% Class for DensoVS060 robot simulation
        %% Define robot Function  
        function self = DensoVS060(base, name)
            self.base = base;
            self.name = name;
            self.getDensoVS060();
            self.SetBase(base);
            self.FKine(self.qz);
        end
        %% Create Denso Links
        function getDensoVS060(self)       
            L1=Link('alpha',pi/2,'a',0, 'd',0.148, 'offset',pi,'qlim',[deg2rad(-170), deg2rad(170)] );
            L2=Link('alpha',0,'a',0.305, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]);
            L3=Link('alpha',-pi/2,'a',0, 'd',0, 'offset',0,'qlim',[deg2rad(-125), deg2rad(155)]);
            L4=Link('alpha',pi/2,'a',0, 'd',0.3, 'offset',0,'qlim',[deg2rad(-270), deg2rad(270)]);
            L5=Link('alpha',-pi/2,'a',0, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]);
            L6=Link('alpha',0,'a',0, 'd',0.06, 'offset',0,'qlim',[deg2rad(-360), deg2rad(360)]);           
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',self.name);
        end
        %% Set Denso Base Location 
        function SetBase(self,base)
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
               if sum(self.model.getpos() ~= self.qz) ~= 0
                   self.qMatrix_gen('jtraj',poseNew,50);
                   self.Plot(self.qMatrix);
               else
                   self.model.animate(self.qz);
                   drawnow();
               end
        end
        %% RMRC: q generating using RMRC, checking for singularity
        function qMatrix = GenerateRMRC(self,pose,steps)
            epsilon = 0.01;
            W = diag([1 1 1 0.2 0.2 0.2]);
            q1 = self.model.getpos;
            T1 = self.FKine(q1);
            T2 = pose;
            x1 = [T1(1:3,4);tr2rpy(T1)'];
            x2 = [T2(1:3,4);tr2rpy(T2)'];
            deltaTime = 0.05;
            x = zeros(6,steps);
            s = lspb(0,1,steps);
            m = zeros(steps,1);
            qdot = zeros(steps,6);
            positionError = zeros(3,steps);
            angleError = zeros(3,steps);
            for i = 1:steps
                x(:,i) = (1-s(i))*x1 + s(i)*x2;
            end
            qMatrixx = nan(steps,6);
            qMatrixx(1,:) = q1;
            for i = 1:steps-1
                T = self.FKine(qMatrixx(i,:));
                deltaX = x(1:3,i+1) - T(1:3,4);
                Ra = T(1:3,1:3);
                Rd = rpy2r((x(4:6,i+1))');
                Rdot = (1/deltaTime) *(Rd -Ra);
                S = Rdot*Ra';
                linear_velocity = (1/deltaTime)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];
                deltaTheta = tr2rpy(Rd*Ra');
                xdot  = W*[linear_velocity;angular_velocity];
                J = self.model.jacob0(qMatrixx(i,:));
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon
                        lambda = (1-(m(i)/epsilon))*5E-2;
                else
                        lambda = 0;
                end
                invJ = (J'*J + lambda *eye(6))\J';
                qdot(i,:) = (invJ*xdot)';
                for j = 1:self.model.n
                    if qMatrixx(i,j)+ deltaTime*qdot(i,j) < self.model.qlim(j,1)
                        qdot(i,j) = 0;
                    elseif qMatrixx(i,j)+ deltaTime*qdot(i,j) > self.model.qlim(j,2) 
                        qdot(i,j) = 0;
                    end
                end 
                qMatrixx(i+1,:) = qMatrixx(i,:) + deltaTime* qdot(i,:);
            end
            qMatrix = qMatrixx;
            positionError(:,i) = x(1:3,i+1) - T(1:3,4);
            angleError(:,i) = deltaTheta;
        end
        
        %% Trajectory generator
        function qMatrix = qMatrix_gen(self,option,pose,steps,obstacle)
            self.qMatrix = [];
            self.isCollision =false;
            self.steps=steps;
            qNew = self.IKine(pose);
            if strcmpi(option,'jtraj') == 1
                self.qMatrix = jtraj(self.model.getpos, qNew,steps);
            elseif strcmpi(option,'rmrc') == 1
                self.qMatrix = GenerateRMRC(self,pose,steps);
            elseif strcmpi(option,'trap') ==1
                s = lspb(0,1,steps);
                qMatrixx = nan(steps,6);
                for i = 1:steps
                    qMatrixx(i,:) = (1-s(i))*self.model.getpos + s(i)*qNew;
                end
                self.qMatrix = qMatrixx;
            end
            
            if(nargin==5)       
%               check if the trajectory collide with the table
              self.qMatrix = LinePlane_Collision(self,qNew,obstacle);
              qMatrix=self.qMatrix; 
            end
            qMatrix=self.qMatrix;   
        end
        %% Check eStopState 
        function checkEStop(self)
            while self.eStopState == 1 || self.eStopState == 2
                pause(0.02)
                if self.eStopState ==0 
                    break;
                end
            end 
        end
        %% qMatrix plotting
        function Plot(self,qMatrix,object)
            [row,col] = size(qMatrix);
            for i=1:1:row
                    self.checkEStop();
                    self.model.animate(qMatrix(i,:));
                    pause(0.005);                    
                    if(nargin==3)
                        self.FKine(self.qMatrix(i,:));
                        object.pos_ = self.endEffector*troty(pi)*transl(0,0,-0.08);
                        object.Move(object.pos_);
                        pause(0.03);  
                    end
            end
        end
        %% Check collision using LinePlane intersection
        function qMatrix = LinePlane_Collision(self,q,object)
            qM=[];
            for i = 1:1:self.steps                   
                   temp = self.model.fkine(self.qMatrix(i,:));
                   pose_ = temp*transl(0,0,0.12);
                   if i == 1
                    qM(i,:)=self.model.ikcon(pose_);
                   else
                       qM(i,:)=self.model.ikcon(pose_,qM(i-1,:));
                   end
            end
            for i = 1:1:self.steps
                   result= IsCollision(self,qM(i,:),object.f,object.vUpdate,object.faceNormals);
                   %               Check if there is any collision with all joins
                   if (result >= 1) 
                        disp('Intersect detected');
                        self.isCollision = true;
                        checkedTillWaypoint = 1;
                        break;
                   else 
%                        disp('no Intersection found');
                       
                   end 
            end
            if self.isCollision == false
                 disp('no Intersection found');
                qMatrix=self.qMatrix;
            else
              disp('Finding new path, please wait');
%               obstacle avoid
              qWaypoints = [self.model.getpos;q];
%               offset the endEffector to go bellow the good
              for i = 1:1:size(qWaypoints,1)
                  temp = self.model.fkine(qWaypoints(i,:));
                  pose_ = temp*transl(0,0,0.12);
                  qW(i,:)=self.model.ikcon(pose_);
              end
              qM=[];
              while (self.isCollision)            
                    startWaypoint = checkedTillWaypoint;
                    for i = startWaypoint:1:size(qW,1)-1
                        qMatrixJoin = InterpolateWaypointRadians(qW(i:i+1,:),deg2rad(5));
                        if ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                            qM = [qM; qMatrixJoin];
                            self.isCollision = false;
                            checkedTillWaypoint = i+1;
                            % Now try and join to the final goal (q2)
                            qMatrixJoin = InterpolateWaypointRadians([qM(end,:); q],deg2rad(5));
                            if ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                                qM = [qM;qMatrixJoin];
                                qMatrix = qM;
                                % Reached goal without collision, so break out
                                break;
                            end
%                             break out if the qwaypoints is out of join
%                             limits
                        elseif (sum(qWaypoints(i,:)' < self.model.qlim(:,1)) ~= 0) || (sum(qWaypoints(i,:)' > self.model.qlim(:,2)) ~= 0)
                            disp('cannot avoid obstacle');
                            qMatrix = [];
                            self.eStopState = 2; 
                            return;
                        else
                            % Randomly pick a pose that is not in collision
                            temp_=self.FKine(qWaypoints(i,:));
                            qRand = self.IKine(temp_*transl(0,0,-0.4));
                            while ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                                temp_=self.FKine(qW(i,:));
                                qRand = self.IKine(temp_*transl(0,0,-0.4));
                            end
                            %join the initial path with the new waypoints
                            qW =[qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
                            break;
                        end
                    end
              end
              
            end
        end


        %% Create a lidar scan at the endEffector
        function Lidar(self,robot,qMatrix)
            pNormal = [-0.3090, 0.9511, 0];            
            pPoint = [0,0.2,0];                          
            tr =[];
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
                endP = tr(1:3,4)' + 3 * tr(1:3,3)';                             
                intersectP = LinePlaneIntersection(pNormal,pPoint,startP,endP);  
                dist = dist2pts(startP,intersectP)  ;
                endP = tr(1:3,4)' + dist * tr(1:3,3)';

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

                % Transform the points on the default plane, to matches the actual triangle
                points = [X(:),Y(:),Z(:)] * tr(1:3,1:3) + repmat(trianglePoint,prod(sizeMat),1);
                X = reshape(points(:,1),sizeMat(1),sizeMat(2));
                Y = reshape(points(:,2),sizeMat(1),sizeMat(2));
                Z = reshape(points(:,3),sizeMat(1),sizeMat(2));

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
            pause(0.005);
            try delete(w); end; 

        end  
           
%% Function to generate qMatrix, avoid collision using ellipsoid method
% For the goods to avoid collision and get to a safe waypoint, before going
% to its designated destination.
        function qMatrix = EllipsoidQGen(self,goalTr,goods,obstacle)          
           % Create mesh points for function checking collision
           obsPoints = obstacle.CreateMesh(false);

           % Define parameters
           qStart = self.model.getpos;
           qGoal = self.IKine(goalTr);
           planning = true;
           Mask=[1,1,1,0,0,0];

           % first step is going up 0.1 m
           firstStepTr = self.model.fkine(qStart)*transl(0,0,-0.1);
           qFirstStep = self.IKine(firstStepTr);
           qWaypoints = [qStart;qFirstStep];
           stepTr = firstStepTr;
           qStep = qFirstStep;

           % Create a safe waypoint be4GoalTr that under the height of the obstacle 
           ObsTr = obstacle.pos_;
           be4GoalZ = ObsTr(3,4);
           be4GoalY = 0.55;

           be4GoalLastCol = [ goalTr(1,4); be4GoalY; be4GoalZ;  1 ];
           be4GoalTr = [goalTr(:,1:3), be4GoalLastCol];
           qBe4Goal = self.IKine(be4GoalTr);

           while(planning)
               % turn joint 1 5 deg to approach destination when there isn't collision.
               % If there is, go up 
                tempQStep = [qStep(1)+deg2rad(5), qStep(2:end)];
                tempStepTr = self.model.fkine(tempQStep);
                if EllipCheckNew(self,obstacle,tempQStep,'obs',obsPoints)
                    stepTr = stepTr*transl(0,0,-0.1);
                    qStep = self.model.ikine(stepTr,qStep,Mask);
                else
                    stepTr = tempStepTr;
                    qStep = tempQStep; 
                end
                qWaypoints = [qWaypoints;qStep];
              % go up when collision with obstacle
                while EllipCheckNew(self,obstacle,qStep,'obs',obsPoints)
                    stepTr = stepTr*transl(0,0,-0.05);
                    qStep = self.model.ikine(stepTr,qStep,Mask);
                end

              % lean forward when collision with goods
                while EllipCheckNew(self,goods,qStep,'goods')
                    stepTr = stepTr*transl(0,-0.05,0);
                    qStep = self.model.ikine(stepTr,qStep,Mask);
                end


                qMatrixBe4End = InterpolateWaypointRMRC(self,[qStep; qBe4Goal],50);
                if ~((EllipCheckNew(self,goods,qMatrixBe4End,'goods')) || (EllipCheckNew(self,obstacle,qMatrixBe4End,'obs',obsPoints)))
                    % Reached be4goal waypoint without collision, so break out
                    break;
                end
                qMatrixBe4End = [];
                if (sum(qStep' < self.model.qlim(:,1)) ~= 0) || (sum(qStep' > self.model.qlim(:,2)) ~= 0)
                    disp('cannot avoid the obstacle');
                    qMatrix = [];
                    self.eStopState = 2; % stop the system if it cannot avoid the obstacle.
                    return;
                end
           end
           
          % Join all the waypoints
          qMatrixStart = InterpolateWaypointRadians(qWaypoints,deg2rad(10));
          qMatrix = [qMatrixStart;qMatrixBe4End];
          qMatrix = [qMatrix; InterpolateWaypointRadians([qBe4Goal;qGoal],deg2rad(5))];
          self.qMatrix = qMatrix;

        end
%% Function to generate qMatrix, avoid collision when return to its initial pose
        function qMatrix = ElipsoidResetQgen(self,obstacle)
            % define parameters
            qStart = self.model.getpos;
            qGoal = self.qz;
            
            obsPoints = obstacle.CreateMesh(false); 
            ObsTr = obstacle.pos_;
            
            % first step that under the obstacle, so there's no collision
            % to check
            firstStepX = 1.25;
            firstStepY = 0.35;
            firstStepZ = ObsTr(3,4) - 0.02;
            
            firstStepTransl = [ firstStepX, firstStepY, firstStepZ ];
            firstStepTr = transl(firstStepTransl)*troty(pi);
            
            qFirstStep = self.IKine(firstStepTr);
            qWaypoints = [qStart;qFirstStep];
            stepTr = firstStepTr;
            qStep = qFirstStep;
            planning = true;
            
            while(planning)
                   % turn joint 1 5 deg when there isn't collision. If there is,go up 
                    tempQStep = [qStep(1)-deg2rad(20), qStep(2:end)];
                    tempStepTr = self.model.fkine(tempQStep);
                    if EllipCheckNew(self,obstacle,tempQStep,'return',obsPoints)
                        stepTr = stepTr*transl(0,0,-0.1);
                        qStep = self.model.ikcon(stepTr,qStep);
                    else
                        stepTr = tempStepTr;
                        qStep = tempQStep; 
                    end
                    qWaypoints = [qWaypoints;qStep];                  

                    qMatrixEnd = InterpolateWaypointRadians([qStep; qGoal],deg2rad(5));
                    
                    if ~EllipCheckNew(self,obstacle,qMatrixEnd,'return',obsPoints)
                        % Reached goal way point without collision, so break out
                        break;
                    end
                    qMatrixEnd = [];
            end
               
            qMatrixStart = InterpolateWaypointRadians(qWaypoints,deg2rad(1));
            qMatrix = [qMatrixStart;qMatrixEnd];
            self.qMatrix = qMatrix;

        end
    end
end