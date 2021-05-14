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
        qMatrix=[];
        useGripper = false;   
        name;
        initialPose=zeros(1,6);
        pose;
        workspace = [-4.5 4.5 -4.5 4.5 -0.5 2]; 
        verts;
        line_h;
        
        Estop_state;
        lcState = false;
        eStopState = 0;
        resumeState = 0;

        
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
               if sum(self.model.getpos() ~= self.qz) ~= 0
                   self.qMatrix_gen('jtraj',poseNew,50);
                   self.Plot(self.qMatrix);
%                    drawnow();
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
%                         disp(['Reach to singularities at q[' num2str(i),']','Adjusting lambda']);
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
%                 qdot = J\xdot;
                qMatrixx(i+1,:) = qMatrixx(i,:) + deltaTime* qdot(i,:);
            end
            qMatrix = qMatrixx;
            positionError(:,i) = x(1:3,i+1) - T(1:3,4);
            angleError(:,i) = deltaTheta;
        end
        
        %% Trajectory generator
        function qMatrix = qMatrix_gen(self,option,pose,steps,obstacle1,obstacle2)
            self.qMatrix = [];
            self.isCollision =false;
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
              self.qMatrix = Check_Collision(self,qNew,obstacle1);
              disp(5);
            elseif(nargin==6)
                qM = Check_Collision(self,qNew,obstacle1);
                self.qMatrix = Check_Collision(self,qM,obstacle2);

%             Move the arm prior to qMatrix
%               Only move the arm without checking the collision
%             else
%                 self.qMatrix = jtraj(self.model.getpos, qNew,steps);
            end
%             self.Plot(self.qMatrix);
            qMatrix=self.qMatrix;
            
        end
        %% Check eStopState 
        function checkEStop(self)
            while self.eStopState == 1 || self.eStopState == 2
                pause(0.05)
                if self.eStopState ==0 
                    break;
                end
            end 
        end
        %% Collision Detection
        function Plot(self,qMatrix,object)
            [row,col] = size(qMatrix);
%             if self.lcState == true
%                 ob=Obstacle('UFO.ply',transl(4,0,1));
%             end
            for i=1:1:row
                    self.checkEStop();
%                     self.Lidar(self,self.qMatrix(i,:));
                    self.model.animate(qMatrix(i,:));
%                     drawnow();
                    pause(0.005);                    
                    if(nargin==3)
                        self.checkEStop();
                        self.FKine(self.qMatrix(i,:));
                        object.pos_ = self.endEffector*troty(pi)*transl(0,0,-0.07);
                        object.Move(object.pos_);
                        pause(0.01);  
                    end
            end
        end
        %% Check collision Function
        function qMatrix = Check_Collision(self,q,object)
            qM=[];
            for i = 1:1:self.steps                   
                   temp = self.model.fkine(self.qMatrix(i,:));
                   pose_ = temp*transl(0,0,0.2);
                   qM(i,:)=self.IKine(pose_);
            end
%             keyboard;
            for i = 1:1:self.steps
                   result= IsCollision(self,qM(i,:),object.f,object.vUpdate,object.faceNormals);
                   %               Check if there is any collision with all joins
%                    r = EllipCheck(self,object,self.qMatrix(i,:));
                   if (result >= 1) 
                        disp('Intersect');
                        self.isCollision = true;
                        checkedTillWaypoint = 1;
                        break;
                   else 
                       disp('not intersect');
                       
                   end 
            end
            if self.isCollision == false
                qMatrix=self.qMatrix;
            else
%               obstacle avoid
              tempT = self.model.fkine(self.model.getpos);
              poseT = tempT*transl(0,0,-0.15);
%               qNew = self.IKine(poseT)
%               qT=self.GenerateRMRC(poseT,30);
              qT=self.IKine(poseT);
%               qT=[1.5706   0.0069911     0.10991  2.9611e-05    -0.11687 -0.00023068];
              qWaypoints = [self.model.getpos;qT;q];
              while (self.isCollision)
                    qM=[];
                    startWaypoint = checkedTillWaypoint;
                    for i = startWaypoint:1:size(qWaypoints,1)-1
                        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
                        if ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                            qM = [qM; qMatrixJoin];
                            self.isCollision = false;
                            checkedTillWaypoint = i+1;
                            % Now try and join to the final goal (q2)
                            qMatrixJoin = InterpolateWaypointRadians([qM(end,:); q],deg2rad(10));
                            if ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                                qM = [qM;qMatrixJoin];
                                qMatrix = qM;
                                % Reached goal without collision, so break out
                                break;
                            end
                        else
                            % Randomly pick a pose that is not in collision
                            a=eye(4);
                            temp_=self.FKine(qWaypoints(i,:));
                            a(1:3,4) = temp_(1:3,4);
                            qRand = self.IKine(a*transl(-0.05,0,0.1));
                            while ~IsCollision(self,qMatrixJoin,object.f,object.vUpdate,object.faceNormals)
                                temp_=self.FKine(qWaypoints(i,:));
                                a(1:3,4) = temp_(1:3,4);
                                qRand = self.IKine(a*transl(-0.05,0,0.1));
                            end
                            qWaypoints =[qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
%                             qMatrix = [qT;qWaypoints];
                            break;
                        end
                    end
              end
              
            end
        end


        %% 
        function Lidar(self,robot,qMatrix)
            pNormal = [-0.3090, 0.9511, 0];            % Create questions
            pPoint = [0,0.2,0];                          % Create questions
            tr =[];
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
            pause(0.005);
            try delete(w); end; 
%             try delete(m); end;
        end  
           
%% Check collision Function using ellipson
        function qMatrix = Check_Collision1(self,qGoal,goods,obstacle)          
               obsPoints = obstacle.CreateMesh(false);
%                plot3(obsPoints(:,1),obsPoints(:,2),obsPoints(:,3),'cyan*');
               
               qStart = self.model.getpos;
               firstStepTr = self.model.fkine(qStart)*transl(0,0,-0.1);
               qFirstStep = self.IKine(firstStepTr);
               qWaypoints = [qStart;qFirstStep];
               stepTr = firstStepTr;
               qStep = qFirstStep;
               planning = true;
               Mask=[1,1,1,0,0,0];
               while(planning)
                   % turn joint 1 5 deg when there isn't collision. If there is,go up 
                    tempQStep = [qStep(1)+deg2rad(5), qStep(2:end)];
                    tempStepTr = stepTr*transl(-0.1,0,0);
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
                        stepTr = stepTr*transl(0,0.05,0);
                        qStep = self.model.ikine(stepTr,qStep,Mask);
                    end
                    

                    qMatrixEnd = InterpolateWaypointRMRC(self,[qStep; qGoal],50);
                    if ~((EllipCheckNew(self,goods,qMatrixEnd,'goods')) || (EllipCheckNew(self,obstacle,qMatrixEnd,'obs',obsPoints)))
                        % Reached goal without collision, so break out
                        break;
                    end
                    qMatrixEnd = [];
                    if (sum(qStep' < self.model.qlim(:,1)) ~= 0) || (sum(qStep' > self.model.qlim(:,2)) ~= 0)
                        disp('cannot avoid the obstacle');
                        qMatrix = [];
                        return;
                    end
               end
              qMatrixStart = InterpolateWaypointRadians(qWaypoints,deg2rad(10));
              self.qMatrix = [qMatrixStart; InterpolateWaypointRadians([qStep; qGoal],deg2rad(10))];
              qMatrix = self.qMatrix;
        end

    end
end