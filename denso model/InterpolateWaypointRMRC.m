%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRMRC(robot,waypointRadians,steps)
        qMatrix = [];
        for i = 1: size(waypointRadians,1)-1
            qMatrix = [qMatrix ; GenRMRC(robot,waypointRadians(i,:),waypointRadians(i+1,:),steps)]; %#ok<AGROW>
        end
end

%% Generate RMRC
 function qMatrix = GenRMRC(robot,q1,q2,steps)
            epsilon = 0.02;
            W = diag([1 1 1 0.2 0.2 0.2]);
            
            T1 = robot.FKine(q1);
            T2 = robot.FKine(q2);
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
                x(:,i) = x1*(1-s(i)) + s(i)*x2;
            end
            qMatrixx = nan(steps,6);
            qMatrixx(1,:) = q1;
            for i = 1:steps-1
                T = robot.FKine(qMatrixx(i,:));
                deltaX = x(1:3,i+1) - T(1:3,4);
                Ra = T(1:3,1:3);
                Rd = rpy2r(x(4:6),i+1);
                Rdot = (1/deltaTime) *(Rd -Ra);
                S = Rdot*Ra';
                linear_velocity = (1/deltaTime)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];
                deltaTheta = tr2rpy(Rd*Ra');
                xdot = (x(:,i+1) - x(:,i))/deltaTime;
                J = robot.model.jacob0(qMatrixx(i,:));
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon
                        lambda = (1-(m(i)/epsilon))*5E-2;
%                         disp(['Reach to singularities at q[' num2str(i),']','Adjusting lambda']);
                else
                        lambda = 0;
                end
                invJ = (J'*J + lambda *eye(6))\J';
                qdot(i,:) = (invJ*xdot)';
                for j = 1:robot.model.n
                    if qMatrixx(i,j)+ deltaTime*qdot(i,j) < robot.model.qlim(j,1)
                        qdot(i,j) = 0;
                    elseif qMatrixx(i,j)+ deltaTime*qdot(i,j) > robot.model.qlim(j,2) 
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