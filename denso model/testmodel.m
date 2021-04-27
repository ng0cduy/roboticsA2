
%%
clf
clear all
clc
a = DensoVS060(false,transl(0,0,0),'denso');
% a.model.teach;
hold on;
green= goods('green',transl(-0.4,0,0)*troty(pi));
%%
a.Animate(transl(-0.5,0.2,0)*troty(pi),50,green);
a.Animate(transl(-0.5,-0.4,0)*troty(pi),50,green);
% % 
% a.Animate(green.pos_,50,green);
% a.Reset()
% %% test collision
% keyboard;
% clc;
% % mdl_planar3
% [vertex,faces,faceNormals] =RectangularPrism([0.8,-.11,-.5], [.2,.11,.5]);
% axis equal
% camlight
% q1 = [-pi/3,0,0,0,0,0];
% q2 =  [pi/3,0,0,0,0,0];
% % keyboard;
% a.model.animate(q1)
% steps = 50;
% qMatrix = jtraj(q1,q2,steps);
% 
% for i= 1:1:steps
%     result(i) = IsCollision(a,qMatrix(i,:),faces,vertex,faceNormals);
%     if result(i) >= 1
%         isCollision = true;
%         checkedTillWaypoint = 1;
%         break
%     end
% %     a.model.animate(qMatrix(i,:));
% end
% 
% %% 
% clc;
% % close all;
% % clear all;
% % a = DensoVS060(false,transl(0,0,0),'denso');
% [vertex,faces,faceNormals] =RectangularPrism([0.8,-.11,-.5], [.2,.11,.5]);
% q1 = [-pi/3,0,0,0,0,0];
% q2 =  [pi/3,0,0,0,0,0];
% qWaypoints = [q1;q2];
% 
% a.model.animate(q1);
% qMatrix = [];
% while (isCollision)
%     startWaypoint = checkedTillWaypoint;
%     for i = startWaypoint:size(qWaypoints,1)-1
%         qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
%         if ~IsCollision(a,qMatrixJoin,faces,vertex,faceNormals)
%             qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
%             isCollision = false;
%             checkedTillWaypoint = i+1;
%             % Now try and join to the final goal (q2)
%             qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
%             if ~IsCollision(a,qMatrixJoin,faces,vertex,faceNormals)
%                 qMatrix = [qMatrix;qMatrixJoin];
%                 % Reached goal without collision, so break out
%                 break;
%             end
%         else
%             % Randomly pick a pose that is not in collision
% %             qRand = (2 * rand(1,6) - 1) * pi;
%             qRand = a.IKine(a.endEffector*transl(0,0,-0.15));
%             while IsCollision(a,qRand,faces,vertex,faceNormals)
% %                 qRand = (2 * rand(1,6) - 1) * pi;
%                 qRand = a.IKine(a.endEffector*transl(0,-0.03,-0.15));
%             end
%             qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
%             isCollision = true;
%             break;
%         end
%     end
% end
% %% 
% keyboard;
% [row,col] = size(qMatrix);
% for i=1:1:row
%     a.model.animate(qMatrix(i,:));
%     pause(0.03);
% end
% 
% 
% %% InterpolateWaypointRadians
% % Given a set of waypoints, finely intepolate them
% function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
% if nargin < 2
%     maxStepRadians = deg2rad(1);
% end
% 
% qMatrix = [];
% for i = 1: size(waypointRadians,1)-1
%     qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
% end
% end
% %% FineInterpolation
% % Use results from Q2.6 to keep calling jtraj until all step sizes are
% % smaller than a given max steps size
% function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
% if nargin < 3
%     maxStepRadians = deg2rad(1);
% end
%     
% steps = 2;
% while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
%     steps = steps + 1;
% end
% qMatrix = jtraj(q1,q2,steps);
% end


