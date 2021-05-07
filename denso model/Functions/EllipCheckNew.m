 function result = EllipCheckNew(robot,obj,qMatrix,option,obsPoints)
        if nargin == 4
            obsPoints = false;
        else
            cubePoints = obsPoints;
        end
        centerPoint = [0 0 0]; 
        radiiSmall = [0.13,0.2,0.13];
        radiiLarge = [0.25,0.25,0.18];
        result = 0;
        
        for qIndex = 1:size(qMatrix,1)
            qSet = qMatrix(qIndex,:);
            if strcmp(option,'goods') 
                cubePoints = obj.CreateMesh('AtOrigin');
                goodsTr = robot.model.fkine(qSet)*troty(pi)*transl(0,0,-0.07);
                centre = goodsTr(1:3,4)';
                cubePoints = cubePoints + repmat(centre,size(cubePoints,1),1); % move the cube to the required location 
            end
    %         cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'cyan.');


            % Creating ellipsoid for links
%             [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radiiSmall(1), radiiSmall(2), radiiSmall(3) );
%             for i = 1:4 
%                 robot.model.points{i} = [X(:),Y(:),Z(:)];
%                 warning off                                         % there will be unnecessary warnings so turn it off for a moment
%                 robot.model.faces{i} = delaunay(robot.model.points{i});         % given a set of the points. This plot a surface covers all those points. 
%                 warning on;
%             end
%     
%             [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radiiLarge(1), radiiLarge(2), radiiLarge(3) );
%             robot.model.points{6} = [X(:),Y(:),Z(:)];
%             warning off                                         % there will be unnecessary warnings so turn it off for a moment
%             robot.model.faces{6} = delaunay(robot.model.points{6});         % given a set of the points. This plot a surface covers all those points. 
%             warning on;
%             
%             [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radiiLarge(1), radiiLarge(2), radiiLarge(3) );
%             robot.model.points{7} = [X(:),Y(:),Z(:)];
%             warning off                                         % there will be unnecessary warnings so turn it off for a moment
%             robot.model.faces{7} = delaunay(robot.model.points{7});         % given a set of the points. This plot a surface covers all those points. 
%             warning on;
%             robot.model.plot3d(qSet);

            % Creating transform for each link
            tr = GetLinkPoses(qSet,robot);

            % Go through each ellipsoid
            if strcmp(option,'goods') %|| strcmp(option,'obs')
                for i = 1: (size(tr,3)-3)
                    cubePointsAndOnes = (tr(:,:,i) \ [cubePoints,ones(size(cubePoints,1),1)]')';
                    updatedCubePoints = cubePointsAndOnes(:,1:3);
                    algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radiiSmall);
                    result = ~isempty(find((algebraicDist < 1),1));
    %                 if result == 1
    %                     Indices = zeros(5,1);
    %                     Indices = find((algebraicDist < 1),5);
    %                     for j = Indices
    %                        plot3(cubePoints(j,1),cubePoints(j,2),cubePoints(j,3),'r*'); 
    %                     end
    %                 end
                    if result == 1
                        return
                    end
                end
            end

            if strcmp(option,'obs')
                for i = 6:7
                    cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
                    updatedCubePoints = cubePointsAndOnes(:,1:3);
                    if i == 7
                        algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radiiLarge);
                    else
                        algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radiiLarge);
                    end
                    result = ~isempty(find((algebraicDist < 1),1));
                    if result == 1
                        return
                    end
                end
%                 cubePointsAndOnes = (tr(:,:,7) \ [cubePoints,ones(size(cubePoints,1),1)]')';
%                 updatedCubePoints = cubePointsAndOnes(:,1:3);
%                 algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radiiLarge);
%                 result = ~isempty(find((algebraicDist < 1),1));
%                 if result == 1
%                     return
%                 end
            end
        end

 end
    
