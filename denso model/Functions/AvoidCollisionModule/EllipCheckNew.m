%% Collision detection given a qMatrix, using ellipsoid collision detection
function isCollision = EllipCheckNew(robot,obj,qMatrix,option,obsPoints)
     % option = 'goods' means self collision detection when carry the goods
     % option = 'obs' means collision-with-environment detection when carry the goods
     % option = 'return' means collision-with-environment detection when
     % return to the initial position

    %% number of arguments = 4 means it uses the point cloud created within the function 
    % otherwise the point cloud will be passed into the input arguments    
    if nargin == 4
        obsPoints = false; % for goods
    else
        cubePoints = obsPoints; % others
    end
    
    % parameters for ellipsoid
    centerPoint = [0 0 0]; 
    radiiLarge = [0.25,0.25,0.18];      % use for option 'obs'
    radiiSmall = [0.13,0.2,0.13];       % use for other options
    isCollision = 0;
    
    % Go through each set of q
    for qIndex = 1:size(qMatrix,1)
        qSet = qMatrix(qIndex,:);
        
        % imagine the goods is attached to the end-effector
        if strcmp(option,'goods') 
            cubePoints = obj.CreateMesh('AtOrigin');
            goodsTr = robot.model.fkine(qSet)*troty(pi)*transl(0,0,-0.07);
            centre = goodsTr(1:3,4)';
            cubePoints = cubePoints + repmat(centre,size(cubePoints,1),1); % move the cube to the required location 
        end

        % Creating transform for each link
        tr = GetLinkPoses(qSet,robot);

        % Go through each ellipsoid
        % Only turn on the first four ellipsoids for self-collision detection 
        if strcmp(option,'goods') 
            for i = 1: (size(tr,3)-3)
                cubePointsAndOnes = (tr(:,:,i) \ [cubePoints,ones(size(cubePoints,1),1)]')';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radiiSmall);
                isCollision = ~isempty(find((algebraicDist < 1),1));
                if isCollision == 1
                    return
                end
            end
        end
        
        % Only turn on the last 2 ellipsoids for collision-with-environment
        % detection, assume that the goods is part of the end-effector. 
        if strcmp(option,'obs')
            for i = (size(tr,3)-1):(size(tr,3))
                cubePointsAndOnes = ((tr(:,:,i)) \ [cubePoints,ones(size(cubePoints,1),1)]')';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radiiLarge);
                isCollision = ~isempty(find((algebraicDist < 1),1));
                if isCollision == 1
                    return
                end
            end

        end
        
        % only turn on some of the last ellipsoids for collision-with-environment
        % detection when return to the initial pose. This is for optimisation.
        if strcmp(option,'return')
            for i = (size(tr,3)-4):(size(tr,3)-1)
                cubePointsAndOnes = ((tr(:,:,i)) \ [cubePoints,ones(size(cubePoints,1),1)]')';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radiiSmall);
                isCollision = ~isempty(find((algebraicDist < 1),1));
                if isCollision == 1
                    return
                end
            end

        end
    end
 end
    
