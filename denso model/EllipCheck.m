classdef EllipCheck<handle
    properties (Constant)
        qz = zeros(1,6);
    end
    properties(Access = public)
        centerPoint = [0,0,0];
        radii = [0.1,0.1,0.2];
    end
    methods (Access = public)
         function self = EllipCheck(robot,cube,qMatrix)
                cubePoints = cube.vUpdate;
                cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
                tr = robot.model.fkine(qMatrix);
                cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedCubePoints, self.centerPoint, self.radii);
                pointsInside = find(algebraicDist < 1);
                display([' There are now ', num2str(size(pointsInside,1)),' points inside']);
                % % 2.10
                % q = [0,0,0,0,0,0]
                tr = zeros(4,4,robot.model.n+1);
                tr(:,:,1) = robot.model.base;
                L = robot.model.links;
                for i = 1 : robot.model.n
                    tr(:,:,i+1) = tr(:,:,i) * trotz(qMatrix(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
                end

                % Go through each ellipsoid
                for i = 1: size(tr,3)
                    cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
                    updatedCubePoints = cubePointsAndOnes(:,1:3);
                    algebraicDist = GetAlgebraicDist(updatedCubePoints, self.centerPoint, self.radii);
                    pointsInside = find(algebraicDist < 1);
                    display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
                end

         end
    end
end