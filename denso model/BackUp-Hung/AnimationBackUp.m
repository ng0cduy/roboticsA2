%function for animating with or without goods, depends on 
%number of arguments
function Animation(steps, robotObj, goodsObj, order)
    robotObj.model.delay = 0;       % to optimise animation
    
    if nargin == 3        % without goods
        pose = goodsObj.pos_;
        qNew = robotObj.IKine(pose);
        robotObj.qMatrix = jtraj(robotObj.model.getpos, qNew,steps);
        for i = 1:steps
            robotObj.model.animate(robotObj.qMatrix(i,:));
            drawnow();
            pause(0.05);
        end
        
    elseif 3 < nargin   % with goods        
        pose = [eye(3), (GetGoodsDes(goodsObj,order))';ones(1,4)]*troty(pi);
        qNew = robotObj.IKine(pose);
        robotObj.qMatrix = jtraj(robotObj.model.getpos, qNew,steps);
        
        for i = 1:steps
            robotObj.model.animate(robotObj.qMatrix(i,:));
            ee_pose = robotObj.FKine(robotObj.model.getpos);
            goodsObj.MoveObject(ee_pose*troty(pi));
            drawnow();

            pause(0.05);
        end
    end
end

