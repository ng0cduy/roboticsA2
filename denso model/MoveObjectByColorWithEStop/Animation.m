%function for animating with or without goods, depends on 
%number of arguments
function Animation(robotObj, q, goodsObj)
    robotObj.model.delay = 0;       % to optimise animation
    
    if nargin == 2        % without goods
         robotObj.model.animate(q);
         drawnow();
        
    elseif nargin == 3   % with goods            
         robotObj.model.animate(q);
         ee_pose = robotObj.FKine(robotObj.model.getpos);
         goodsObj.Move(ee_pose*troty(pi));

    end
end

