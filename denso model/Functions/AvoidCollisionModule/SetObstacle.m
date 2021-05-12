% Set Obstacle for gui, put it at a controled random position
function SetObstacle
    temp = rand;        
    obsX = temp/2;   %0 to 0.5
    temp = rand;
    obsY = temp/10 - 0.25;    %-0.25 to -0.15
    obsZ = 0;
    Obstacle(transl(obsX,obsY,obsZ));
end