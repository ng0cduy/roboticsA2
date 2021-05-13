function LCTesting(lightcurtain,denso)

    denso.Reset();
    ob=Obstacle('UFO.ply',transl(4,0,1)*troty(-30,'deg'));
    obCurrentPos = ob.pos_;
    jointStates = denso.model.getpos();
    pose = denso.FKine(jointStates);
    pose1 = pose;
    touchFlag =0; % Flag to determine the obstacle touch to the light curtain or not
    moveInFlag = true; % Flag to control the movement of the obstacle
    touchTimes = 0;
    pose1(3,4) = pose(3,4)-0.3; % destination pose of end effector
    qMatrix = denso.GenerateRMRC(pose1,50); % RMRC path
    qLength =length(qMatrix);
    for i = 1:1:qLength
        touchFlag = lightcurtain.DetectObstacle(ob);
        while touchFlag == 1
            touchTimes = touchTimes +1;
            if touchTimes == 1
                pause(4);
            end
            obCurrentPos(1,4) = obCurrentPos(1,4) + 0.01
            obCurrentPos(1:3,1:3) = roty(30,'deg');
            ob.Move(obCurrentPos);
            drawnow();
            touchFlag = lightcurtain.DetectObstacle(ob);
            if touchFlag ==0
                disp('Robot can resume');
                moveInFlag = false;
            end 
        end
        if moveInFlag == true
            obCurrentPos(1,4) = obCurrentPos(1,4) - 0.02;
        else
            obCurrentPos(1,4) = obCurrentPos(1,4) + 0.02;
        end
        ob.Move(obCurrentPos);
        denso.model.animate(qMatrix(i,:));
        drawnow();
    end 
end 