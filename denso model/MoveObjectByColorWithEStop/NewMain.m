function NewMain(pickUpRobot,dropOffRobot,goodsArray,guiObj)
     
     pickUpRobot.Reset();
     dropOffRobot.Reset();
    %% coming to objects
    %% define parameters
    for i=1:3
        good{i} = goodsArray{i}; %temporary
    end
    pose = good{3}.pos_*transl(0,0,-0.06);

    qMatrix = pickUpRobot.qMatrix_gen('jtraj',pose,80);
    pickUpRobot.Plot(qMatrix);
    

    % delivering objects
    %% define parameters
    conveyor_pos = transl(-0.5,-0.08,0.4)*troty(pi); 
  
    %% animation

     qMatrix=pickUpRobot.qMatrix_gen('rmrc',conveyor_pos,80);
     pickUpRobot.Plot(qMatrix,good{3});
     pickUpRobot.Reset();
     Move_conveyor(good{3});      
     b=VisServo(dropOffRobot,good{3});
     camview = EEcam(dropOffRobot);
     
     pause(1);
     qMatrix=dropOffRobot.qMatrix_gen('jtraj',b.object_pose*troty(pi),80);
     dropOffRobot.Plot(qMatrix);
     %% 

% 
%      pose = good{3}.pos_*transl(0,0,-0.06);
%      qMatrix = pickUpRobot.qMatrix_gen('rmrc',pose,70);
%      pickUpRobot.Plot(qMatrix);
%      qMatrix=pickUpRobot.qMatrix_gen('rmrc',conveyor_pos,70);
%      pickUpRobot.Plot(qMatrix,good{3});
%      
%      Move_conveyor(good{3});

end

