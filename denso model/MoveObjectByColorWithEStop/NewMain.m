function NewMain(pickUpRobot,dropOffRobot,goodsArray,guiObj)
     pickUpRobot.Reset();
     dropOffRobot.Reset();
    %% coming to objects
    %% define parameters
    for i=1:3
        goods{i} = goodsArray{i}; %temporary
    end
    pose = goods{3}.pos_*transl(0,0,-0.06);

    qMatrix = pickUpRobot.qMatrix_gen('rmrc',pose,70);
    pickUpRobot.Plot(qMatrix);
            

    %% delivering objects
    %% define parameters
    conveyor_pos = transl(0.8,-0.1,0.4)*troty(pi); 
  
    %% animation

     qMatrix=pickUpRobot.qMatrix_gen('rmrc',conveyor_pos,70);
     pickUpRobot.Plot(qMatrix,goods{3});

 
     for i=0.8:-0.3:-0.5
         goods{3}.Move(transl(i,-0.1,0.3));
%          pause(0.05);
     end
     
     b=VisServo(dropOffRobot,goods{3});
     
%      camview = EEcam(dropOffRobot);
% 
%      pose = goods{2}.pos_*transl(0,0,-0.06);
%      qMatrix = pickUpRobot.qMatrix_gen('rmrc',pose,70);
%      pickUpRobot.Plot(qMatrix);
%      qMatrix=pickUpRobot.qMatrix_gen('rmrc',conveyor_pos,70);
%      pickUpRobot.Plot(qMatrix,goods{2});
%      
%      for i=0.8:-0.01:-0.5
%          goods{2}.Move(transl(i,-0.1,0.3));
%          pause(0.05);
%      end

end

