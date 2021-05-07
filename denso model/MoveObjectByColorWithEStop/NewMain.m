function NewMain(pickUpRobot,dropOffRobot,goodsArray,guiobj,conveyor)
     clc;
     conveyor = goods('conveyor.ply',transl(-0.1,0,0.2));
     ob=Obstacle(transl(1.15,-0.2,0.35));
     pickUpRobot.Reset();
     dropOffRobot.Reset();
    %% coming to objects
    %% define parameters
    for i=3:-1:1
    good{i} = goodsArray{i}; %temporary
    pose = good{i}.pos_*transl(0,0,-0.06);

    qMatrix = pickUpRobot.qMatrix_gen('jtraj',pose,40);
    pickUpRobot.Plot(qMatrix);
    

    % delivering objects
    %% define parameters
    conveyor_pos = transl(-0.5,-0.08,0.4)*troty(pi);
    redOrder = 0;
    blueOrder = 0;
    greenOrder = 0;
  
    %% animation

     qMatrix=pickUpRobot.qMatrix_gen('jtraj',conveyor_pos,40);
     pickUpRobot.Plot(qMatrix,good{i});
     pickUpRobot.Reset();
     
    
     
     Move_conveyor(good{i});   
     camview{i} = EEcam(dropOffRobot);
     disp(camview{i}.color);
     view(30,0);
     pause(1);
     b=VisServo(dropOffRobot,good{i});
     
     good{i}.color = camview{i}.color;       % Set color of goods by camview
%                                             At the moment, use name
%      disp(good{i}.color);
     pause(1);
     qMatrix=dropOffRobot.qMatrix_gen('jtraj',b.object_pose*troty(pi),80);
     dropOffRobot.Plot(qMatrix);
     
     % Identify the order of the goods kind we will deliver 
     switch good{i}.color
         case 'red'
             redOrder = redOrder + 1;
             goodsOrder = redOrder;
         case 'blue'
             blueOrder = blueOrder + 1;
             goodsOrder = blueOrder;
         case 'green'
             greenOrder = greenOrder + 1;
             goodsOrder = greenOrder; 
         otherwise
             disp('the color is not in the default set of color')
             goodsOrder = -1;
     end
     
     % deliver the goods
     goodsTr = GetGoodsDes(good{i},goodsOrder); 
     qGoal = dropOffRobot.IKine(goodsTr);
%      pose=transl(0.5,0.55,0.1)*troty(pi);
     qMatrix=dropOffRobot.Check_Collision(qGoal,good{i},ob);
     dropOffRobot.Plot(qMatrix,good{i});
     
     dropOffRobot.Reset();
    end

end