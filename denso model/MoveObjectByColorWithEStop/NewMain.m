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
    redOrder = 0;
    blueOrder = 0;
    greenOrder = 0;
  
    %% animation

     qMatrix=pickUpRobot.qMatrix_gen('rmrc',conveyor_pos,80);
     pickUpRobot.Plot(qMatrix,good{3});
     pickUpRobot.Reset();
     Move_conveyor(good{3});      
     b=VisServo(dropOffRobot,good{3});
     camview = EEcam(dropOffRobot);
     good{3}.color = camview.color;       % Set color of goods by camview
                                            % At the moment, use name
     pause(1);
     qMatrix=dropOffRobot.qMatrix_gen('jtraj',b.object_pose,80);
     dropOffRobot.Plot(qMatrix);
     
     % Identify the order of the goods kind we will deliver 
     switch good{3}.color
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
     goodsTr = GetGoodsDes(good{3},goodsOrder); 
     qMatrix=dropOffRobot.qMatrix_gen('jtraj',goodsTr,80);
     dropOffRobot.Plot(qMatrix,good{3});
     
     dropOffRobot.Reset();
         
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

