function RunDemo(pickUpRobot,dropOffRobot,goodsArray,guiobj,conveyor)
    clc;
    view(60,40);
    pan on;
    hold on;
    zoom(2.2);
    ob=Obstacle('UFO.ply',transl(3,-0.05,0.45));
    for i = 3:-0.05:1.15
        ob.Move(transl(i,-0.05,0.45));
        pause(0.001);
        drawnow;
    end
    
%      Reset the robot to intial place
    pickUpRobot.Reset();
    redOrder = 0;
    blueOrder = 0;
    greenOrder = 0;
    
    disp('Simulation Start');
  
    %% coming to objects
    %% define parameters
    for i=6:-1:1
    %Robot to goods
    good{i} = goodsArray{i}; %assign the goods from the array
    pose = good{i}.pos_*transl(0,0,-0.08); %take the pose of the good

    qMatrix = pickUpRobot.qMatrix_gen('rmrc',pose,80); %generate the path from the
                                                        %to the good
    pickUpRobot.Plot(qMatrix); %plot the path
    

    % delivering objects
    %% define parameters
    conveyor_pos = transl(-0.6,-0.08,0.4)*troty(pi); %position of the conveyor
    
    %% animation
    %GOODS to conveyor
     clc;
     disp('Checking for collision before picking up');
     qMatrix=pickUpRobot.qMatrix_gen('jtraj',conveyor_pos,60,conveyor); %generate the path from the goods to the conveyor, check if there is any collision
     pickUpRobot.Plot(qMatrix,good{i}); 
     pickUpRobot.Reset(); 

     Move_conveyor(pickUpRobot,good{i});
     pause(0.5);
     camview{i} = EEcam(dropOffRobot);
     
     view(60,40);
     zoom(2.2);
%      pause(1);
     good{i}.color = camview{i}.color;       % Set color of goods by camview
%      disp(good{i}.color);
    guiobj.ColorEditField.Value = good{i}.color ;
    if strcmpi(good{i}.color,'red') == 1
        guiobj.ColorLamp.Color = 'r';
    elseif strcmpi(good{i}.color,'green') ==1
        guiobj.ColorLamp.Color = 'g';
    elseif strcmpi(good{i}.color,'blue') ==1
        guiobj.ColorLamp.Color = 'b';
    end
    b{i}=VisServo(dropOffRobot,good{i}); %Use visual servo to estimate the pose of the goods
    
%      adjust the endEffector
     qMatrix=dropOffRobot.qMatrix_gen('jtraj',b{i}.object_pose*troty(pi),80);
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
     clc;
     goodsTr = GetGoodsDes(good{i},goodsOrder); 
     disp('Checking collision before drop off, please wait');
     qMatrix=dropOffRobot.EllipsoidQGen(goodsTr,good{i},ob);
     dropOffRobot.Plot(qMatrix,good{i});
     
     % reset to its initial pose
     qMatrix=dropOffRobot.ElipsoidResetQgen(ob);
     dropOffRobot.Plot(qMatrix);
     dropOffRobot.Reset();
    end
    clc;
    disp('End of the Simulation');
end