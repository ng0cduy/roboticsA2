function NewMain(pickUpRobot,dropOffRobot,goodsArray,guiobj,conveyor,lightcurtain)
     clc;
%      conveyor =Obstacle('conveyor1.ply',transl(0,-0.05,0.2));
     view(60,40);
     pan on;
%      zoom(2.2);
     ob=Obstacle('UFO.ply',transl(1.15,-0.1,0.35));
     pickUpRobot.Reset();
     dropOffRobot.Reset();
%      dropOffRobot.Reset();
    redOrder = 0;
    blueOrder = 0;
    greenOrder = 0;
  
    %% coming to objects
    %% define parameters
    for i=6:-1:1
    %Robot to goods
    good{i} = goodsArray{i}; %assign the goods from the array
    pose = good{i}.pos_*transl(0,0,-0.08); %take the pose of the good

    qMatrix = pickUpRobot.qMatrix_gen('rmrc',pose,100); %generate the path from the
                                                        %to the good
    pickUpRobot.Plot(qMatrix); %plot the path
    

    % delivering objects
    %% define parameters
    conveyor_pos = transl(-0.6,-0.08,0.4)*troty(pi); %position of the conveyor
    
    %% animation
    %GOODS to conveyor
     qMatrix=pickUpRobot.qMatrix_gen('jtraj',conveyor_pos,60,conveyor); %generate the path from the goods to the conveyor, check if there is any collision
     pickUpRobot.Plot(qMatrix,good{i}); 
     pickUpRobot.Reset(); 

     Move_conveyor(good{i});   
     camview{i} = EEcam(dropOffRobot);
     
     view(60,40);
     zoom(2.5);
%      pause(1);
     good{i}.color = camview{i}.color;       % Set color of goods by camview
%      disp(good{i}.color);
    guiobj.ColorEditField.Value = good{i}.color ;
    if strcmpi(good{i}.color,'red') == 1
        guiobj.ColorLamp.Color = 'r';
    elseif strcmpi(good{i}.color,'blue') ==1
        guiobj.ColorLamp.Color = 'b';
    elseif strcmpi(good{i}.color,'green') ==1
        guiobj.ColorLamp.Color = 'g';
    end
    b{i}=VisServo(dropOffRobot,good{i}); %Use visual servo to estimate the pose of the goods
%      pause(1);
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
     goodsTr = GetGoodsDes(good{i},goodsOrder); 
     qGoal = dropOffRobot.IKine(goodsTr);
     qMatrix=dropOffRobot.Check_Collision1(qGoal,good{i},ob);
     dropOffRobot.Plot(qMatrix,good{i});
     dropOffRobot.Reset();
    end
    

end