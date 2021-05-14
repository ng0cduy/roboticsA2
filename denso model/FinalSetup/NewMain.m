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
    good{i} = goodsArray{i}; %temporary
    pose = good{i}.pos_*transl(0,0,-0.08);

    qMatrix = pickUpRobot.qMatrix_gen('rmrc',pose,100);

    pickUpRobot.Plot(qMatrix);
    

    % delivering objects
    %% define parameters
    conveyor_pos = transl(-0.6,-0.08,0.4)*troty(pi);
    
    %% animation
    %GOODS to conveyor

     qMatrix=pickUpRobot.qMatrix_gen('jtraj',conveyor_pos,30,conveyor);
     pickUpRobot.Plot(qMatrix,good{i});
     pickUpRobot.Reset();

%      Move_conveyor(good{i});   
%      camview{i} = EEcam(dropOffRobot);
%      
%      view(60,40);
%      zoom(2.4);
% %      pause(1);
%      b{i}=VisServo(dropOffRobot,good{i});
%      
%      good{i}.color = camview{i}.color;       % Set color of goods by camview
% %                                             At the moment, use name
%      disp(num2str(b{i}.object_pose));
%      disp(good{i}.color);
%    
% %      pause(1);
%      qMatrix=dropOffRobot.qMatrix_gen('jtraj',b{i}.object_pose*troty(pi),80);
%      dropOffRobot.Plot(qMatrix);
%      
%      % Identify the order of the goods kind we will deliver 
%      switch good{i}.color
%          case 'red'
%              redOrder = redOrder + 1;
%              goodsOrder = redOrder;
%              
%          case 'blue'
%              blueOrder = blueOrder + 1;
%              goodsOrder = blueOrder;
%          case 'green'
%              greenOrder = greenOrder + 1;
%              goodsOrder = greenOrder; 
%          otherwise
%              disp('the color is not in the default set of color')
%              goodsOrder = -1;
     
    
     
     Move_conveyor(good{i});   
     camview{i} = EEcam(dropOffRobot);
     
     view(60,40);
     zoom(2.2);
%      pause(1);
     b{i}=VisServo(dropOffRobot,good{i});
     
     good{i}.color = camview{i}.color;       % Set color of goods by camview
%                                             At the moment, use name
     disp(num2str(b{i}.object_pose));
     disp(good{i}.color);
   
%      pause(1);
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
     disp(num2str(goodsOrder));
     
     % deliver the goods
     goodsTr = GetGoodsDes(good{i},goodsOrder); 
%      if strcmpi(good{i}.color,'green')
%         qMatrix=dropOffRobot.qMatrix_gen('jtraj',goodsTr,80);
%         dropOffRobot.Plot(qMatrix,good{i});
%      else
         qGoal = dropOffRobot.IKine(goodsTr);
    %      pose=transl(0.5,0.55,0.1)*troty(pi);
         qMatrix=dropOffRobot.Check_Collision1(qGoal,good{i},ob);
         dropOffRobot.Plot(qMatrix,good{i});
%      end
%      disp(num2str(goodsOrder));
%      
%      % deliver the goods
%      goodsTr = GetGoodsDes(good{i},goodsOrder); 
%      qGoal = dropOffRobot.IKine(goodsTr);
%      qMatrix=dropOffRobot.Check_Collision1(qGoal,good{i},ob);
%      dropOffRobot.Plot(qMatrix,good{i});
%      dropOffRobot.Reset();
    end
    

end