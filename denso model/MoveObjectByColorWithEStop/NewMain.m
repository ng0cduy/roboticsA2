function NewMain(pickUpRobot,dropOffRobot,goodsArray,guiObj)
     pickUpRobot.Reset()
    %% coming to objects
    %% define parameters
    goods = goodsArray{3}; %temporary
    steps = 50;
    pose = goods.pos_*transl(0,0,-0.06);
    
    % change these to RMRC
%     qNew = pickUpRobot.IKine(pose);
%     qMatrix = jtraj(pickUpRobot.model.getpos, qNew,steps);
%     %% animation
%     i = 1;
%     working = true;
%     machineState = 1;  %working, 0 means not working.
    %%
%  while(working)   
%         if ((guiObj.getEstopState == 0) && (machineState == 1))
%             Animation(pickUpRobot,qMatrix(i,:));
% %             pause(0.1);
%             i = i+1;
%         end
%         if guiObj.getEstopState == 1
%             machineState = 0;
% %             pause(0.001);
%         end
%         while (machineState == 0)
% %             pause(0.001);
%             if guiObj.getEstopState == 0
%                 guiObj.resumeSign
%                 if guiObj.resumeSign == 1
%                     machineState = 1;
%                     guiObj.resumeSign = 0;
%                 end
%             end
%         end 
%     if steps < i %finish the task
%             working = false;
%     end
%  end 
    qMatrix = pickUpRobot.qMatrix_gen('rmrc',pose,50);
    pickUpRobot.Plot(qMatrix);
            

    %% delivering objects
    %% define parameters
    order = 1;      % the first item of this kind.
    pose = transl(0.8,-0.05,0.4)*troty(pi); 
    % change these to RMRC
    qNew = pickUpRobot.IKine(pose);
    qMatrix = jtraj(pickUpRobot.model.getpos, qNew,steps);

    %% animation
    i = 1;
    working = true;
    machineState = 1;  %working, 0 means not working.
    %%
% while(working)   
%         if ((guiObj.getEstopState == 0) && (machineState == 1))
%             Animation(pickUpRobot,qMatrix(i,:),goods);
% %             pause(0.1);
%             i = i+1;
% %             pickUpRobot.Animate('rmrc',pose,50);
%         end
%         if guiObj.getEstopState == 1
%             machineState = 0;
% %             pause(0.1);
%         end
%         while (machineState == 0)
% %             pause(0.5)
%             if guiObj.getEstopState == 0
%                 guiObj.resumeSign
%                 if guiObj.resumeSign == 1
%                     machineState = 1;
%                     guiObj.resumeSign = 0; 
%                 end
%             end
%         end 
%     if steps < i %finish the task
%             working = false;
%     end
%  end 
 qMatrix=pickUpRobot.qMatrix_gen('rmrc',pose,50);
 pickUpRobot.Plot(qMatrix,goods);

end

