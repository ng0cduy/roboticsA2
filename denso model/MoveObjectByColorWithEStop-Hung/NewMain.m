function NewMain(robot,goods,guiObj)
    %% coming to objects
    %% define parameters
    steps = 50;
    pose = goods.pos_;
    
    % change these to RMRC
    qNew = robot.IKine(pose);
    qMatrix = jtraj(robot.model.getpos, qNew,steps);
    %% animation
    i = 1;
    working = true;
    machineState = 1;  %working, 0 means not working.
    %%
 while(working)   
        if ((guiObj.getEstopState == 0) && (machineState == 1))
            Animation(robot,qMatrix(i,:));
            pause(0.1);
            i = i+1;
        end
        if guiObj.getEstopState == 1
            machineState = 0;
            pause(0.1);
        end
        while (machineState == 0)
            pause(0.5);
            if guiObj.getEstopState == 0
                guiObj.resumeSign
                if guiObj.resumeSign == 1
                    machineState = 1;
                    guiObj.resumeSign = 0;
                end
            end
        end 
    if steps < i %finish the task
            working = false;
    end
 end 
            

    %% delivering objects
    %% define parameters
    order = 1;      % the first item of this kind.
    pose = [eye(3), (GetGoodsDes(goods,goods.color,order))';ones(1,4)] * troty(pi); 
    % change these to RMRC
    qNew = robot.IKine(pose);
    qMatrix = jtraj(robot.model.getpos, qNew,steps);

    %% animation
    i = 1;
    working = true;
    machineState = 1;  %working, 0 means not working.
    %%
while(working)   
        if ((guiObj.getEstopState == 0) && (machineState == 1))
            Animation(robot,qMatrix(i,:),goods);
            pause(0.1);
            i = i+1;
        end
        if guiObj.getEstopState == 1
            machineState = 0;
            pause(0.1);
        end
        while (machineState == 0)
            pause(0.5)
            if guiObj.getEstopState == 0
                guiObj.resumeSign
                if guiObj.resumeSign == 1
                    machineState = 1;
                    guiObj.resumeSign = 0; 
                end
            end
        end 
    if steps < i %finish the task
            working = false;
    end
 end 

end

