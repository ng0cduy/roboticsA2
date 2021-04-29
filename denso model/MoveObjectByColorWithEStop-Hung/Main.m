function Main
    %% set up
    clf
    clear all
    robot = DensoVS060(false,transl(0,0,0),'denso');
    % a.model.teach;
    hold on;
    red = goods('red.ply',transl(0.4,0,0)*troty(pi)); 
    robot.Reset;
    %% coming to objects
    %% define parameters
    steps = 200;
    pose = red.pos_;

    % change these to RMRC
    qNew = robot.IKine(pose);
    qMatrix = jtraj(robot.model.getpos, qNew,steps);
    %% animation
    i = 1;
    global eStop
    TestApp
    working = true;
    machineState = 1;  %working, 0 means not working.
    %%
    while (working)
        if machineState == 1
            Animation(robot,qMatrix(i,:));
            i = i+1;
            pause(0.05);
            if eStop == 1 % eStop switch is turned on 
               machineState = 0;
            end
        end
        
        if machineState == 0
            qMatrix(i,:);
            if eStop == 0 % eStop switch is turned off
                machineState = 1;
            end
        end

        if steps < i %finish the task
            working = false;
        end
        eStop
    end

    %% delivering objects
    %% define parameters
    order = 1;      % the first item of this kind.
    pose = [eye(3), (GetGoodsDes(red,red.color,order))';ones(1,4)]*troty(pi); % change goodsObj.name to color info later


    % change these to RMRC
    qNew = robot.IKine(pose);
    qMatrix = jtraj(robot.model.getpos, qNew,steps);

    %% animation
    i = 1;
    while (working)
        if machineState == 1
            Animation(robot,qMatrix(i,:), red);
            i = i+1;
            pause(0.05);
            if eStop == 1 % eStop switch is turned on 
               machineState = 0;
            end
        end
        
        if machineState == 0
                if eStop == 0 % eStop switch is turned off
                    machineState = 1;
                end
        end

        if 50 < i %finish the task
            working = false;
        end
        eStop
    end

end

