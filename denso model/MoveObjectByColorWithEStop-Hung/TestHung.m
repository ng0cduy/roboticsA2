%% setup
clf
clear all
robot = DensoVS060(false,transl(0,0,0),'denso');
% a.model.teach;
hold on;
red = goods('red.ply',transl(0.4,0,0)*troty(pi));
%% animation
% coming to objects
steps = 50;
pose = red.pos_;

% change these to RMRC
qNew = robot.IKine(pose);
qMatrix = jtraj(robot.model.getpos, qNew,steps);

i = 1;
while (1)
    for i = i:steps
        if robot.eStop == 1 %estop GUI will call function robot.eStopToggle to change this 
            break;
        end
        Animation(robot,qMatrix(i,:));
        pause(0.05);
    end
    
    while robot.eStop == 1 %if robot is stil not reactivated by pressing estop
    end 

    if i == 50 %finish the task
        break;
    end
end

% delivering objects
order = 1;      % the first item of this kind.
pose = [eye(3), (GetGoodsDes(red,red.color,order))';ones(1,4)]*troty(pi); % change goodsObj.name to color info later


% change these to RMRC
qNew = robot.IKine(pose);
qMatrix = jtraj(robot.model.getpos, qNew,steps);

i = 1;
while (1)
    for i = i:steps
        if robot.eStop == 1 %estop GUI will call function robot.eStopToggle to change this 
            break;
        end
        Animation(robot,qMatrix(i,:),red);
        pause(0.05);
    end
    
    while robot.eStop == 1 %if robot is stil not reactivated by pressing estop
    end    

    if i == 50 %finish the task
        break;
    end
end