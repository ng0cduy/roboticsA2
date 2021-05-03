function [robot1,robot2,boxes] = EnvironmentSetup
    clf;
    robot1 = DensoVS060(false,transl(0.8,0.45,0)*trotz(-pi/2),'denso_1');
    robot2 = DensoVS060(false,transl(-0.5,-0.5,0)*trotz(pi/2),'denso_2');
    hold on;
    conveyor = goods('conveyor.ply',transl(0,0,0.2));
    table = goods('table1.ply',transl(0.3,0,-0.26));
    % a.model.teach;
    
%     red = goods('red.ply',transl(0.4,0,0)*troty(pi)); 
    boxes = SetGoodsPick;
    
end

