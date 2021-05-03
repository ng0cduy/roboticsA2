function [robot,boxes] = EnvironmentSetup
    clf;
    robot = DensoVS060(false,transl(0,0,0),'denso');
    % a.model.teach;
    hold on;
%     red = goods('red.ply',transl(0.4,0,0)*troty(pi)); 
    boxes = SetGoodsPick;
    
end

