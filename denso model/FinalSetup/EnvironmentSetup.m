function [robot1,robot2,boxes,lightcurtain,conveyor] = EnvironmentSetup
    clf;
%     set(0,'DefaultFigureWindowStyle');
    robot2 = DensoVS060(false,transl(0.8,0.35,0)*trotz(-pi/2),'denso_1');
    robot1 = DensoVS060(false,transl(-0.5,-0.5,0)*trotz(pi/2),'denso_2');
    hold on;
%     conveyor = goods('conveyor1.ply',transl(0,0,0.2));
    table = goods('table.ply',transl(0.3,0,-0.26));
    lightcurtain = LightCurtain(true);
    conveyor =Obstacle('conveyor1.ply',transl(0,-0.05,0.2));
    % a.model.teach;
%     estop = goods('estop.ply',transl(1.8,1.4,0.18));
%     human = goods('human.ply',transl(2,2,0.64));
%     brickwall=goods('brick_wall.ply',transl(0.4,3.5,1.3));
%     fire_ext=goods('ext.ply',transl(-2,2.5,0.2)*trotz(pi));
    
    ground_qx=[-5,-5;5,5];
    ground_qy=[-5,5;-5,5];
    ground_qz=[-0.49,-0.49;-0.49,-0.49];
    ground=ImgRead(ground_qx,ground_qy,ground_qz,'wood_floor.jpg');
    
    logo_qx1=[-1,-1;1,1];
    logo_qy1=[3.45,3.45;3.45,3.45];
    logo_qz1=[1,2;1,2];
    logo1=ImgRead(logo_qx1,logo_qy1,logo_qz1,'warning1.jpg');
    
    logo_qx2=[-1,-1;1,1];
    logo_qy2=[3.45,3.45;3.45,3.45];
    logo_qz2=[0,1;0,1];
    logo2=ImgRead(logo_qx2,logo_qy2,logo_qz2,'UTS.jpg');
%     red = goods('red.ply',transl(0.4,0,0)*troty(pi)); 
    boxes = SetGoodsPick;
    
end