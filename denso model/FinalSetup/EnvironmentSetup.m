%% functions to set up the environment and return main components to GUI
function [robot1,robot2,boxes,lightcurtain,conveyor] = EnvironmentSetup
    clf;
    
    % set up robots
    robot1 = DensoVS060(transl(-0.5,-0.5,0)*trotz(pi/2),'denso_1');
    robot2 = DensoVS060(transl(0.8,0.35,0)*trotz(-pi/2),'denso_2');
   
    hold on;
    
    % set up environment
    table = goods('table.ply',transl(0.35,-0.1,-0.26));
    lightcurtain = LightCurtain(true);
    conveyor =Obstacle('conveyor1.ply',transl(0,-0.05,0.2));
    
    estop = goods('estop.ply',transl(1.8,0.9,0.14));
    human = goods('human.ply',transl(2,2,0.64));
    brickwall=goods('brick_wall.ply',transl(0.4,3,1.3));
    fire_ext=goods('ext.ply',transl(-2,2.4,0.2)*trotz(pi));
%     
    red_qx=[1.25-0.12,1.25-0.12;1.25+0.12,1.25+0.12];
    red_qy=[0.2-0.12,0.2+0.12;0.2-0.12,0.2+0.12];
    red_qz=[-0,-0;-0,-0];
    red=ImgRead(red_qx,red_qy,red_qz,'red.png');
    
    green_qx=[1.25-0.12,1.25-0.12;1.25+0.12,1.25+0.12];
    green_qy=[0.32,0.56;0.32,0.56];
    green_qz=[-0,-0;-0,-0];
    green=ImgRead(green_qx,green_qy,green_qz,'green.png');
    
    blue_qx=[1.25-0.12,1.25-0.12;1.25+0.12,1.25+0.12];
    blue_qy=[0.56,0.56+0.24;0.56,0.56+0.24];
    blue_qz=[-0,-0;-0,-0];
    blue=ImgRead(blue_qx,blue_qy,blue_qz,'blue.png');
 
    ground_qx=[-5,-5;5,5];
    ground_qy=[-5,5;-5,5];
    ground_qz=[-0.49,-0.49;-0.49,-0.49];
    ground=ImgRead(ground_qx,ground_qy,ground_qz,'wood_floor.jpg');
    
    logo_qx1=[-1,-1;1,1];
    logo_qy1=[2.9,2.9;2.9,2.9];
    logo_qz1=[1,2;1,2];
    logo1=ImgRead(logo_qx1,logo_qy1,logo_qz1,'warning1.jpg');
    
    logo_qx2=[-1,-1;1,1];
    logo_qy2=[2.9,2.9;2.9,2.9];
    logo_qz2=[0,1;0,1];
    logo2=ImgRead(logo_qx2,logo_qy2,logo_qz2,'UTS.jpg');
    
    poster_qx=[-4,-4;-4,-4];
    poster_qy=[-4,-4;3,3];
    poster_qz=[-0.5,2;-0.5,2];
    poster=ImgRead(poster_qx,poster_qy,poster_qz,'Lab.jpg');
%     red = goods('red.ply',transl(0.4,0,0)*troty(pi)); 
    boxes = SetGoodsPick;
    
end