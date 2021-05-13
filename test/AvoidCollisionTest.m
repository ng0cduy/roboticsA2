clf
robot = DensoVS060New(false,transl(0.8,0.35,0)*trotz(-pi/2),'denso_1');
hold on;
ground_qx=[-5,-5;5,5];
    ground_qy=[-5,5;-5,5];
    ground_qz=[-0.49,-0.49;-0.49,-0.49];
    ground=ImgRead(ground_qx,ground_qy,ground_qz,'wood_floor.jpg');
ob=Obstacle('UFO.ply',transl(1.15,-0.05,0.45));
g = goods('green.ply',transl(0.8,-0.04,0.4)*troty(pi));
clc
EEcam(robot)

% qMatrix=robot.qMatrix_gen('jtraj',transl(0.8,-0.08,0.4)*troty(pi),80);
% robot.Plot(qMatrix);

%%
% goodsTr = GetGoodsDes(g,1); 
% 
% qGoal = robot.IKine(goodsTr);
% qMatrix=robot.Check_Collision(qGoal,g,ob);
% robot.Plot(qMatrix,g);
