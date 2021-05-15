clf
robot = DensoVS060New(false,transl(0.8,0.35,0)*trotz(-pi/2),'denso_1');
hold on;

ob=Obstacle('UFO.ply',transl(1.15,-0.05,0.45));
g = goods('blue.ply',transl(0.8,-0.04,0.4)*troty(pi));

qMatrix=robot.qMatrix_gen('jtraj',transl(0.8,-0.08,0.4)*troty(pi),80);
robot.Plot(qMatrix);
% robot.model.teach

%%
goodsTr = GetGoodsDes(g,2); 

qMatrix=robot.Check_Collision(goodsTr,g,ob);
robot.Plot(qMatrix,g);
