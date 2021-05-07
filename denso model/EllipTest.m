clf
robot = DensoVS060New(false,transl(0.8,0.35,0)*trotz(-pi/2),'denso_1');
hold on;
ob = Obstacle(transl(1.05,-0.38,0.176));
g = goods('red.ply',transl(1,-0.08,0.32)*troty(pi));

pause()
qMatrix=robot.qMatrix_gen('jtraj',transl(1,-0.08,0.32)*troty(pi),80);
robot.Plot(qMatrix);

EllipCheckNew(robot,g,robot.model.getpos(),'goods')

Points = ob.CreateMesh(false);
plot3(Points(:,1),Points(:,2),Points(:,3),'cyan*');


EllipCheckNew(robot,g,robot.model.getpos(),'obs',Points)