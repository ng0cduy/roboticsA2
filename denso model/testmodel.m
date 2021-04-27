
%%
clf
clear all
a = DensoVS060(false,transl(0,0,0),'denso');
% a.model.teach;
hold on;
green= goods('green',transl(0.4,0.3,-0.13)*troty(pi));

%%
keyboard
clc
pose = transl(0,0,0.5)*troty(pi);
q = a.IKine(pose);
a.Animate(green.pos_,50,green);
% a.Reset()
%% test collision

clc;
close all;
% mdl_planar3
[vertex,faces,faceNormals] =RectangularPrism([0.8,-.11,-.5], [.2,.11,.5]);
axis equal
camlight
q1 = [-pi/3,0,0,0,0,0];
q2 =  [pi/3,0,0,0,0,0];
% keyboard;
a.model.plot(q1)
steps = 50;
qMatrix = jtraj(q1,q2,steps);

for i= 1:1:steps
    result(i) = IsCollision(a,qMatrix(i,:),faces,vertex,faceNormals);
    if result(i) >= 1
%         qMatrix(i,:);
        display(num2str(qMatrix(i,:)));
        break
    end
    a.model.animate(qMatrix(i,:));
end