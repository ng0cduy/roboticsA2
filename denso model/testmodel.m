
%%
clf;
clc
a = DensoVS060(false,transl(0,0,0),'denso');
% a.model.teach;
hold on;
blue= goods('blue.ply',transl(0.4,0,0.05)*troty(pi));
red= goods('red.ply',transl(-0.4,0,0.05)*troty(pi));
table=goods('table1.ply',transl(0,0,-0.3));
%% Test rmrc
clc
qMatrix = a.GenerateRMRC(transl(-0.4,0,0)*troty(pi),50);
%%
for i =1:size(qMatrix,1)
    a.model.animate(qMatrix(i,:));
    drawnow();
    pause(0.05);
end
%%
a.Reset;
a.Animate(transl(0.5,0.2,0)*troty(pi),50,blue,table);
a.Animate(transl(0.5,-0.4,0)*troty(pi),50,blue,table);
a.Reset;
%% 

