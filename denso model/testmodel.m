
%%
clf
clear all
clc
a = DensoVS060(false,transl(0,0,0),'denso');
% a.model.teach;
hold on;
<<<<<<< HEAD
green= goods('green',transl(0.4,0,0)*troty(pi));
=======
green= goods('green',transl(-0.4,0,-0.13)*troty(pi));
%% Test rmrc
clc
qMatrix = a.GenerateRMRC(transl(-0.4,0,0)*troty(pi),50);
%%
for i =1:size(qMatrix,1)
    a.model.animate(qMatrix(i,:));
    drawnow();
    pause(0.05);
end
>>>>>>> 09b9d7aa6ad97ac4bb44423024e60b410a79e223
%%
a.Animate(transl(0.5,0.2,0)*troty(pi),50,green);
a.Animate(transl(0.5,-0.4,0)*troty(pi),50,green);
% a.Reset;


