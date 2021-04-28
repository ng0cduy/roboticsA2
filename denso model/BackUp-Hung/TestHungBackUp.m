%% setup
clf
clear all
a = DensoVS060(false,transl(0,0,0),'denso');
% a.model.teach;
hold on;
red = goods('red',transl(0.4,0,0)*troty(pi));
%% animation
% After the color censor recognise the goods's color, the order of that color will
% increase 1 to allow the robot arm move the goods to the correct position
% with its color
steps = 50;
order = 1;
Animation(steps,a,red);
pause();
Animation(steps,a,red,order);
