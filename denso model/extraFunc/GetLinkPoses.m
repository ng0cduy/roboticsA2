%% function taken from lab 5 solutions
function [ transforms ] = GetLinkPoses( q, robot)
%% Determine the links' transform matrix of a given robot
links = robot.model.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.model.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end