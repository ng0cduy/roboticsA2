% function LightCurtainTest()
clear;
clf;
clc;

ob = Obstacle('UFO.ply',transl(-3,0,0.95));
hold on
lCurtain = LightCurtain(true);


%%
c =lCurtain.detect_Obstacle(ob);
for i = 0:0.1:4
    ob.UpdatePose(transl(i,0,1));
    collisionFlag = lCurtain.detect_Obstacle(ob);
    if collisionFlag ==1
        break
    end
    drawnow();
    pause(0.01);
end

