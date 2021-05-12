function stopState = LCTesting(lightcurtain)
    ob=Obstacle('UFO.ply',transl(4,0,1));
%     for i = 4:-0.1:0
%         ob.Move(transl(i,0,1));
%         collisionFlag = lightcurtain.detect_Obstacle(ob);
%         if collisionFlag ==1
%             break
%         end
%         drawnow();
%         pause(0.1);
%     end
%     stopState = collisionFlag;
end 