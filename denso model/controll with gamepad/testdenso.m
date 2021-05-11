% Gamepad for Denso Robot
clc
clear all
clf
%%
id = 1;
joy = vrjoystick(id);
caps(joy)
q = zeros(1,6);
duration = 100;
dt = 0.15;
n =0;
denso = DensoVS060(false,transl(0,0,0),'denso');
denso.model.delay=0.001;
kLinear = 0.3;
kAngular = 0.8;
%%
tic;
while(toc <duration)
    n = n+1;
    [axes, buttons, povs] = read(joy);
    J = denso.model.jacob0(q);
    vx = kLinear*axes(1);
    vy = kLinear*axes(2);
    vz = kLinear*axes(3);
    wx = kAngular*(buttons(2)-buttons(1));
    wy = kAngular*(buttons(7)-buttons(5));
    wz = kAngular*(buttons(8)-buttons(6));
    xdot = [vx;vy;vz;wx;wy;wz];
    qdot = J\xdot;
    % Update plot
    q = q + dt*qdot';
    denso.model.animate(q);  
    for joint = 1:1:6
        if q(joint) < denso.model.qlim(joint,1)
            qdot(joint) = 0;
        elseif q(joint) > denso.model.qlim(joint,2)
            qdot(joint) =0;
        end 
    end
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n) % wait until loop time (dt) has elapsed 
    end
end