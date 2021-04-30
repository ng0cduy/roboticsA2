% global eStop;
% eStop = false;
clc
clear all
close all
a = TestApp
for i = 1:1:1000
    if a.getEstopState ==0
        disp('off')
    else
        disp('on')
    end
    pause(0.1)

end
        