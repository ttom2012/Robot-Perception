close all; clear all; clc;
%this code is for plot the gif plot
for j=1:100:4000
    A=imread(sprintf('image/%d.png',j));
    [I,map]=rgb2ind(A,256);
    if(j==1)
        imwrite(I,map,'movefig.gif','DelayTime',0.1,'LoopCount',Inf)
    else
        imwrite(I,map,'movefig.gif','WriteMode','append','DelayTime',0.1)    
    end
end
