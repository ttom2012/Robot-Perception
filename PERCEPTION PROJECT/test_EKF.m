close all
clear all
clc
%% load data
load('ourdata11.mat'); %if you want to look at other data, you have to change this and change variable:"our_data1"
%% get Vm
Vm=[0,0.954672720828752,-0.297657514780710]+[0,0.977289220557714,-0.211909837859631]+[0,0.847397923256457,-0.530958340795814]+[0,0.999041751356042,-0.043767328539141];
Vm=Vm/norm(Vm); % this Vm is a corretion for the north deriction, since north measured by magno is not perpendicular to down measured by accelerometer
%% get t(i)
t(1)=0;
Pk_EKF=[];
n=size(our_data11,1);
%% simple calibraion 
%this is just using senser raw data to find bias, since we got a better
%calibration, we don't need this anymore.
%a=[0.0031    0.0446    0.0028   -0.0448    0.0020   -0.0275];
%our_data(:,2)=our_data(:,2)*(a(1)+1)+a(2);
%our_data(:,3)=our_data(:,3)*(a(3)+1)+a(4);
%our_data(:,4)=our_data(:,4)*(a(5)+1)+a(6);
%our_data(:,5)=medfilt1(our_data(:,5),20);
%our_data(:,6)=medfilt1(our_data(:,6),20);
%our_data(:,7)=medfilt1(our_data(:,7),20);
figure;
%% EKF 
for i=1:n
    if i>1   
    t(i)=our_data11(i,1)-our_data11(i-1,1);
    end 
    if i==1
    [Qval(i,:),Pk_EKF]=EkfFilter(randn(1,4),our_data11(i,:),t(i),Vm,Pk_EKF);
    %give a random value as the initial condition and see whether it will
    %converge. 
    else
    [Qval(i,:),Pk_EKF]=EkfFilter(Qval(i-1,:),our_data11(i,:),t(i),Vm,Pk_EKF);
    %rotation the initial position to the measured one.
    end

end
    %writerObj=VideoWriter('G:\out.avi');
    %open(writerObj);
for i=1:n
    R=quatern2rotMat(Qval(i,:)); %transformation from quaterion into euler rotation matrix

    r0=R(1,:);        g0=R(2,:);       b0=R(3,:);
    
plot3([0,r0(1)],[0,r0(2)],[0,r0(3)],'r',...
        [0,g0(1)],[0,g0(2)],[0,g0(3)],'g',...
        [0,b0(1)],[0,b0(2)],[0,b0(3)],'b')
    
    title(['i=' num2str(i)]);
    axis equal
    set(gca,'XLim',[-2.5 2.5]);
    set(gca,'YLim',[-2.5 2.5]);
    set(gca,'ZLim',[-2.5 2.5]);
    
    xlabel('X');  
    ylabel('Y');  
    zlabel('Z');  
    drawnow;

    %frame=getframe;
    %writeVideo(writerObj,frame);
 
end
%close(writerObj);
%this function is used for getting rotation matrix from a quaterion
function R = quatern2rotMat(q)

    R(1,1) = q(1)^2+q(2)^2-q(3)^2-q(4)^2;
    R(1,2) = 2*(q(2)*q(3)+q(1)*q(4));
    R(1,3) = 2*(q(2)*q(4)-q(1)*q(3));
    R(2,1) = 2*(q(2)*q(3)-q(1)*q(4));
    R(2,2) = q(1)^2+q(3)^2-q(2)^2-q(4)^2;
    R(2,3) = 2*(q(3)*q(4)+q(1)*q(2));
    R(3,1) = 2*(q(2)*q(4)+q(1)*q(3));
    R(3,2) = 2*(q(3)*q(4)-q(1)*q(2));
    R(3,3) = q(1)^2+q(4)^2-q(2)^2-q(3)^2;
end