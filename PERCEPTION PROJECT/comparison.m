%% comparison
%This code is for compare all the result together without calibration
%% load data
close all; clear all; clc;
load('test2.txt')
our_data11=test2
% load('Cdata.mat')
% our_data11=Cdata
%% set parameters
% this Vm is a corretion for the north deriction,...
% since north measured by magno is not perpendicular to down measured by accelerometer...
Vm=[0,0.954672720828752,-0.297657514780710]+[0,0.977289220557714,-0.211909837859631]...
    +[0,0.847397923256457,-0.530958340795814]+[0,0.999041751356042,-0.043767328539141];
Vm=Vm/norm(Vm); 
eInt=[];
t(1)=0;
Pk_EKF=[];
n=size(our_data11,1);
%% raw data (only gyro)
for i=1:n
     if i>1
         t(i)=our_data11(i,1)-our_data11(i-1,1);
     end
     norm_a(i)=norm(our_data11(i,2:4));
     norm_g(i)=norm(our_data11(i,5:7));
     %angular speed is very small, quaterion means no rotation (no fixing) 
    if norm_g(i)<0.05    %3*pi/180
        q(i,:)=[1,0,0,0];
    else
        %angular rotation in quaterion form
        q(i,:) = axisAngle2quatern(our_data11(i,5:7)/norm_g(i), norm_g(i)*t(i));
    end
    if i==1
        Q(i,:)  = accMeg2qRichard(our_data11(i,:));  % only gyro
    else
        Q(i,:)=quaternProd(Q(i-1,:),q(i,:));    %Q(i-1,:)*q(i,:)
    end
end


%% ekf
for i=1:n
    if i>1   
    t(i)=our_data11(i,1)-our_data11(i-1,1);
    end 
    if i==1
    [Qval0(i,:),Pk_EKF]=EkfFilter(randn(1,4),our_data11(i,:),t(i),Vm,Pk_EKF);
    %give a random value as the initial condition and see whether it will
    %converge. 
    else
    [Qval0(i,:),Pk_EKF]=EkfFilter(Qval0(i-1,:),our_data11(i,:),t(i),Vm,Pk_EKF);
    %rotation the initial position to the measured one.
    end

end
%% HLP (HIGH LOW PASS)
for i=1:n
    if i>1   
    t(i)=our_data11(i,1)-our_data11(i-1,1);
    end 
    if i==1
    Qval1(i,:)=HighLowPassFilter(randn(1,4),our_data11(i,:),t(i));
    %give a random value as the initial condition and see whether it will
    %converge. 
    else
    Qval1(i,:)=HighLowPassFilter(Qval1(i-1,:),our_data11(i,:),t(i));
    %rotation the initial position to the measured one.
    end

end
%% MAHONY
for i=1:n
    if i>1   
    t(i)=our_data11(i,1)-our_data11(i-1,1);
    end 
    if i==1
    Qval2(i,:)=MahonyFilter(randn(1,4),our_data11(i,:),t(i),Vm,eInt);
    %give a random value as the initial condition and see whether it will
    %converge. 
    else
    Qval2(i,:)=MahonyFilter(Qval2(i-1,:),our_data11(i,:),t(i),Vm,eInt);
    %rotation the initial position to the measured one.
    end

end
%% comparison result
% % compare the quterion
% figure(1)
% subplot(4,1,1)
% plot(1:n,Q(:,1),'k',1:n,Qval0(:,1),'r',1:n,Qval1(:,1),'g',1:n,Qval2(:,1),'b')
% legend('gyro','EKF','HLP','MAHONY')
% xlabel('1/100s')
% ylabel('q0')
% 
% hold on;
% subplot(4,1,2)
% plot(1:n,Q(:,1),'k',1:n,Qval0(:,2),'r',1:n,Qval1(:,2),'g',1:n,Qval2(:,2),'b')
% legend('gyro','EKF','HLP','MAHONY')
% xlabel('1/100s')
% ylabel('q1')
% 
% hold on
% subplot(4,1,3)
% plot(1:n,Q(:,1),'k',1:n,Qval0(:,3),'r',1:n,Qval1(:,3),'g',1:n,Qval2(:,3),'b')
% legend('gyro','EKF','HLP','MAHONY')
% xlabel('1/100s')
% ylabel('q2')
% 
% hold on
% subplot(4,1,4)
% plot(1:n,Q(:,1),'k',1:n,Qval0(:,4),'r',1:n,Qval1(:,4),'g',1:n,Qval2(:,4),'b')
% legend('gyro','EKF','HLP','MAHONY')
% xlabel('1/100s')
% ylabel('q3')

% compare the quterion
figure(1)
subplot(4,1,1)
plot(1:n,Qval0(:,1),'r',1:n,Qval1(:,1),'g',1:n,Qval2(:,1),'b')
legend('EKF','HLP','MAHONY','FontSize',12)
xlabel('1/100s')
ylabel('q0')

hold on;
subplot(4,1,2)
plot(1:n,Qval0(:,2),'r',1:n,Qval1(:,2),'g',1:n,Qval2(:,2),'b')
legend('EKF','HLP','MAHONY','FontSize',12)
xlabel('1/100s')
ylabel('q1')

hold on
subplot(4,1,3)
plot(1:n,Qval0(:,3),'r',1:n,Qval1(:,3),'g',1:n,Qval2(:,3),'b')
legend('EKF','HLP','MAHONY','FontSize',12)
xlabel('1/100s')
ylabel('q2')

hold on
subplot(4,1,4)
plot(1:n,Qval0(:,4),'r',1:n,Qval1(:,4),'g',1:n,Qval2(:,4),'b')
legend('EKF','HLP','MAHONY','FontSize',12)
xlabel('1/100s')
ylabel('q3')

% rotation plot
figure(2)
for i=1:50:n
    R0=quatern2rotMat(Q(i,:)); %transformation from quaterion into euler rotation matrix
    R1=quatern2rotMat(Qval0(i,:));
    R2=quatern2rotMat(Qval1(i,:));
    R3=quatern2rotMat(Qval2(i,:));
    
% r:x axis; g: y axis; b: z axis;
    r0=R0(1,:);        g0=R0(2,:);       b0=R0(3,:);
    r1=R1(1,:);        g1=R1(2,:);       b1=R1(3,:);
    r2=R2(1,:);        g2=R2(2,:);       b2=R2(3,:);
    r3=R3(1,:);        g3=R1(2,:);       b3=R3(3,:);
    
    
% figure(2);
plot3([0,r0(1)],[0,r0(2)],[0,r0(3)],'r',...
        [0,g0(1)],[0,g0(2)],[0,g0(3)],'g',...
        [0,b0(1)],[0,b0(2)],[0,b0(3)],'b',...
        ...
        [3,r1(1)+3],[0,r1(2)],[0,r1(3)],'r',...
        [3,g1(1)+3],[0,g1(2)],[0,g1(3)],'g',...
        [3,b1(1)+3],[0,b1(2)],[0,b1(3)],'b',...
        ...
        [6,r2(1)+6],[0,r2(2)],[0,r2(3)],'r',...
        [6,g2(1)+6],[0,g2(2)],[0,g2(3)],'g',...
        [6,b2(1)+6],[0,b2(2)],[0,b2(3)],'b',...
        ...
        [9,r3(1)+9],[0,r3(2)],[0,r3(3)],'r',...
        [9,g3(1)+9],[0,g3(2)],[0,g3(3)],'g',...
        [9,b3(1)+9],[0,b3(2)],[0,b3(3)],'b')
    
    title(['i=' num2str(i)]);
    text(-2,0,4,'only gyro ');
    text(1,0,4,'EKF');
    text(4,0,4,'HLP');
    text(7,0,4,'MAHONY');
    axis equal
    set(gca,'XLim',[-3 12]);
    set(gca,'YLim',[-3 3]);
    set(gca,'ZLim',[-3 3]);
    
    xlabel('X');  
    ylabel('Y');  
    zlabel('Z');  
    drawnow;
% print(2,'-dpng',sprintf('image/%d',i)); %this is code is for save image
% to creat gif
% close;
    
end
%% calibrated result
