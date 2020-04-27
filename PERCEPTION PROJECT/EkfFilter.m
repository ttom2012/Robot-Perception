function [Q2,Pk2]=EkfFilter(Q1,ImuData,t,Vm,Pk1)
%EKF filter to Gyro atitude with Accelerate & Magnetic
% derivation <A Double-Stage Kalman Filter for Orientation Tracking
%            with an Integrated Processor in 9-D IMU>

%initial condition for P0 (error covariance)
if isempty(Pk1)
   Pk1=[ 0.1 , 0.01 , 0.01 , 0.01;...
           0.01 , 0.1 , 0.01 , 0.01;...
           0.01 , 0.01 , 0.1 , 0.01;...
           0.01 , 0.01 , 0.01 , 0.1  ]*0.001;
       
end
%setting bias for gyro, acce and magn
para1=0.0001;%0.0001;
para2=0.1;
para3=0.5;
Qk=para1*eye(4);   
Rk1=para2*eye(3);
Rk2=para3*eye(3);

%getting initial gyro data * initial time step=rotation
wx=ImuData(1,5)*t;
wy=ImuData(1,6)*t;
wz=ImuData(1,7)*t;

%initial G value and theta value
norm_a=norm(ImuData(1,2:4));
norm_g=norm(ImuData(1,5:7));

%using rotation we get before to find Ak matrix (I+A)
Ak=[ 1    , -wx/2 , -wy/2 , -wz/2 ;...
     wx/2 ,   1   ,  wz/2 , -wy/2 ;...
     wy/2 , -wz/2 ,   1   ,  wx/2 ;...
     wz/2 ,  wy/2 , -wx/2 ,   1   ];

% predicting the second step quaterion
Qp=Ak*Q1';     %Q_predict

Qp=Qp/norm(Qp);

%predicting the new error covariance
P_k=Ak*Pk1*Ak'+Qk;

%if the acceleration change and rotation change is not very fast
if abs(norm_a-9.8)<2 && norm_g< 2
    %R = quatern2rotMat(Qp);
    R2=[2*Qp(2)*Qp(3)+2*Qp(1)*Qp(4);...
        2*Qp(1)^2+2*Qp(3)^2-1;...
        2*Qp(3)*Qp(4)-2*Qp(1)*Qp(2)];

    R3=[2*Qp(2)*Qp(4)-2*Qp(1)*Qp(3);...
        2*Qp(3)*Qp(4)+2*Qp(1)*Qp(2);...
        2*Qp(1)^2+2*Qp(4)^2-1 ];   


    J2=2*[ Qp(4), Qp(3), Qp(2) , Qp(1);...
           Qp(1),-Qp(2), Qp(3) ,-Qp(4);...
          -Qp(2),-Qp(1), Qp(4) , Qp(3)] ;   

    J3=2*[-Qp(3), Qp(4),-Qp(1) , Qp(2);...
           Qp(2), Qp(1), Qp(4) , Qp(3);...
           Qp(1),-Qp(2),-Qp(3) , Qp(4)] ;   

    h1=R3;                               %acc in sensor fixed frame 
    h2=Vm(2)*R2+Vm(3)*R3;                %mag in sensor fixed frame

    Hk1=J3;                              %Hk matrix for accelerometer
    Hk2=Vm(2)*J2+Vm(3)*J3;               %Hk matrix for megnetometer

    Kk1=P_k*Hk1'*inv(Hk1*P_k*Hk1'+Rk1);

    qe1=Kk1*(ImuData(1,2:4)'/norm_a-h1);         % Acc difference value between real & prediction from attitude
                                                 % this fix pitch and roll
    Pk_1=(eye(4)-Kk1*Hk1)*P_k;
    
    Kk2=P_k*Hk2'*inv(Hk2*P_k*Hk2'+Rk2);

    mag=ImuData(1,8:10);
    mag=mag/norm(mag);

    qe2=Kk2*(mag'-h2);       % mag difference value between real & prediction from attitude
                             % this fix yaw
    Pk2=(eye(4)-Kk2*Hk2)*Pk_1;
    
else
    %if the rotation is so fast and with high acceleration the answer will
    %be un-trustworthy so we just don't do the rotation at all
    qe1=[0;0;0;0];
    Pk_1=P_k;
    qe2=[0;0;0;0];
    Pk2=Pk_1;
    
end
    

% Kk2=P_k*Hk2'*inv(Hk2*P_k*Hk2'+Rk2);
% 
% mag=ImuData(1,8:10);
% mag=mag/norm(mag);
% 
% qe2=Kk2*(mag'-h2);       % mag difference value bwtween real & prediction from attitude

qt=Qp+qe1+qe2;             % rotation + fixed pitch roll + fixed yaw

Q2=qt'/norm(qt);           % normalization so it can fit the function quatern2rotMat()

if Q2(1)<0
    Q2=-Q2;                % rotation is always with in a small amount
end

       
%Pk2=(eye(4)-Kk2*Hk2)*Pk_1;
  
end



