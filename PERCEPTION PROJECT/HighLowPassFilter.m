function [Qfuse2]=HighLowPassFilter(Qfuse1,ImuData,t)
%high and low pass filter to Gyro atitude with Accelerate & Magnetic

%fixing ratio (how much you want to fix each loop)
a=0.01;
% get magnitude of rotation
norm_g=norm(ImuData(1,5:7));
norm_a=norm(ImuData(1,2:4));
%for really small rotation
if norm_g<0.05    %3*pi/180    
    q=[1,0,0,0];    %just assume there is no rotation
else
    %take the integrate of gyro as the rotation angle
    q=axisAngle2quatern(ImuData(5:7)/norm_g,  norm_g*t); 
    
end
% when system doesn't have fast change in movement and rotation
if abs(norm_a-9.8)<2 && norm_g< 0.1
    
    Qtemp1= accMeg2qRichard(ImuData);     %world coordinate       
    Qtemp2=quaternProd(Qfuse1,q);         %using rotation to fix direction
    if Qtemp1*Qtemp2'>0       
        Qfuse2=(1-a)*Qtemp2+a*Qtemp1;       %this step is used since sometimes the rotation is negative and just beacuse of calculation
    else                                    %this negative value doesn't mean it the rotation direction was wrong
        Qfuse2=(1-a)*Qtemp2-a*Qtemp1;       %this is just due to quaterion calculation and you can also see similar thing in other script
    end    
    Qfuse2=Qfuse2/norm(Qfuse2);      
else   
    Qfuse2=quaternProd(Qfuse1,q);           %if the system change fast then we just using gyro to fix the value
end
if Qfuse2(1)<0
    Qfuse2=-Qfuse2;                         % rotation is always with in a small amount
end
end

% this function is transfering rotation of 'angle' about 'axis' into quaterion
function q = axisAngle2quatern(axis, angle)
    q0 = cos(angle./2);
    q1 = axis(:,1)*sin(angle./2);
    q2 = axis(:,2)*sin(angle./2);
    q3 = axis(:,3)*sin(angle./2); 
    q = [q0 q1 q2 q3];
end

%this function is the cross product of two quaterion also as A acting on B
function ab = quaternProd(a, b)
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
end

%this function is to find world coordinate using acce data and megne data
%this function is used to get the opposite of gravity direction but only
%using the first value (initial value)
function q = accMeg2qRichard(data)


vX=cross(data(1,8:10),data(1,2:4));
vX=vX/norm(vX);
vY=cross(data(1,2:4),vX);
vY=vY/norm(vY);

qX = qUtoV(vX,[1,0,0]);

y= qMultiVec(vY, qX);
qY = qUtoV(y,[0,1,0]);

qx=[-qX(1),qX(2:4)];
qy=[-qY(1),qY(2:4)];

q =qMultiQ(qx,qy);
q=[q(1),-q(2:4)];
end

%this function is the cross product of two quaterion also as A acting on B
function [qq]=qMultiQ(p,q)   %p*q
qq=[...
        p(1) * q(1) - p(2) * q(2) - p(3) * q(3) - p(4) * q(4)...
       ,p(2) * q(1) + p(1) * q(2) - p(4) * q(3) + p(3) * q(4)...
       ,p(3) * q(1) + p(4) * q(2) + p(1) * q(3) - p(2) * q(4)...
       ,p(4) * q(1) - p(3) * q(2) + p(2) * q(3) + p(1) * q(4)  ];

end

%this function is used for rotate the first vector into the second vecter
%and repersent in a quaterion form
function q = qUtoV(u, v)        %two vetor rotation to quaternions
nu = u/norm(u);
nv = v/norm(v);
%the angle between two vector
if (u*v' == -1)
    q = [0, [1,0,0]]; %180deg
end
    half = (nu + nv)/norm(nu + nv);
    q = [nu*half',cross(nu, half)];
end

%this function is using a quaterion rotation acting on a vector
function [vector]=qMultiVec(vec,q)  %sensor frame to world frame
x = q(2);
y = q(3);
z = q(4);
w = q(1);

vecx = vec(1);
vecy = vec(2);
vecz = vec(3);

x_ =  w * vecx  +  y * vecz  -  z * vecy;
y_ =  w * vecy  +  z * vecx  -  x * vecz;
z_ =  w * vecz  +  x * vecy  -  y * vecx;
w_ = -x * vecx  -  y * vecy  -  z * vecz;

vector = [x_ * w  +  w_ * -x  +  y_ * -z  -  z_ * -y ...
    , y_ * w  +  w_ * -y  +  z_ * -x  -  x_ * -z ...
    , z_ * w  +  w_ * -z  +  x_ * -y  -  y_ * -x ...
    ];

end