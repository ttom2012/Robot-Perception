function q = accMeg2qRichard(data)


vX=cross(data(1,8:10),data(1,2:4)); %get direction east
vX=vX/norm(vX); 
vY=cross(data(1,2:4),vX); %get direction north
vY=vY/norm(vY);

qX = qUtoV(vX,[1,0,0]); % get east as X direction

y= qMultiVec(vY, qX); %this step seems unnecessary
qY = qUtoV(y,[0,1,0]); %get north as Y direction

qx=[-qX(1),qX(2:4)]; %flip 180deg
qy=[-qY(1),qY(2:4)]; %flip 180deg

q =qMultiQ(qx,qy); %get gravity direction
q=[q(1),-q(2:4)];
if q(1)<0
    q=-q;
end
end