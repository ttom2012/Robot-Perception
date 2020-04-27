close all; clear all; clc;
% this code is for getting a calibrated raw data
% load('ourdata11.mat')
load('test3.txt')
Rdata=test3
%% parameter setting
a=[0.0035 0.0422 0.0033 -0.0424 0.0021 -0.0264]

%% acc

Accrx_c=(a(1).*Rdata(:,2)+a(2)+Rdata(:,2));   % Offset the bias and noise of raw data
Accry_c=a(3).*Rdata(:,3)+a(4)+Rdata(:,3);
Accrz_c=a(4).*Rdata(:,4)+a(6)+Rdata(:,4);
Accr_c=[Accrx_c,Accry_c,Accrz_c]
%% gyro
gyrox_c=medfilt1(Rdata(:,5),20); %Use a 20th-order median filter to eliminate the non zero bias of triaxial Gyro
gyroy_c=medfilt1(Rdata(:,6),20);
gyroz_c=medfilt1(Rdata(:,7),20)
gyro_c=[gyrox_c,gyroy_c,gyroz_c]
%% mag
M(:,1)=Rdata(:,8);
M(:,2) = Rdata(:,9);
M(:,3) = Rdata(:,10);
xmin = min(M(:,1));
ymin = min(M(:,2));
zmin = min(M(:,3));
xmax = max(M(:,1));
ymax = max(M(:,2));
zmax = max(M(:,3));
xc = 0.5*(xmax+xmin);
yc = 0.5*(ymax+ymin);
zc = 0.5*(zmax+zmin);
a = 0.5*abs(xmax-xmin);
b = 0.5*abs(ymax-ymin);
c = 0.5*abs(zmax-zmin);

x = M(:,1);
y = M(:,2);
z = M(:,3);
L = length(x(:,1));
err = 0;
for i=1:L
    err = err + abs((x(i)-xc)^2/a^2 + (y(i)-yc)^2/b^2+(z(i)-zc)^2/c^2 - 1 );
end
x = M(:,1);
y = M(:,2);
z = M(:,3);
xclast = xc;
yclast = yc;
zclast = zc;
alast = a;
blast = b;
clast = c;
errlast = 100000000000;
for i = 1:20000
    r = rand(1,6);
    xcnew = xclast + r(1)-0.5;
    ycnew = yclast + r(2)-0.5;
    zcnew = zclast + r(3)-0.5;
    anew = abs(alast + r(4)-0.5);
    bnew = abs(blast + r(5)-0.5);
    cnew = abs(clast + r(6)-0.5);
    errnew = 0;
    for j=1:L
        errnew = errnew + abs((x(j)-xcnew)^2/anew^2 + (y(j)-ycnew)^2/bnew^2+(z(j)-zcnew)^2/cnew^2 - 1 );
    end
    if(errnew<errlast)  
        xclast = xcnew;
        yclast = ycnew;
        zclast = zcnew;
        alast = anew;
        blast = bnew;
        clast = cnew;
        errlast = errnew;
    end
end
avr = (alast+blast+clast)/3;
X_c=xclast*ones(length(M),1);
Y_c=yclast*ones(length(M),1);
Z_c=zclast*ones(length(M),1);
MC(:,1) = (M(:,1)+X_c)*(alast/avr);
MC(:,2) = (M(:,2)-Y_c)*(blast/avr);
MC(:,3) = (M(:,3)-Z_c)*(clast/avr);
Mcx=xc*ones(length(M),1)+M(:,1);
Mcy=-yc*ones(length(M),1)+M(:,2);
Mcz=zc*ones(length(M),1)+M(:,3);
Mcg=[Mcx,Mcy,Mcz]
%% calibrated data
Cdata=[Rdata(:,1),Accr_c,gyro_c,Mcg]
save('Cdata.mat','Cdata')