clear all    
close all    
clc  
%% IMU static calibration for Acc%%
t=30;               % sampling time
N=1400; 
data1 = xlsread('x.xls'); % raw data of IMU at 
data2 = xlsread('y.xls');
data3 = xlsread('z.xls');
xdata1=data1(52:1551,2);
xdata2=data2(52:1551,2);
xdata3=data3(52:1551,2);
xdata=[xdata1;xdata2;xdata3];
ydata1=data1(52:1551,3);
ydata2=data2(52:1551,3);
ydata3=data3(52:1551,3);
ydata=[ydata1;ydata2;ydata3];
zdata1=data1(52:1551,4);
zdata2=data2(52:1551,4);
zdata3=data3(52:1551,4);
zdata=[zdata1;zdata2;zdata3];
Data=[xdata,ydata,zdata];  % Bias & measurement noise of triaxial Acc
f=@(a,xdata)(a(1).*Data(:,1)+a(2)).^2 + (a(3).*Data(:,2)+a(4)).^2 + (a(5).*Data(:,3)+a(6)).^2;
a0=[1,0,1,0,1,0] % cost function to estimate accelerometers' parameters
G=-9.79968^2*ones(length(xdata),1); % the exact value of gravity in college station
options=optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt'); 
lb=[];
ub=[];
a=lsqcurvefit(f,a0,Data,G,lb,ub,options) % Employ the levenberg-marquardt algorithm to minimize the cost function
%rdata=xlsread('raw.xls');
Rdata=Data;
Accrx_c=(a(1).*Rdata(52:1551,1)+a(2)+Rdata(52:1551,1));   % Offset the bias and noise of raw data
Accry_c=a(3).*Rdata(52:1551,2)+a(4)+Rdata(52:1551,2);
Accrz_c=a(5).*Rdata(52:1551,3)+a(6)+Rdata(52:1551,3);
Accr_c=[Accrx_c,Accry_c,Accrz_c]
y=(Accr_c(:,1).^2+Accr_c(:,2).^2+Accr_c(:,3).^2);
r=(Rdata(52:1551,1).^2+Rdata(52:1551,2).^2+Rdata(52:1551,3).^2);
G_H=-9.79968^2*ones(length(y),1).^2;
yce=sum(abs(a))*(y+G_H);            % errors of calibrated Acc
rac=r+G_H;                          % errors of raw data
figure(1)
plot(1:N,yce(1:N))
hold on
grid on
title('Acc bias&noise calibration')
plot(1:N,rac(1:N),'r')                       

%% IMU static calibration for gyro%%
gyro_N=xlsread('gyro.xls');     % Horizontal 
fs=50;
N=1500;
gyrox_N=180*(gyro_N(52:1551,2))/pi; % rad 2 degree
gyroy_N=180*(gyro_N(52:1551,3))/pi;
gyroz_N=180*(gyro_N(52:1551,4))/pi;
xfft=fft(gyrox_N);                  % Power spectrum of static noise
P=abs(xfft).*abs(xfft);
f=0:1*fs/N:fs-1*fs/N;
figure(3);
plot(f,20*log10(P));        % if the bias of gyro is Gaussian distribution, we use Gaussian filtering to elimate the bias.
xlabel('Frequency/Hz');          
ylabel('Power/dB');         % if not the Gaussian noise, we use the nonlinear median filtering for gyro calibration
figure(4);
gyrox_C=medfilt1(gyrox_N,20); %Use a 20th-order median filter to eliminate the non zero bias of triaxial Gyro
gyroy_C=medfilt1(gyroy_N,20);
gyroz_C=medfilt1(gyroz_N,20)
subplot(1,2,1);
xlabel('Time(t)');          
ylabel('Deg');  
title('Raw Data of Gyro (static in degree)')
plot(1/fs:1/fs:N/fs,gyrox_N);
subplot(1,2,2);
title('Calibrated gyro data (static in degree)')
plot(1/fs:1/fs:N/fs,gyrox_C)
%% IMU calibration for Mag%%
Mdata=xlsread('M.xls')
M(:,1) = Mdata(101:3100,2);
M(:,2) = Mdata(101:3100,3);
M(:,3) = Mdata(101:3100,4);
figure(5)
subplot(1,2,1);
title('Raw data of magnetometer X-Z')
grid on
plot(M(:,1),M(:,3));
subplot(1,2,2);
title('Raw data of magnetometer Y-Z')
plot(M(:,2),M(:,3));
scatter3(M(:,1),M(:,2),M(:,3));    %3D & 2D raw data plot of Magnetometer

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
%scatter3(MC(:,1),MC(:,2),MC(:,3),'r');
Mcx=xc*ones(length(M),1)+M(:,1);
Mcy=-yc*ones(length(M),1)+M(:,2);
Mcz=zc*ones(length(M),1)+M(:,3);
Mcg=[Mcx,Mcy,Mcz]
figure(6)
subplot(1,2,1)
grid on
plot(Mcg(:,1),Mcg(:,3),'r')
hold on
plot(M(:,1),M(:,3),'b')
subplot(1,2,2)
grid on
plot(Mcg(:,2),Mcg(:,3),'r')
figure(7)
plot(M(:,2),M(:,3),'b')
scatter3(Mcg(:,1),Mcg(:,2),Mcg(:,3)); 
hold on
scatter3(M(:,1),M(:,2),M(:,3),'r')
