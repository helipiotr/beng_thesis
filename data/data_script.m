clear

%disp('importing data');
%import=import_data('dane_ostatnie.csv');
%calibration=import_data('calibration.csv');
load('import.mat');
%import = calibration;

acc(:,1) = cell2mat(import(2:size(import,1),1));
acc(:,2) = cell2mat(import(2:size(import,1),2));
acc(:,3) = cell2mat(import(2:size(import,1),3));
time = 10^-3 * cell2mat(import(2:size(import,1),33)); %milliseconds

gyro(:,1) = cell2mat(import(2:size(import,1),10));
gyro(:,2) = cell2mat(import(2:size(import,1),11));
gyro(:,3) = cell2mat(import(2:size(import,1),12));

mag(:,1) = cell2mat(import(2:size(import,1),14));
mag(:,2) = cell2mat(import(2:size(import,1),15));
mag(:,3) = cell2mat(import(2:size(import,1),16));

speed = cell2mat(import(2:size(import,1),28));
altitude_gps = cell2mat(import(2:size(import,1),25));
altitude_barometer = cell2mat(import(2:size(import,1),27));

attitude_gps = cell2mat(import(2:size(import,1),30));


lat=cell2mat(import(2:size(import,1),23));
long=cell2mat(import(2:size(import,1),24));

%calibrating
load('calibration_results.mat')

for i=1:3
    acc(:,i)=acc(:,i)-x(i)*ones(size(acc,1),1);
end

for i=1:3
    acc(:,i)=acc(:,i)/(1+x(i+3));
end

init_span=220:323;

a=mean(acc(init_span,:))';
a=a/norm(a);

%plusminus
beta=acos(a(3));

alpha=atan2(a(1),-a(2));

%not rotated about x axis
C_b_n_pre=rotz(rad2deg(alpha))*rotx(rad2deg(beta));
%C_b_n_pre=C_b_n_pre*rotz(90);

%out1=C_b_n_pre'*a;

%those transposistions may lie about the spatial orienatation
gyro_n=(C_b_n_pre'*gyro')';



acc_n=(C_b_n_pre'*acc')';

mag_n=(C_b_n_pre'*mag')';

for i=1:3
mag_n(:,i)=mag_n(:,i)-0.5*(max(mag_n(:,i))+min(mag_n(:,i)));
end



attitude_mag=rad2deg(atan2(mag_n(:,2),mag_n(:,1)));



attitude_gps = fillmissing(attitude_gps,'nearest');

%{
subplot(2,2,1)
plot(time,gyro_n(:,1),time,gyro_n(:,2),time,gyro_n(:,3))
subplot(2,2,2)
plot(time,acc_n(:,1),time,acc_n(:,2),time,acc_n(:,3))
subplot(2,2,3)
plot(time, speed);
subplot(2,2,4)
plot(time, attitude_mag, time,...
    attitude_gps);

figure
scatter(10^6*6*deg2rad(long-long(1)),10^6*6*deg2rad(lat-lat(1)))
axis equal
%}

time = time';
size_t = size(time,2);

%TODOwe can't use acc_b since the noise might be correlated
acc_b = acc';
om_b_ib = gyro';
r_n_gps = [deg2rad(lat') ; deg2rad(long') ; altitude_gps'];
v_n_gps = [cos(deg2rad((attitude_gps'))).*speed';...
    sin(deg2rad((attitude_gps'))).*speed';...
    zeros(1,size_t)];

data(1:3,:,6)=r_n_gps;
data(1:3,:,7)=v_n_gps;

r_n_0 = r_n_gps(:,1);
v_n_0 = [0 0 0]';
addpath(genpath('C:\Users\Piotr\Documents\Nauka\Praca In¿ynierska'));
C_b_n=rotz(rad2deg(alpha))*rotx(rad2deg(beta))*rotz(-attitude_mag(1));
C_n_b=C_b_n';
q_0 = C2q(C_n_b);
del_r_n=[0;0;0];
del_v_n=[0;0;0];


gyro_0=(C_b_n'*gyro')';

acc_0=(C_b_n'*acc')';

mag_0=(C_b_n'*mag')';

acc_noise=0.01;%0.03;
acc_bias=0.01;%0.05;%50*10^-3*9.81;% calibrated: filter unstable with higher values
gyro_noise=0.01;%deg2rad(10*10^-3);
r_gps_noise=(3/3); %because it is 3 sigma
v_gps_noise=0.1;%noise of velocity measurement

last=1;
gps_acquired=zeros(1,size_t);
for i = 1:size_t
    if ~strcmp(import(i+1,32),import(i,32)) && (i > last+5 )
       last=i;
       gps_acquired(i)=1;
    end
end


