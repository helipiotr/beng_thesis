tic

%disp('importing data');
%import=import_data('dane_ostatnie.csv');
%calibration=import_data('calibration.csv');
load('import.mat');
%import = calibration;

acc(:,1) = cell2mat(import(2:size(import,1),1));
acc(:,2) = cell2mat(import(2:size(import,1),2));
acc(:,3) = cell2mat(import(2:size(import,1),3));
time = 10^-3 * cell2mat(import(2:size(import,1),33)); %milliseconds
%span=1:size(acc,1);
%span=1:500;


%subplot(1,2,1);
subplot(2,2,1)
%plot(sqrt(acc(:,1).^2+acc(:,2).^2+acc(:,3).^2));
%plot(time,acc(:,1),time,acc(:,2),time,acc(:,3))
%plot(span,acc(:,1),span,acc(:,2),span,acc(:,3))

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

%subplot(1,2,2);


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


%plot(sqrt(acc(:,1).^2+acc(:,2).^2+acc(:,3).^2))
%plot(time,acc(:,1),time,acc(:,2),time,acc(:,3))

%plot(span,acc(:,1),span,acc(:,2),span,acc(:,3))
%estimating the gravity vector (or at least a part of it)

%gamma=9.8123;

init_span=220:323;

a=mean(acc(init_span,:))';
a=a/norm(a);

%plusminus
beta=acos(a(3));

alpha=atan2(a(1),-a(2));

%not rotated about x axis
C_b_n_pre=rotz(rad2deg(alpha))*rotx(rad2deg(beta));
C_b_n_pre=C_b_n_pre*rotz(90);

%out1=C_b_n_pre'*a;
gyro_b=(C_b_n_pre'*gyro')';
plot(time,gyro_b(:,1),time,gyro_b(:,2),time,gyro_b(:,3))

subplot(2,2,2)
acc_b=(C_b_n_pre'*acc')';
plot(time,acc_b(:,1),time,acc_b(:,2),time,acc_b(:,3))
mag_b=(C_b_n_pre'*mag')';

for i=1:3
mag_b(:,i)=mag_b(:,i)-0.5*(max(mag_b(:,i))+min(mag_b(:,i)));
end


subplot(2,2,3)
%plot(time,mag_b(:,1),...
%    time,mag_b(:,2),...
%    time,mag_b(:,3));

plot(time, speed);

%plot(time, sqrt(out(:,1).^2+out(:,2).^2+out(:,3).^2))

attitude_mag=rad2deg(atan2(mag_b(:,2),mag_b(:,1)));

subplot(2,2,4)

%attitude_gps = fillmissing(attitude_gps,'next');




plot(time, attitude_mag, time,...
    attitude_gps-360);

figure
plot(10^6*6*deg2rad(long-long(1)),10^6*6*deg2rad(lat-lat(1)))
axis equal

toc


