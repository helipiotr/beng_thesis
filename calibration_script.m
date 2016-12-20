%disp('importing data');
load('calibration.mat')
%calibration=import_data_cal('calibration.csv');

%1-x 2-y 3-z
acc(:,1) = cell2mat(calibration(2:size(calibration,1),1));
acc(:,2) = cell2mat(calibration(2:size(calibration,1),2));
acc(:,3) = cell2mat(calibration(2:size(calibration,1),3));
time = 0.5 * 10^-3 * cell2mat(calibration(2:size(calibration,1),32)); %milliseconds


%plot(time,acc(:,1),time,acc(:,2),time,acc(:,3))

gyro(:,1) = cell2mat(calibration(2:size(calibration,1),10));
gyro(:,2) = cell2mat(calibration(2:size(calibration,1),11));
gyro(:,3) = cell2mat(calibration(2:size(calibration,1),12));

%plot(time,gyro(:,1),time,gyro(:,2),time,gyro(:,3))

subplot(2,1,1)
plot(sqrt(acc(:,1).^2+acc(:,2).^2+acc(:,3).^2))
%subplot(1,2,2);
%plot(time,gyro_x,time,gyro_y,time,gyro_z)

%lat=cell2mat(calibration(2:size(import,1),23));
%long=cell2mat(calibration(2:size(import,1),24));

%plot(10^6*6*deg2rad(long-long(1)),10^6*6*deg2rad(lat-lat(1)))
%axis equal

%plot(1:size(acc_x,1),acc_x,1:size(acc_x,1),acc_y,1:size(acc_x,1),acc_z)

%inserting measured values

range{1}=5:37;
range{2}=52:84;
range{3}=104:137;
range{4}=153:183;
range{5}=204:238;
range{6}=250:298;

gamma=9.8123;

%cal_phi(i,k,l)
for i = 1:size(range,2)
    cut = acc(range{i},:);
    l(i,:) = mean(cut, 1);
end

handle = @(x) f_g(x,l,gamma);
x0=zeros(6,1);

options = optimoptions(@fsolve,'Display','iter',...
    'Algorithm','trust-region',...
    'SpecifyObjectiveGradient',false,'PrecondBandWidth',0);
[x,fval,exitflag,output] = fsolve(handle,x0,options);

bgx=x(1);
bgy=x(2);
bgz=x(3);
sgx=x(4);
sgy=x(5);
sgz=x(6);

for i=1:3
    acc(:,i)=acc(:,i)-x(i)*ones(size(acc,1),1);
end

for i=1:3
    acc(:,i)=acc(:,i)/(1+x(i+3));
end

subplot(2,1,2)
plot(sqrt(acc(:,1).^2+acc(:,2).^2+acc(:,3).^2))



%{
M=zeros(6,6);

for i=1:6
    for j=1:6
        for k=1:size(range,2)
            M(i,j) = M(i,j)+cal_phi(i,k,l)*cal_phi(j,k,l);
        end
    end
end


for i=1:6
    sum=0;
    for k=1:size(range,2)
        sum=sum+cal_phi(i,k,l);
    end
    z(i,1)=gamma*sum;
end

a=M\z; %cos nie dziala

%}
%{
lg(1)=
lg(2)=
lg(3)=
lg(4)=
lg(5)=
lg(6)=
%}