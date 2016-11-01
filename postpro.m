
t_mem=del_t:del_t:(del_t*size_t);

%plot(t_mem,(data(2,:,1)-data(2,1,1))*6*10^6, ...
%    t_mem,(data(2,:,6)-data(2,1,6))*6*10^6)

%plot(t_mem,(data(1,:,1)-data(1,1,1))*6*10^6,...
%    t_mem,(data(1,:,6)-data(1,1,6))*6*10^6)

%subplot(2,2,1)
%plot(t_mem,(data_q(3,:)),...
%    t_mem,(data_q(4,:)))

%plot(t_mem,2*rad2deg(atan2(data_q(3,:),data_q(4,:))),...
%    t_mem,rad2deg(atan2(data(2,:,3),data(1,:,3))))

%plot(t_mem,data(1,:,7), ...
%    t_mem,data(1,:,3))

%subplot(2,2,2)
%plot(t_mem,data(3,:,5))



%plotting r_n trajectory
%subplot(2,2,3)
plot(6*10^6*(data(1,:,1)-data(1,1,1)),6*10^6*(data(2,:,1)-data(2,1,1)),...
    6*10^6*(data(1,:,6)-data(1,1,6)),6*10^6*(data(2,:,6)-data(2,1,6)));

%plot(del_t:del_t:(del_t*size_t),data(1,:,7),...
%    del_t:del_t:(del_t*size_t),data(2,:,7)) 


%plot(t_mem,data(3,:,1)) %looks ok
%plot(t_mem,data4(1,:)) %looks ok
%plot(t_mem,data4(2,:))  %looks ok


%{
vec=[];
a=10;
c=5;
b=0.1;
d=2*b;

for i=1:600
    t=i/10;
    temp=[a*cos(b*t) c*sin(d*t) 0]';
    vec = [vec  temp]; 
end

%plot(cos(deg2rad(20))*(vec(1,:)-vec(1,1)),...
%   (vec(2,:)-vec(2,1)));
    
    %}


%Postprocessing
%{
%benchmark successfull
data=data1(2,1:100)-data1(2,1)*ones(1,100);
data=6.37*10^6*data*cos(phi);
t=0:0.001:0.1-0.001;
plot(t,data);

benchmark successfull
data=data1(1,1:100)-data1(3,1)*ones(1,100);
data=6.37*10^6*data;
t=0:0.001:0.1-0.001;
plot(t,data);

benchmark successfull
data=data1(3,1:100)-data1(3,1)*ones(1,100);
t=0:0.001:0.1-0.001;
plot(t,data);
%}
