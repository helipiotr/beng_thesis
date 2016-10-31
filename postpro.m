
plot(del_t:del_t:(del_t*size_t),(data(1,:,1)-data(1,1,1))*6*10^6, ...
    del_t:del_t:(del_t*size_t),(data(1,:,6)-data(1,1,6))*6*10^6)

%plot(del_t:del_t:(del_t*size_t),data7(1,:), ...
%    del_t:del_t:(del_t*size_t),data3(1,:))

%plot(del_t:del_t:(del_t*size_t),data5(1,:))



%plotting r_n trajectory
%plot(6*10^6*(data6(1,:)-data6(1,1)),...
%    6*10^6*(data6(2,:)-data6(2,1)));

%plot(del_t:del_t:(del_t*size_t),data7(1,:)) %looks ok
%plot(del_t:del_t:(del_t*size_t),data7(2,:)) %looks ok
%plot(del_t:del_t:(del_t*size_t),data4(1,:)) %looks ok
%plot(del_t:del_t:(del_t*size_t),data4(2,:))  %looks ok


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
