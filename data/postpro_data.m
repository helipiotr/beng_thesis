%time = ins_del_t:ins_del_t:(ins_del_t*(size_t-1));

%tspan=size_t-1;
tspan=2000;

%subplot(1,2,1)

%plotting r_n trajectory, with reference 3D

plot3(6*10^6*(data(1,1:tspan,1)-data(1,1,1)), ... 
    6*10^6*(data(2,1:tspan,1)-data(2,1,1)), ...
    data(3,1:tspan,1)-data(3,1,1),...
    6*10^6*(data(1,1:tspan,6)-data(1,1,6)), ... 
    6*10^6*(data(2,1:tspan,6)-data(2,1,6)), ...
    data(3,1:tspan,6)-data(3,1,6));
axis equal;


%just filtered trajectory
%{
plot3(6*10^6*(data(1,1:tspan,1)-data(1,1,1)), ... 
    6*10^6*(data(2,1:tspan,1)-data(2,1,1)), ...
    data(3,1:tspan,1)-data(3,1,1))
axis equal;
%}

%just GPS trajectory
%{
plot3(6*10^6*(data(1,1:tspan,6)-data(1,1,6)), ... 
    6*10^6*(data(2,1:tspan,6)-data(2,1,6)), ...
    data(3,1:tspan,6)-data(3,1,6));
axis equal;
%}

%{
plot(6*10^6*(data(1,1:tspan,1)-data(1,1,1)), ... 
    6*10^6*(data(2,1:tspan,1)-data(2,1,1)),...
    6*10^6*(data(1,1:tspan,6)-data(1,1,6)), ... 
    6*10^6*(data(2,1:tspan,6)-data(2,1,6)));
axis equal;
%}
%subplot(1,2,2);

%plot(time, data(2,:,3))



%{
plot(6*10^6*(data(1,:,1)-data(1,1,1)), ... 
    6*10^6*(data(2,:,1)-data(2,1,1)), ... 
    6*10^6*(data(1,:,6)-data(1,1,6)),...%+wgn(1,size_t,r_gps_noise^2,'linear'),...
    6*10^6*(data(2,:,6)-data(2,1,6)));%+wgn(1,size_t,r_gps_noise^2,'linear'));
%}

%{
plot(6*10^6*(data(1,:,1)-data(1,1,6)), ... 
    6*10^6*(data(2,:,1)-data(2,1,6)), ... 
    6*10^6*(data(1,:,6)-data(1,1,6)),...%+wgn(1,size_t,r_gps_noise^2,'linear'),...
    6*10^6*(data(2,:,6)-data(2,1,6)));%+wgn(1,size_t,r_gps_noise^2,'linear'));
%}
%plot(t_mem, data(1,:,12), t_mem, data(1,:,7))

%plot(del_t:del_t:(del_t*size_t),data(1,:,7),...
%    del_t:del_t:(del_t*size_t),data(2,:,7)) 


%plot(t_mem,data(3,:,1)) %looks ok
%plot(t_mem,data4(1,:)) %looks ok
%plot(t_mem,data4(2,:))  %looks ok

