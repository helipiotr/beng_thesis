%TRAJ_GEN Function generating 8-like trajectory
%	The function generates 8-like trajectory and adds artificial noise to
%	test the filter robustness. It includes values important for the Kalman filter
%	operation, and kalman_gps_ins should take care of assigning correct noise estimation 
%	values

%please mind the values coded in the Kalman filter
acc_noise=50*10^-6*9.81;
acc_bias=0.1;%50*10^-3*9.81;% calibrated: filter unstable with higher values
gyro_noise=deg2rad(10*10^-3);
r_gps_noise=(6.7/3); %because it is 3 sigma
v_gps_noise=0.1;%noise of velocity measurement

r_n_traj_gen = [deg2rad(52.259) deg2rad(21.020) 144]';
size_t=6000;    %size of the time list
ins_del_t = 0.01;   %desired main measurement latency 
gps_del_t = 1;     %desired GPS latency

data(:,:,1:10)=zeros(3,size_t,10);
data_q(:,:)=zeros(4,size_t);

acc_b_noise=wgn(3,size_t,acc_noise^2/ins_del_t/2,'linear')+...
    diag(wgn(3,1,acc_bias^2,'linear'))*ones(3,size_t);
om_b_ib_noise=wgn(3,size_t,gyro_noise^2/ins_del_t/2,'linear')/100;

gps_acquired=zeros(1,size_t);

for i=1:size_t
    t=(i-1)*ins_del_t;
    [ alpha(1,i) , r_n_ref(:,i), v_n_ref(:,i) ,acc_b(:,i), om_b_ib(:,i) ] =...
        intertial_data( t , r_n_traj_gen);
    
    GPS_acquired=(mod((i-1),gps_del_t/ins_del_t)==0);
    
    if GPS_acquired
        gps_acquired(i)=1;
        r_n_gps(:,i)=r_n_addnoise(r_n_ref(:,i), r_gps_noise);
        v_n_gps(:,i)=v_n_addnoise(v_n_ref(:,i), v_gps_noise);
    else
        r_n_gps(:,i)=r_n_gps(:,i-1);
        v_n_gps(:,i)=v_n_gps(:,i-1);

    end
    
    data(1:3,i,11)=r_n_gps(:,i);
    data(1:3,i,12)=v_n_gps(:,i);
    
    data(1:3,i,6)=r_n_ref(:,i);
    data(1:3,i,7)=v_n_ref(:,i);
    data(1,i,8)=alpha(1,i);
    
    %adding noise

    acc_b(:,i)=acc_b(:,i)+acc_b_noise(1:3,i);
    om_b_ib(:,i)=om_b_ib(:,i)+om_b_ib_noise(1:3,i);  
    data(1:3,i,4)=acc_b(:,i);   %it's the specific force with a negative sign
    data(1:3,i,5)=om_b_ib(:,i);
    
end

time = 0:ins_del_t:(ins_del_t*(size_t-1));%size_t-1
r_n_0=r_n_ref(:,1);
v_n_0=v_n_ref(:,1);

q_0=[0 0 sin(pi/4) cos(pi/4)]';
q=q_0;

del_r_n=[0;0;0];
del_v_n=[0;0;0];
data(1:3,i,1)= r_n_0;
data(1:4,i,2)= q_0;
data(1:3,i,3)= v_n_0;

clear GPS_acquired;

%finished generating trajectory