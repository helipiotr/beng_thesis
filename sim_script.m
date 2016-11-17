% variable format
% om_b_ei - angular velocity om described in reference frame b(ody) 
% measuring the angular speed between e(arth) and i(nertial) framess 
% C_b_n - transformation matrix between n(avigation) and b
% f_b - force measured in b


% hard-coded values of r_n and v_n -> they have to be changed in 
% further versions, Warszawa 52.259 deg N, 21.020 deg E, 144 meters over
% ellipsioid

clear
clear functions
tic
r_n_traj_gen = [deg2rad(52.259) deg2rad(21.020) 144]';
q_0=[0 0 sin(pi/4) cos(pi/4)]';
q=q_0;

size_t=6000;
del_t = 0.01; 

data(:,:,1:10)=zeros(3,size_t,10);
data_q(:,:)=zeros(4,size_t);

%please mind the values coded in the kalman filter
acc_noise=50*10^-6*9.81;
gyro_noise=deg2rad(10*10^-3);
r_gps_noise=(6.7/3); %because it is 3 sigma
v_gps_noise=1/sqrt(2);%noise of velocity measurement

f_b_noise=wgn(3,size_t,acc_noise^2/del_t/2,'linear');
om_b_ib_noise=wgn(3,size_t,gyro_noise^2/del_t/2,'linear');

r_n_noise=wgn(3,size_t,r_gps_noise^2/del_t,'linear');
%derivation in notes
v_n_noise=wgn(3,size_t,2*r_gps_noise^2/del_t^2,'linear');

for i=1:size_t
    t=(i-1)*del_t;

    [ alpha , r_n_ref, v_n_ref ,f_b, om_b_ib ] =...
        intertial_data( t , r_n_traj_gen);
    
    %adding noise
    f_b=f_b+f_b_noise(1:3,i);
    om_b_ib=om_b_ib+om_b_ib_noise(1:3,i);
    
    r_n_gps=r_n_addnoise(r_n_ref, r_gps_noise);
    v_n_gps=v_n_addnoise(v_n_ref, v_gps_noise, del_t);
    
    if i == 1
       r_n_0=r_n_ref;
       v_n_0=v_n_ref;
       data(1:3,i,1)= r_n_0;
       data(1:4,i,2)= q_0;
       data(1:3,i,3)= v_n_0;
    end
   
    [ r_n_ins , v_n_ins, q ] = ...
    INS_mechanisation( f_b, om_b_ib, r_n_0, v_n_0, q_0, del_t);
    
    [del_r_n,del_v_n] = kalman_gps_ins(r_n_gps, v_n_gps, r_n_ins,...
        v_n_ins, f_b, q, del_t, acc_noise, gyro_noise, r_gps_noise, ...
        v_gps_noise);

    if( i~= size_t )
        data(1:3,i+1,1) = r_n_ins;
        data_q(:,i+1) = q;
        data(1:3,i+1,3) = v_n_ins;
        data(1:3,i,9) = del_r_n;
        data(1:3,i,10) = del_v_n;
    end
    
    data(1:3,i,4)=f_b;
    data(1:3,i,5)=om_b_ib;
    data(1:3,i,6)=r_n_ref;
    data(1:3,i,7)=v_n_ref;
    data(1,i,8)=alpha;
end

toc

postpro
