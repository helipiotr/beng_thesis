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

%generating trajectory and assigning filter values
%traj_gen %<- simulated trajectory
addpath(genpath('C:\Users\Piotr\Documents\Nauka\Praca In¿ynierska\data'));
data_script

data(1:3,1,1) = r_n_0;
data_q(:,1) = q_0;
data(1:3,1,3) = v_n_0;

%GPS_acquired=0;
kalman_correction=zeros(9,1);

%iterating through the time list

last_ins = 0;
last_gps = 0;

for i=1:(size_t-1)
    
    ins_del_t = (time(i)-last_ins);
    last_ins=time(i);
    
    if ~isempty( find( start_list == i, 1) )
        op_mode = 1; %start;
    elseif ~isempty( find( stop_list == i, 1 ) )
        op_mode = 2; %stop;
    elseif ~isempty( find( resume_list == i, 1 ) )
        op_mode = 3; %resume;
    else
        op_mode = 4; %normal operation
    end
    
    if op_mode == 3
        a=acc_b(1:3,i);
        a=a/norm(a); 
        beta=acos(a(3));
        alpha=atan2(a(1),-a(2));
        C_b_n=rotz(rad2deg(alpha))*rotx(rad2deg(beta))*rotz(-attitude_mag(i));
        C_n_b=C_b_n';
        q_0 = C2q(C_n_b);
        clear alpha beta C_b_n C_n_b a;
    end
    
    [ r_n_ins , v_n_ins, q ] = ...
    INS_mechanisation( acc_b(:,i), om_b_ib(:,i), r_n_0, v_n_0, q_0, ins_del_t,...
    ~isempty(find(gps_acquired==(i),1)), kalman_correction, op_mode);

    %GPS_acquired=0;
    %we need to make sure the first correction is not taken into account
    %GPS_acquired=(mod((i-1),gps_del_t/ins_del_t)==0) & (i~=1) ;

    if ~isempty(find(gps_acquired==(i+1),1))
        gps_del_t = (time(i)-last_gps);
        last_gps=time(i);
        %GPS_acquired=1;
        [kalman_correction,del_r_n,del_v_n] = kalman_gps_ins(...,
            r_n_gps(:,i+1), v_n_gps(:,i+1), r_n_ins, v_n_ins, acc_b, q,...
            ins_del_t, gps_del_t,acc_noise, acc_bias, gyro_noise,...
            r_gps_noise, v_gps_noise);
    end
    
    if( i~= size_t )
        data(1:3,i+1,1) = r_n_ins;
        data_q(:,i+1) = q;
        data(1:3,i+1,3) = v_n_ins;
        data(1:3,i,9) = del_r_n; %yes some data will 
        data(1:3,i,10) = del_v_n;
        data(1:3,i,11) = kalman_correction(7:9);%del_eps
    end
    
    %{
    if (i-1)<(gps_del_t/ins_del_t)
        data(1:3,i,11)=r_n_ref(:,i+1);
        data(1:3,i,12)=v_n_ref(:,i+1);
    else
        data(1:3,i,11)=r_n_gps(:,i+1);
        data(1:3,i,12)=v_n_gps(:,i+1);
    end
    %}
end

toc
%postpro
postpro_data
