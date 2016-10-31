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
q_0=[0 0 0 1]';
q=q_0;

size_t=6000;
del_t = 0.01; %set hard in different integrating functions



data(:,:,1:7)=zeros(3,size_t,7);
data_q(:,:)=zeros(4,size_t);

for i=1:size_t
    t=i*del_t;
    
    [ r_n_ref, v_n_ref ,f_b, om_b_ib ] = intertial_data( t , r_n_traj_gen);
    
    if i == 1
       r_n_0=r_n_ref;
       v_n_0=v_n_ref;
       data(1:3,i,1)= r_n_0;
       data(1:4,i,2)= q_0;
       data(1:3,i,3)= v_n_0;
    end
   
    [ r_n , v_n, q ] = ...
    INS_mechanisation( f_b, om_b_ib, r_n_0, v_n_0, q_0);

    if( i~= size_t )
        data(1:3,i+1,1)= r_n;
        data_q(:,i+1)= q;
        data(1:3,i+1,3)= v_n;
    end
    
    data(1:3,i,4)=f_b;
    data(1:3,i,5)=om_b_ib;
    data(1:3,i,6)=r_n_ref;
    data(1:3,i,7)=v_n_ref;
end

toc
