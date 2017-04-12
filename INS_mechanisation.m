function [ r_n_out , v_n_out, q_out ] = ...
    INS_mechanisation( acc_b, om_b_ib, r_n_0, v_n_0, q_0, ins_del_t,...
    GPS_acquired, kalman_correction)
%INS_MECHANISATION This function computes the appropriate state values 
%   based on accelerometer and gyroscope readings 
%   This function takes acceleration, gyroscope readings and returns next 
%	position, velocity and orientation quaterion (q_out)

persistent i; %variable counting run number
persistent r_n;
persistent v_n;
persistent q;


if( isempty(i) == 0)
    i=i+1;
else

    i=0;
    % REMOVE IN FURTHER VERSIONS
    % hard-coded values of r_n and v_n -> they have to be changed in 
    % further versions, Warszawa 52.259 deg N, 21.020 deg E, 144 meters over
    % ellipsioid
    %r_n_traj_gen = [deg2rad(52.259) deg2rad(21.020) 144]';
    %v_n_0= [ 0 0 0]';

    % quaternion transformation will be used

    %q_0=[0 0 0 1]';
    
    r_n=r_n_0;
    v_n=v_n_0;
    q=q_0;
    
end

%correcting with values from kalman filter

if GPS_acquired
    del_r_n = kalman_correction(1:3);
    del_v_n = kalman_correction(4:6);
    r_n=r_n-del_r_n;
    v_n=v_n-del_v_n;
    eps_n=kalman_correction(7);
    eps_e=kalman_correction(8);
    eps_d=kalman_correction(9);
    E=[0 -eps_d eps_e;
        eps_d 0 -eps_n;
        eps_e eps_n 0];
    C_n_b=q2C(q);
    C_n_b=(eye(3,3)+E)*C_n_b;
    q=C2q(C_n_b);

end

%computation only: persistient values will be updated later

phi=r_n(1);
lam=r_n(2);
h=r_n(3);

gamma=get_gamma(phi,h);
gamma_n=[0 0 gamma]';

om_e_ie=[0 0 7.2921158*10^-5]';
a= 6378137.0;       %semi-major axis of the reference ellipsoid
f=1/298.257223563;    %flattening
e=sqrt(2*f-f^2);    %linear eccentricity of the reference ellipsoid

M=a*(1-e^2)/(1-e^2*sin(phi)^2)^(3/2);
N=a/sqrt(1-e^2*sin(phi)^2);

C_n_e=[-sin(phi)*cos(lam), -sin(phi)*sin(lam), cos(phi);
    -sin(lam), cos(lam), 0;
    -cos(phi)*cos(lam), -cos(phi)*sin(lam), -sin(phi)];

%dphi=v_n(1);
%dlam=v_n(2);

%D^-1
vel2pos=[1/(M+h) 0 0 ;
    0 1/((N+h)*cos(phi)) 0;
    0 0 -1];

dr_n=vel2pos*v_n;

dphi=dr_n(1);
dlam=dr_n(2);

om_n_en=[dlam*cos(phi) -dphi -dlam*sin(phi)]';
om_n_ie=C_n_e*om_e_ie;
om_n_in=om_n_en+om_n_ie;
C_n_b=q2C(q);
C_b_n=C_n_b';
om_b_in=C_b_n*om_n_in;
%om_b_nb=om_b_ib-om_b_in;

%time difference assumed constant: might however be variale in later
%stages of development

%updating values for the next iteration
del_theta_b_nb=om_b_ib*ins_del_t-C_b_n*(om_n_ie+om_n_en)*ins_del_t;
del_theta_x=del_theta_b_nb(1);
del_theta_y=del_theta_b_nb(2);
del_theta_z=del_theta_b_nb(3);
del_theta=norm(del_theta_b_nb);

del_v_b_n=-acc_b*ins_del_t;    %not explicitly stated in paper
sculling=[1 del_theta_z/2 -del_theta_y/2;
    -del_theta_z/2 1 del_theta_x/2;
    del_theta_y/2 -del_theta_x/2 1];
del_v_n_f=C_n_b*sculling*del_v_b_n;

del_v_n = del_v_n_f - cross((2*om_n_ie+om_n_en),v_n*ins_del_t) + gamma_n*ins_del_t;
v_n_new= v_n + del_v_n;


r_n=r_n+0.5*vel2pos*(v_n_new+v_n)*ins_del_t;
v_n=v_n_new;

%There sould be inequality, however I'm not yet sure what kind of
if(del_theta~=0)
    s=2/del_theta*sin(del_theta/2);
    c=2*(cos(del_theta/2)-1);

    %something was odd here in the last line state d in paper : corrected
    q_del_theta_mat=[c s*del_theta_z -s*del_theta_y s*del_theta_x;
        -s*del_theta_z c s*del_theta_x s*del_theta_y;
        s*del_theta_y -s*del_theta_x c s*del_theta_z;
        -s*del_theta_x -s*del_theta_y -s*del_theta_z c ]; 

    q=q+0.5*q_del_theta_mat*q;
    q=q/norm(q);
else
    disp('del_theta yields zero, something might have gone wrong')
end

r_n_out=r_n;
v_n_out=v_n;
q_out=q; %normalisation


if abs(v_n(3))>1
%   disp('ziemniak') 
end


end

