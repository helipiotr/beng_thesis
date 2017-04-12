function [x_kplus_kplus,del_r_n,del_v_n] = ... 
    kalman_gps_ins(r_n_gps, v_n_gps, r_n_ins, v_n_ins, acc_b, q, ...
    ins_del_t, gps_del_t, acc_noise, acc_bias, gyro_noise, r_gps_noise,...
    v_gps_noise)
%KALMAN_GPS_INS Kalman filter for BEng diploma project
%   Kalman filter written for specific application of integrating INS
%   and GPS navigation using the difference method

persistent i; %variable counting run number
persistent P_k_k;
persistent x_k_k;



om_e=7.2921158*10^-5;
om_e_ie=[0 0 om_e]';
a= 6378137.0;       %semi-major axis of the reference ellipsoid
f=1/298.257223563;    %flattening
e=sqrt(2*f-f^2);    %linear eccentricity of the reference ellipsoid

%INS should be good enough as estimate: we also utilise q
r_n=r_n_ins;
v_n=v_n_ins;

phi=r_n(1);
lam=r_n(2);
h=r_n(3);

M=a*(1-e^2)/(1-e^2*sin(phi)^2)^(3/2);
N=a/sqrt(1-e^2*sin(phi)^2);
R=sqrt(M*N);

vel2pos=[1/(M+h) 0 0 ;
    0 1/((N+h)*cos(phi)) 0;
    0 0 -1];

dr_n=vel2pos*v_n;

dphi=dr_n(1);
dlam=dr_n(2);

gamma=get_gamma( phi, h ); %assuming actual model

vn=v_n(1);
ve=v_n(2);
vd=v_n(3);


C_n_e=[-sin(phi)*cos(lam), -sin(phi)*sin(lam), cos(phi);
    -sin(lam), cos(lam), 0;
    -cos(phi)*cos(lam), -cos(phi)*sin(lam), -sin(phi)];

om_n_ie=C_n_e*om_e_ie;
om_n_en=[dlam*cos(phi) -dphi -dlam*sin(phi)]';
om_n_in=om_n_ie+om_n_en;


Frr=[0 0 -vn/(M+h)^2;
    ve*sin(phi)/((N+h)*cos(phi)^2) 0 -ve/((N+h)^2*cos(phi));
    0 0 0];


Frv=[1/(M+h) 0 0 ;
    0 1/((N+h)*cos(phi)) 0;
    0 0 -1];


Fvr=[-2*ve*om_e*cos(phi)-ve^2/((N+h)*cos(phi)^2), 0,...
    -vn*vd/(M+h)^2+ve^2*tan(phi)/(N+h)^2;
    
    2*om_e*(vn*cos(phi)-vd*sin(phi))+ve*vn/((N+h)*cos(phi)^2), 0, ...
    -ve*vd/(N+h)^2-vn*ve*tan(phi)/(N+h)^2;
    
    2*ve*om_e*sin(phi), 0, ...
    ve^2/(N+h)^2 + vn^2/(M+h)^2-2-2*gamma/(R+h)]; %assuming precise gamma
    
Fvv=[vd/(M+h), -2*om_e*sin(phi)-2*ve*tan(phi)/(N+h), vn/(M+h);
    
    2*om_e*sin(phi)+ve*tan(phi)/(N+h), (vd + vn*tan(phi))/(N+h),...
    2*om_e*cos(phi)+ve/(N+h);
    
    -2*vn/(M+h), -2*om_e*cos(phi) - 2*ve/(N+h), 0];

Fer=[ -om_e*sin(phi), 0, -ve/(N+h)^2;
    0 , 0 , vn/(M+h)^2;
    -om_e*cos(phi) - ve/((N+h)*cos(phi)^2), 0 , ve*tan(phi)/(N+h)^2];

Fev=[0, 1/(N+h) , 0;
    -1/(M+h), 0 , 0;
    0 , -tan(phi)/(N+h), 0];

C_n_b=q2C(q);
f_n=-C_n_b*acc_b;

F=[Frr Frv zeros(3,3);
    Fvr Fvv skew(f_n);
    Fer Fev -skew(om_n_in)];

G=[zeros(3,3) zeros(3,3);
    C_n_b zeros(3,3);
    zeros(3,3) -C_n_b];

%standard deviations of accelerometers and gyroscopes

sdev_ax = acc_noise^2/ins_del_t/2;%*100;
sdev_ay = acc_noise^2/ins_del_t/2;%*100;
sdev_az = acc_noise^2/ins_del_t/2;%*100;

sdev_omx = gyro_noise^2/ins_del_t/2;
sdev_omy = gyro_noise^2/ins_del_t/2;
sdev_omz = gyro_noise^2/ins_del_t/2;
%
sdev_ins=[sdev_ax+acc_bias^2 sdev_ay+acc_bias^2 sdev_az+acc_bias^2 ...
    sdev_omx sdev_omy sdev_omz];

%sdev_ins=[6.04*10^-5 1.02*10^-4 2.63*10^-4 ...
%    7.5*10^-5 1.31*10^-7 1.14*10^-7];


%sdev_ins=[0.0016 0.0032 0.0109...
%    1.13*10^-6 5.225*10^-6 3.5192*10^-6];
Q=diag(sdev_ins);


%we can scale Q, so that it trusts GPS measurements more
%this step is still discussable
Q=40*Q;

aux=[(M+h),0,0;
    0, (N+h)*cos(phi), 0;
    0, 0, 1];


H=[aux,zeros(3,3),zeros(3,3);
    zeros(3,3),eye(3,3),zeros(3,3)];

%new measurement
z_kplus_kplus=[(M+h)*(r_n_ins(1)-r_n_gps(1));
    (N+h)*cos(phi)*(r_n_ins(2)-r_n_gps(2));
    r_n_ins(3)-r_n_gps(3);
    v_n_ins-v_n_gps];

%gps_noise
sdev_phi=(r_gps_noise)^2;%/(M+h)^2;% to match measurement transformation
sdev_lam=(r_gps_noise)^2;%/((N+h)*cos(phi))^2;
sdev_h=r_gps_noise^2;

sdev_vn = v_gps_noise^2;
sdev_ve = v_gps_noise^2;
sdev_vd = v_gps_noise^2; %we are pretty certain there is no movement in the z axis

sdev_gps=[sdev_phi sdev_lam sdev_h sdev_vn sdev_ve sdev_vd];

Rk=diag(sdev_gps);

%Kalman filter calculations

if( isempty(i) == 0)
    i=i+1;
else

    i=0;
    %We assume the initial orientation is well known
    P_0=diag([sdev_phi sdev_lam sdev_h ...
        sdev_vn sdev_ve sdev_vd 0.01 0.01 0.01]);
    P_k_k=P_0;
    %Kalman filter state vector
    x_0=zeros(9,1);
    x_k_k=x_0;
end

Phik=eye(9,9)+F*gps_del_t;

%Fk=eye(9)+50*F*del_t;

Qk=Phik*G*Q*G'*Phik'*gps_del_t;

%State prediction covariance error matix
P_kplus_k=Phik*P_k_k*Phik'+Qk;

%H has no index due to negligible variance
K_kplus=P_kplus_k*H'*(H*P_kplus_k*H'+Rk)^-1;

%State estimate covariance error matix
P_kplus_kplus=(eye(9)-K_kplus*H)*P_kplus_k;

x_k_k=zeros(9,1);

x_kplus_k=Phik*x_k_k;

z_kplus_k=H*x_kplus_k;

del_z_kplus_k=z_kplus_kplus-z_kplus_k;

x_kplus_kplus=x_kplus_k+K_kplus*del_z_kplus_k;

%assigning persistent values fo next iteration

%if i==100
%    disp('ziemniak')
%end

%if abs(vd)>1
%   disp(i) 
%end

P_k_k=P_kplus_kplus;
x_k_k=x_kplus_kplus;

%we output state vectors as state error
del_r_n = x_k_k(1:3);
del_v_n = x_k_k(4:6);



end

