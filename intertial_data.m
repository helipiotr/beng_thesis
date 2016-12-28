function [alpha, r_n, v_n, acc_b, om_b_ib ] = intertial_data( t , r_n_0)
%INERTIAL_DATA This function generates inertial data for a given time
%  written with intent of using with analytic 2D trajectory
%persistent alpha_0;
a=30;
c=15;
b=0.1;
d=2*b;
N_test=6*10^6; %test 'earth radius'
r_n=[a*cos(b*t) c*sin(d*t) 0]'/N_test+r_n_0; %an arbitrarily chosen function
phi=r_n(1);
lam=r_n(2);
h=r_n(3);
dr_n=[-a*b*sin(b*t) c*d*cos(d*t) 0]'/N_test;
ddr_n=[-a*b^2*cos(b*t) -c*d^2*sin(d*t) 0]'/N_test;

om_e_ie=[0 0 7.2921158*10^-5]';
a_geo= 6378137.0;       %semi-major axis of the reference ellipsoid
f=1/298.257223563;    %flattening
e=sqrt(2*f-f^2);    %linear eccentricity of the reference ellipsoid
N=a_geo/sqrt(1-e^2*sin(phi)^2);
M=a_geo*(1-e^2)/(1-e^2*sin(phi)^2)^(3/2);

dphi=dr_n(1);
dlam=dr_n(2);
dh=dr_n(3);

gamma=get_gamma(phi, h);
gamma_n=[0 0 gamma]';

D=[(M+h) 0 0 ;
    0 (N+h)*cos(phi) 0;
    0 0 -1];
dD=[dh 0 0 ;
    0 dh*cos(phi)-dphi*(N+h)*sin(phi) 0;
    0 0 0];
v_n= D*dr_n;
%using the chain differentiation rule
dv_n= dD*dr_n + D*ddr_n;

alpha= atan2(v_n(2),v_n(1));
C_n_b=rotz(rad2deg(alpha));
C_b_n=C_n_b';

C_n_e=[-sin(phi)*cos(lam), -sin(phi)*sin(lam), cos(phi);
    -sin(lam), cos(lam), 0;
    -cos(phi)*cos(lam), -cos(phi)*sin(lam), -sin(phi)];

om_n_ie=C_n_e*om_e_ie;
om_n_en=[dlam*cos(phi) -dphi -dlam*sin(phi)]';

acc_b=C_b_n*(-dv_n+cross(2*om_n_ie+om_n_en,v_n)+gamma_n);

om_n_nb=[0 0 (dv_n(2)*v_n(1)-dv_n(1)*v_n(2))/norm(v_n)^2 ]';
om_n_bn=-om_n_nb;

om_b_ib=C_b_n*(om_n_ie+om_n_en-om_n_bn);



end

