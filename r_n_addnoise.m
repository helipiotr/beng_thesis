function [ r_n_gps ] = r_n_addnoise( r_n_ref, r_gps_noise )
%UNTITLED This function takes an ideal trajectory and adds noise to it
%   warning: this noise is actually random (no seed)

phi=r_n_ref(1);
h=r_n_ref(3);

a= 6378137.0;       %semi-major axis of the reference ellipsoid
f=1/298.257223563;    %flattening
e=sqrt(2*f-f^2);    %linear eccentricity of the reference ellipsoid

M=a*(1-e^2)/(1-e^2*sin(phi)^2)^(3/2);
N=a/sqrt(1-e^2*sin(phi)^2);

noise=[1/(M+h) 0 0 ; 0 1/((N+h)*cos(phi)) 0; 0 0 1]*...
    wgn(3,1,r_gps_noise^2,'linear');

r_n_gps=r_n_ref+noise;

end

