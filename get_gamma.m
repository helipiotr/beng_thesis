function [ gamma ] = get_gamma( phi, h )
%GET_GAMMA Computes gravity given phi and h
%   Detailed explanation goes here

%   Gravity computation factors
a1=9.7803267715;
a2=0.0052790414;
a3=0.0000232718;
a4=-0.0000030876910891;
a5=0.0000000043977311;
a6=0.0000000000007211;

gamma=a1*(1+a2*sin(phi)^2+a3*sin(phi)^4)+(a4+a5*sin(phi)^2)*h+a6*h;

end

