function [ prod ] = multquat( q, p )
%MULTQUAT Quaterion multiplication function
%   Basic quaternion multiplication scheme required for 
%	operations in other functions of this programme

a=q(1); 
b=q(2);
c=q(3);
d=q(4);

%{
e=p(1);
f=p(2);
g=p(3);
h=p(4);
%}

prod=[a -b -c -d;
    b a -d c ;
    c d a -b;
    d -c b a]*p;

end

