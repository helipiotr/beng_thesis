function [ q ] = C2q( C )
%C2q Function changing rotation matrix into an appropriate quaternion
 
%Wrong method: doesn't really utilise the Euler angles
%{
q1=0.5*(C(3,2)-C(2,3))/sqrt(1+C(1,1)+C(2,2)+C(3,3));
q2=0.5*(C(1,3)-C(3,1))/sqrt(1+C(1,1)+C(2,2)+C(3,3));
q3=0.5*(C(2,1)-C(1,2))/sqrt(1+C(1,1)+C(2,2)+C(3,3));
q4=0.5*sqrt(1+C(1,1)+C(2,2)+C(3,3));
q=[q1;q2;q3;q4];
%}

q(4,1) = 0.5*sqrt(abs(1 + C(1,1) + C(2,2) + C(3,3)));
q(1,1) = 1/(4*q(4,1))*(C(3,2) - C(2,3));
q(2,1) = 1/(4*q(4,1))*(C(1,3) - C(3,1));
q(3,1) = 1/(4*q(4,1))*(C(2,1) - C(1,2));

end

