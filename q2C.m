function [ C_n_b ] = q2C( q )
%q2C functions takes a quaternion and returns the matrix that can be used 
%   to express vectors in the main reference frame

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

C_n_b= [(q1^2-q2^2-q3^2+q4^2), 2*(q1*q2-q3*q4), 2*(q1*q3+q2*q4);
    2*(q1*q2+q3*q4), (q2^2-q1^2-q3^2+q4^2), 2*(q2*q3-q1*q4);
    2*(q1*q3-q2*q4), 2*(q2*q3+q1*q4), (q3^2-q1^2-q2^2+q4^2)
];

end

