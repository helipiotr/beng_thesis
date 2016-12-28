function [ F , J ] = f_g( x , l , gamma)
%F_G This finction computes right-hand minimised value and Jacobian
% %TODO implement Jacobian
% x = [bbx bqy .. sgz]'
bgx=x(1);
bgy=x(2);
bgz=x(3);
sgx=x(4);
sgy=x(5);
sgz=x(6);

for i=1:6
    F(i,1)=(l(i,1)-bgx)^2/(1+sgx)^2+(l(i,2)-bgy)^2/(1+sgy)^2+...
        (l(i,3)-bgz)^2/(1+sgz)^2 - gamma^2;
end


end

