function [ phi ] = cal_phi( i, k, l )
%CAL_PHI returns proper phi-value to optimise using least squares method

i=int32(i);
k=int32(k);

if mod(i,3)==1
    l_index=idivide(i,3,'floor')+1;
    phi=l(k,l_index);

elseif mod(i,3) == 2
    l_index=idivide(i,3,'floor')+1;
    l_tmp=l(k,l_index);
    phi=l_tmp^2;
else
    phi=1/i;
end
end

