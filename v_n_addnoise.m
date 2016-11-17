function [ v_n_gps ] = v_n_addnoise( v_n_ref, v_gps_noise, del_t )
%V_N_ADDNOISE Adds proper noise to the velocity
%   We assume vertical velocity to be always zero (and its noise)

noise=wgn(2,1,v_gps_noise^2,'linear');
noise=[noise;0];

v_n_gps=v_n_ref+noise;

end

