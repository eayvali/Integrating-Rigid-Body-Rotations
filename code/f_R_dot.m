function dR_out = f_R_dot(t,R)
%R(1,9)
%R_dot=R*w_body
R=reshape(R(1:9),3,3);
w=get_w(t);
%Skew-symmetric matrix of angular velocities:
w_hat=[   0, -w(3), w(2);
      w(3),    0 ,-w(1);
     -w(2),  w(1),   0];
dR=R*w_hat;
dR_out=reshape(dR,1,9);
end