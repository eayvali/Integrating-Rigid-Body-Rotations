function dq = f_q_dot(t,q)
w=get_w(t);

%Skew-symmetric matrix of angular velocities:
W=  [ 0,-w(1),-w(2),-w(3);
w(1),  0, w(3),-w(2);
w(2),-w(3),  0, w(1);
w(3), w(2),-w(1),  0];
k=0.1;
c=k*(1-q*q');
dq=0.5*(W*q')'+c*q; % time derivative of quaternion
end