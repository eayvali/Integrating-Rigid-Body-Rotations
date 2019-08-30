function dq = f_q_unit_dot(t,q)
w=get_w(t);

%Skew-symmetric matrix of angular velocities:
W=  [ 0,-w(1),-w(2),-w(3);
w(1),  0, w(3),-w(2);
w(2),-w(3),  0, w(1);
w(3), w(2),-w(1),  0];
dq=0.5*(W*q')'; % time derivative of unit quaternion

end