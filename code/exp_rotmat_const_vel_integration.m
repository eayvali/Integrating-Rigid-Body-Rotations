function R_next = exp_rotmat_const_vel_integration(R,t,delt)
w=get_w(t);
what=[   0, -w(3), w(2);
      w(3),    0 ,-w(1);
     -w(2),  w(1),   0];
 R_next=R*expm(what*delt);
end