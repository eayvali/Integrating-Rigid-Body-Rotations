function q_next = exp_quat_const_vel_integration(q,t,delt)
w=get_w(t);
v=0.5*w*delt;
if norm(v)>eps
    exp_v=[cos(norm(v)), (sin(norm(v))/norm(v)*v')']';
%     exp_v=[cos(norm(v)),(sinc(norm(v))*v')'];
else
    exp_v=[1;0;0;0];
end  
q_next=quatmultiply(exp_v,q);
end