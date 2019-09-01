for i=1:length(R_from_unit_q_RK4(:,1))
%degrees
R_rel_R_exp=reshape(R_exp(i,:),3,3)'*reshape(R_from_unit_q_RK4(i,:),3,3);
error_R_exp(i)=norm(0.5*sqrt(2)*logm(R_rel_R_exp),'fro')*180/pi;

R_rel_R_from_nonunit_q_RK4=reshape(R_from_nonunit_q_RK4(i,:),3,3)'*reshape(R_from_unit_q_RK4(i,:),3,3);
error_R_from_nonunit_q_RK4(i)=norm(0.5*sqrt(2)*logm(R_rel_R_from_nonunit_q_RK4),'fro')*180/pi;

R_rel_R_from_nonunit_q_zeroc_RK4=reshape(R_from_nonunit_q_zeroc_RK4(i,:),3,3)'*reshape(R_from_unit_q_RK4(i,:),3,3);
error_R_from_nonunit_q_zeroc_RK4(i)=norm(0.5*sqrt(2)*logm(R_rel_R_from_nonunit_q_zeroc_RK4),'fro')*180/pi;

R_rel_R_from_q_exp=reshape(R_from_q_exp(i,:),3,3)'*reshape(R_from_unit_q_RK4(i,:),3,3);
error_R_from_q_exp(i)=norm(0.5*sqrt(2)*logm(R_rel_R_from_q_exp),'fro')*180/pi;

R_rel_R_rk4=reshape(R_rk4(i,:),3,3)'*reshape(R_from_unit_q_RK4(i,:),3,3);
error_R_rk4(i)=    norm(0.5*sqrt(2)*logm(R_rel_R_rk4),'fro')*180/pi;
   
end
%%
close all
delt=0.1;
t_max=10000;
time=0:delt:t_max;
figure
h=loglog(time,error_R_exp,'r',time,error_R_from_q_exp,'k',time,error_R_from_nonunit_q_RK4,'b',time,error_R_from_nonunit_q_zeroc_RK4,'c', time,error_R_rk4,'m');
legend(h,'R exponential update','unit quaternion exponential update','RK4 nonunit q (c\neq0)',  'RK4, nonunit q (c=0)','RK4, R')
title('rotation error')
