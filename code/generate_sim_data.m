%% Generate angular velocity data
clc;close all;clear;
delt=0.1;
t_max=1000;
i=1;
for t=0:delt:t_max
w(i,:)=get_w(t);
time(i)=t;
i=i+1;
end
plot(time,w)
title('angular velocity profile')
%% ---------------- QUATERNION----------------------- %%

%% RK4 nonunit quaternion integration
disp('RK4 nonunit quaternion integration:')
time=0:delt:t_max;
q_init=[1,0,0,0];
tic
q_rk4 = RK4(@f_q_dot,0,delt,t_max,q_init);
toc
%
% figure
% plot(time,q_rk4)
% title('quat nonunit RK4')
figure;
loglog(time,abs(sqrt(sum(q_rk4.^2,2))-ones(length(time),1)))
grid on
title('nonunit RK4 : limit norm drift (c \neq 0)')

%% RK4 nonunit quaternion integration with c=0
disp('RK4 nonunit quaternion integration (c=0):')
time=0:delt:t_max;
q_init=[1,0,0,0];
tic
q_zeroc_rk4 = RK4(@f_q_unit_dot,0,delt,t_max,q_init);
toc
%
% figure
% plot(time,q_rk4)
% title('quat nonunit RK4')
figure;
loglog(time,abs(sqrt(sum(q_zeroc_rk4.^2,2))-ones(length(time),1)))
grid on
title('nonunit RK4 : norm drift (c=0)')

%% RK4 unit quaternion integration
disp('RK4 unit quaternion integration')
time=0:delt:t_max;
q_init=[1,0,0,0];
tic
q_unit_rk4 = RK4_normalized(@f_q_unit_dot,0,delt,t_max,q_init);
toc
%
% figure
% plot(time,q_unit_rk4)
% title('unit quat RK4')
figure;
loglog(time,abs(sqrt(sum(q_unit_rk4.^2,2))-ones(length(time),1)))
grid on
title('unit quat RK4:norm drift')%expect none


%% Quat exponential update
disp('Quat exponential update')
i=1;
q_t=[1,0,0,0];
tic
for t=0:delt:t_max
q_next=exp_quat_const_vel_integration(q_t,t,delt);
if q_next(1)<0
   q_next=-q_next; 
end
q_t=q_next;
qexp_t(i)=t;
qexp(i,:)=q_next;
i=i+1;
end
toc

% figure
% plot(qexp_t,qexp)
% title('quat exponential update')
figure;
loglog(qexp_t,abs(sqrt(sum(qexp.^2,2))-ones(length(qexp_t),1)))
grid on
title('quat exponential update  : norm drift')

%% Convert quaternions to rotation matrices for comparison
for i=1:length(time)
    %exponential update
    q=qexp(i,1:4); 
    q1=q(1); q2=q(2); q3=q(3); q4=q(4);
    R=[q1^2+q2^2-q3^2-q4^2,2*(q2*q3-q1*q4),2*(q2*q4+q1*q3);
            2*(q2*q3+q1*q4),q1^2+q3^2-q2^2-q4^2,2*(q3*q4-q1*q2);
            2*(q2*q4-q1*q3),2*(q3*q4+q1*q2),q1^2+q4^2-q2^2-q3^2];
    R_from_q_exp(i,1:9)=reshape(R,1,9);
    det_R_from_q_exp(i)=det(R);
    
    %unit quat integration    
    q=q_unit_rk4(i,1:4);
    q1=q(1); q2=q(2); q3=q(3); q4=q(4);
    R=[q1^2+q2^2-q3^2-q4^2,2*(q2*q3-q1*q4),2*(q2*q4+q1*q3);
            2*(q2*q3+q1*q4),q1^2+q3^2-q2^2-q4^2,2*(q3*q4-q1*q2);
            2*(q2*q4-q1*q3),2*(q3*q4+q1*q2),q1^2+q4^2-q2^2-q3^2]/(q1^2+q2^2+q3^2+q4^2);
    R_from_q_unit_RK4(i,1:9)=reshape(R,1,9);  
    det_R_from_q_unit_RK4(i)=det(R);
    
    %nonunit quat integration
    q=q_rk4(i,1:4);
    q1=q(1); q2=q(2); q3=q(3); q4=q(4);
    R=[q1^2+q2^2-q3^2-q4^2,2*(q2*q3-q1*q4),2*(q2*q4+q1*q3);
            2*(q2*q3+q1*q4),q1^2+q3^2-q2^2-q4^2,2*(q3*q4-q1*q2);
            2*(q2*q4-q1*q3),2*(q3*q4+q1*q2),q1^2+q4^2-q2^2-q3^2]/(q1^2+q2^2+q3^2+q4^2);
    R_from_q_RK4(i,1:9)=reshape(R,1,9);  
    det_R_from_q_RK4(i)=det(R);
    
    %nonunit quat integration with c=0
    q=q_zeroc_rk4(i,1:4);
    q1=q(1); q2=q(2); q3=q(3); q4=q(4);
    R=[q1^2+q2^2-q3^2-q4^2,2*(q2*q3-q1*q4),2*(q2*q4+q1*q3);
            2*(q2*q3+q1*q4),q1^2+q3^2-q2^2-q4^2,2*(q3*q4-q1*q2);
            2*(q2*q4-q1*q3),2*(q3*q4+q1*q2),q1^2+q4^2-q2^2-q3^2]/(q1^2+q2^2+q3^2+q4^2);
    R_from_q_zeroc_RK4(i,1:9)=reshape(R,1,9);  
    det_R_from_q_zeroc_RK4(i)=det(R);
end

%% ---------------- ROTATION MATRIX----------------------- %%
%% Rotation matrix exponential update
disp('Rotation matrix exponential update')
i=1;
R_t=eye(3);
%t should be same as time
tic
for t=0:delt:t_max
R_next=exp_rotmat_const_vel_integration(R_t,t,delt);
R_t=R_next;
exp_t(i)=t;
R_exp(i,:)=reshape(R_next,1,9);
det_R_exp(i)=det(R_next);
i=i+1;
end
toc

%% RK4  rotation matrix integration
disp('RK4  rotation matrix integration')
time=0:delt:t_max;
R_init=reshape(eye(3),1,9);
tic
R_rk4 = RK4(@f_R_dot,0,delt,t_max,R_init);
toc
for i=1:length(time)
det_R_rk4(i)=det(reshape(R_rk4(i,1:9),3,3)); 
end

%% Plot rotation matrix elements
% figure
% h1=plot(time,R_from_q_exp,'r',time,R_from_q_RK4,'b',time,R_rk4,'g',time,R_exp,'k',time,R_from_q_unit_RK4,'m');
% legend(h1,'quaternion exponential method','RK4 on nonunit q','RK4 on R','R exponential method','RK4 on unit q')
% title('Rotation matrix elements')
figure
h2=loglog(time,det_R_from_q_exp,'r',time,det_R_from_q_RK4,'b',time,det_R_exp,'k',time,det_R_from_q_zeroc_RK4,'c');
legend(h2,'quaternion exponential method','RK4 on nonunit q (with drift limiting)','R exponential method',  'RK4 for nonunit q (no drift limiting)')
title('determinant of rotation')
figure
h3=loglog(time,det_R_rk4,'g');
legend(h3,'RK4 on R')
title('determinant of rotation')

figure
h4=loglog(time,abs(sqrt(sum(q_rk4.^2,2))-ones(length(time),1)),'r',time,abs(sqrt(sum(q_zeroc_rk4.^2,2))-ones(length(time),1)),'b',qexp_t,abs(sqrt(sum(qexp.^2,2))-ones(length(qexp_t),1)),'k');
legend(h4,'Nonunit quaternion RK4 (c \neq 0)','Nonunit quaternion RK4 (c = 0)', 'Unit quaternion exponential update')
title('quat norm drift')






