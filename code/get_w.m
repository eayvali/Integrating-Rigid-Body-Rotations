function w = get_w(t)
a=pi/2;
b=2*pi/10;
w=a.*[sin(b*t),sin(b*t+2*pi/3),sin(b*t+4*pi/3)];
% w=[1,2,3]*sin(t);
end