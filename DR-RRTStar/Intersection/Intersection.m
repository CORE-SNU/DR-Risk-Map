function [xout,yout]=Intersection(m,b,o)
a=1+m^2;
d=2*m*b-2*o(1)-2*m*o(2);
c=o(1)^2+o(2)^2-2*o(2)*b+b^2-o(3)^2;
D=d^2-4*a*c;
if D<0
    xout=[NaN NaN];
    yout=[NaN NaN];
else
x1=(-d+sqrt(D))/(2*a);
x2=(-d-sqrt(D))/(2*a);
y1=m*x1+b;
y2=m*x2+b;
xout=[x1 x2];
yout=[y1 y2];
end
end