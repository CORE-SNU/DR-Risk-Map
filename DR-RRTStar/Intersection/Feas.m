function [t,control] = Feas(from,to)
global L_r minAngle maxAngle minVel maxVel Ts;
v1=(to(1)-from(1))/(Ts*cos(from(3)));
v2=(to(2)-from(2))/(Ts*sin(from(3)));
    if isnan(v2) && to(2)==from(2)
        v2=v1;
    end
    if isnan(v1) && to(1)==from(1)
        v1=v2;
    end
if abs(v1-v2)<=10e-6 && minVel-v1<=10e-6 && v1-maxVel<=10e-6
    delta=atan((to(3)-from(3))*L_r/(Ts*v1));
    if minAngle-delta<=10e-6 && delta-maxAngle<=10e-6
        t=1;
        control=[v1;delta];
    else
        t=0;
        control=[];
    end
else
    t=0;
    control=[];
end
end