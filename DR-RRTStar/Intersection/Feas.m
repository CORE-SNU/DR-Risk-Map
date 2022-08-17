function [t,control] = Feas(from,to)
global L minW maxW minVel maxVel Ts;
v1=(to(1)-from(1))/(Ts*cos(from(3)));
v2=(to(2)-from(2))/(Ts*sin(from(3)));
    if isnan(v2) && to(2)==from(2)
        v2=v1;
    end
    if isnan(v1) && to(1)==from(1)
        v1=v2;
    end
if abs(v1-v2)<=10e-6 && minVel-v1<=10e-6 && v1-maxVel<=10e-6
    delta=(to(3)-from(3))/Ts;
    if minW-delta<=10e-6 && delta-maxW<=10e-6
        t=1;
        control=[v1;delta];
%         [minvel, inddelta]=min(abs(delta-(vels)));
%         [minangle, indv]=min(abs(v1-(angles)));
%         [~,control_index]=find((abs(poscont(1,:)-minvel)<=10e-7) & (abs(poscont(2,:)-minangle)<=10e-7));
    else
        t=0;
        control=[];
    end
else
    t=0;
    control=[];
end
end