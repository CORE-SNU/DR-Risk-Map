ys=sdpvar(3,1);
xs=sdpvar(3,1);
xolds=sdpvar(3,1);
us=sdpvar(2,1);
objs=(ys-xs)'*(ys-xs);
consts=[xs(1)==xolds(1)+Ts*us(1)*cos(xolds(3)),xs(2)==xolds(2)+Ts*us(1)*sin(xolds(3)),xs(3)==xolds(3)+Ts*us(2),minVel<=us(1)<=maxVel,minW<=us(2)<=maxW];
steercontrol=optimizer(consts,objs,sdpsettings('solver','gurobi'),{xolds,ys},{xs,objs});

