% M=200;
% r=0.1;
Sigma_hat=sdpvar(2,2);
mu_hat=sdpvar(2,1);
sdpvar gamma z z1 y0 riskr;
x=sdpvar(2,1);
y=sdpvar(2,1);
Y=sdpvar(2,2);
Z=sdpvar(2,2);


obj=z1+1/(1-alpha)*(y0+gamma*(theta^2-mu_hat'*mu_hat-trace(Sigma_hat'*Sigma_hat))+z+trace(Z));
const=[[gamma*eye(2)-Y y+gamma*mu_hat;y'+gamma*mu_hat' z]>=0,[gamma*eye(2)-Y gamma*Sigma_hat; gamma*Sigma_hat Z]>=0,...
    [Y+eye(2) y-x; y'-x' y0+x'*x+z1]>=0, [Y y;y' y0]>=0, Z>=0, gamma>=0, z>=0];
options=sdpsettings('solver','mosek');
controller=optimizer(const,obj,options,{x,Sigma_hat,mu_hat, riskr},obj);




% x1(1,:)=linspace(0,5,M);
% x1(2,:)=linspace(0,5,M);
% [X11,X12]=meshgrid(x1(1,:),x1(2,:));
% Sigma01=0.003;
% Sigma02=0.002;
% X01=3;
% X02=2.5;
% 
% for n=1:M
%     for m=1:M
%         [sol(n,m),diagnos(n,m)]=controller({[X11(n,m);X12(n,m)],diag([Sigma01,Sigma02]),[X01;X02]});
%     end
% end
