% Solver generation for DR-CVaR upper bound in (15)

Sigma_hat = sdpvar(2,2); % Not the covariance matrix, but its Cholesky decomposition!
mu_hat = sdpvar(2,1); 
sdpvar gamma z z1 y0;
x = sdpvar(2,1);
y = sdpvar(2,1);
Y = sdpvar(2,2);
Z = sdpvar(2,2);

obj=z1+1/(1-alpha)*(y0+gamma*(theta^2-mu_hat'*mu_hat-trace(Sigma_hat'*Sigma_hat))+z+trace(Z));

const=[[gamma*eye(2)-Y y+gamma*mu_hat;y'+gamma*mu_hat' z]>=0,[gamma*eye(2)-Y gamma*Sigma_hat; gamma*Sigma_hat Z]>=0,...
       [Y+eye(2) y-x; y'-x' y0+x'*x+z1]>=0, [Y y;y' y0]>=0, Z>=0, gamma>=0, z>=0];

controller=optimizer(const,obj,sdpsettings('solver','mosek'),{x,Sigma_hat,mu_hat},obj);