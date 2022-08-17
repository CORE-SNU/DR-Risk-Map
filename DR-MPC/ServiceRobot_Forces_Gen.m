umin = [-4, -4];     umax = [4, 4];
xmin = [0, 0, -2, -2]; xmax = [10, 10, 2, 2];
%% Model
model = {};
model.N = K;
model.nvar= nx+nu;
model.neq = nx;
model.npar(1) = ny;
model.npar(2:K) = ny+L*ny+L*ny*ny+Nobs*2;
model.nh(1)=0;
model.nh(2:K) = L+Nobs;
% model.nh(2:K) = L;

model.objective = @(z,p) z(1:nu)'*R*z(1:nu) + (z(nu+1:nu+ny)-p(1:2))'*Q*(z(nu+1:nu+ny)-p(1:2));
model.objectiveN = @(z,p) (z(nu+1:nu+ny)-p(1:2))'*Q*(z(nu+1:nu+ny)-p(1:2));
% model.LSobjectiveN = @(z,p) (z(nu+1:nu+ny)-p(1:2))'*sqrt(Q);
model.eq = @dynamics;
model.xinitidx = (nu+1):(nu+nx);
% model.E = [zeros(nx,nu), eye(nx,nx)];
model.E = [zeros(nx,nu), eye(nx,nx)];
model.ub{1} = [umax, xmax];
model.lb{1} = [umin, xmin];

for k=2:model.N
    model.ub{k} = [umax, xmax];
    model.lb{k} = [umin, xmin];
    model.ineq{k} = @inequality;
    model.hl{k} = [-inf(1,L), ones(1,Nobs)];
    model.hu{k} = [delta*ones(1,L), inf(1,Nobs)];
end


codeoptions = getOptions(['FORCESNLP',str]);
codeoptions.maxit = 20000;  
codeoptions.optlevel = 3;
codeoptions.cleanup = 0;
codeoptions.timing = 1;
codeoptions.overwrite = 1;
codeoptions.BuildSimulinkBlock = 0;
codeoptions.printlevel = 1;
codeoptions.nlp.ad_tool = 'casadi-3.5.1';

% codeoptions.nlp.TolStat = 1e-8;
% codeoptions.nlp.TolEq = 1e-8;
% codeoptions.nlp.TolComp = 1e-8;
% codeoptions.nlp.TolIneq = 1e-8;

% codeoptions.sqp_nlp.maxqps = 1;
% codeoptions.nlp.bfgs_init = 0.5*eye(model.nvar);
% codeoptions.nlp.hessian_approximation = 'bfgs';
FORCES_NLP(model, codeoptions);

%%

function obj = LSobj(z,p)
    global nu nx ny Q R
 obj = [(z(nu+1:nu+ny)-p(1:2)); z(1:nu)]'*sqrt([Q zeros(nu,ny);zeros(ny,nu) R].*2);
end
function f = dynamics(z)
global Ts nu nx
    u = z(1:nu);
    x = z(nu+1:nu+nx);
    f = [x(1)+Ts*x(3)+Ts^2/2*u(1);
        x(2)+Ts*x(4)+Ts^2/2*u(2);
        x(3)+Ts*u(1);
        x(4)+Ts*u(2)];
end

function f = inequality(z,p)
global nu nx ny Nn L w b W veh_rad mean_train std_train Nobs
% obs = [4.5 8.5 4.7;1 4 5.5];
% b1 = [5.5; -4.5; 4.5; -3.5];
% A1 = [1 0; -1 0; 0 1; 0 -1];
% obs1 = [5;4];
% obs2 = [6.6;4];
P = inv([0.7^2 0;0 0.8^2]);
    f = [];
    x = z(nu+1:nu+ny);
%     rho = z(nu+nx+1:nu+nx+4);
    for l=1:L
        mu_hat{l} = p(ny+(l-1)*ny+1:ny+l*ny);
        Sigma_hat{l} = p(ny+L*ny+(l-1)*ny*ny+1:ny+L*ny+l*ny*ny);        
        f = [f; subplus(subplus(subplus((([x(1:2);Sigma_hat{l}(1);Sigma_hat{l}(2);...
                Sigma_hat{l}(4);mu_hat{l}]'-mean_train)./std_train)*w{1}+b{1})*w{2}+b{2})*w{3}+b{3})*w{4}+b{4}];
    end
    for i=1:Nobs
        obs{i} = p(ny+L*ny+L*ny*ny+(i-1)*2+1:ny+L*ny+L*ny*ny+i*2);
        f=[f; (x-obs{i})'*P*(x-obs{i})];
    end
end
