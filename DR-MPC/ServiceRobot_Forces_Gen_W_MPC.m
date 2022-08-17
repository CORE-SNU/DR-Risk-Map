umin = [-4, -4];     umax = [4, 4];
xmin = [0, 0, -2, -2]; xmax = [10, 10, 2, 2];
%% Model
model = {};
model.N = K;
model.nvar = nx+nu+L;
model.neq = nx;
model.npar(1) = ny;
model.npar(2:K) = ny+N*ny*L+Nobs*ny;
model.nh(1)=0;
model.nh(2:K) = L+Nobs;

model.objective = @(z,p) z(1:nu)'*R*z(1:nu) + (z(nu+L+1:nu+L+ny)-p(1:2))'*Q*(z(nu+L+1:nu+L+ny)-p(1:2));
model.objectiveN = @(z,p) (z(nu+L+1:nu+L+ny)-p(1:2))'*Q*(z(nu+L+1:nu+L+ny)-p(1:2));
% model.LSobjectiveN = @(z,p) (z(nu+1:nu+ny)-p(1:2))'*sqrt(Q);
model.eq = @dynamics;
model.xinitidx = (nu+L+1):(nu+L+nx);
model.E = [zeros(nx,nu), zeros(nx,L), eye(nx,nx)];
model.ub = [umax, inf(1,L), xmax];
model.lb= [umin, -inf(1,L), xmin];
%     model.ub = [umax, xmax, delta*ones(1,L)];
%     model.lb= [umin, xmin, -inf(1,L)];
for k=2:model.N
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
codeoptions.nlp.solvemethod = 'PDIP';

% codeoptions.forcenonconvex = 1;
% codeoptions.nlp.TolStat = 1e-8;
% codeoptions.nlp.TolEq = 1e-8;
% codeoptions.nlp.TolComp = 1e-8;
% codeoptions.nlp.TolIneq = 1e-8;

% codeoptions.sqp_nlp.maxqps = 1;
% codeoptions.nlp.bfgs_init = 0.5*eye(model.nvar);
% codeoptions.nlp.hessian_approximation = 'bfgs';
FORCES_NLP(model, codeoptions);

%%
function f = dynamics(z)
global Ts nu nx L
    u = z(1:nu);
    x = z(nu+L+1:nu+L+nx);
    f = [x(1)+Ts*x(3)+Ts^2/2*u(1);
        x(2)+Ts*x(4)+Ts^2/2*u(2);
        x(3)+Ts*u(1);
        x(4)+Ts*u(2)];
end

function f = inequality(z,p)
global nu nx ny L W veh_rad alpha Nobs N
% obs = [4.5 8.5 4.7;1 4 5.5];
% b1 = [5.5; -4.5; 4.5; -3.5];
% A1 = [1 0; -1 0; 0 1; 0 -1];
P = inv([0.7^2 0;0 0.8^2]);
w_hat{1} = [1.5;4.5];
w_hat{2} = [5.8800;5.7000];
w_hat{3} = [5.700;5.8800];

    f = [];
    x = z(nu+L+1:nu+L+ny);
%     rho = z(nu+nx+1:nu+nx+4);
    for l=1:L
        z1{l} = z(nu+l);
        w_hat{l} = reshape(p(ny+Nobs*2+(l-1)*ny*N+1:ny+Nobs*2+l*ny*N),[ny N]);
        s{l}=[];
        for n=1:N
        s{l} = [s{l} subplus((W+veh_rad)^2-(x(1:2)-w_hat{l}(:,n))'*(x(1:2)-w_hat{l}(:,n))-z1{l})];
        end
        f = [f; z1{l}+1/(1-alpha)*1/N*sum(s{l})];
   end
    for i=1:Nobs
        obs{i} = p(ny+(i-1)*ny+1:ny+i*2);
        f=[f; (x-obs{i})'*P*(x-obs{i})];
    end
end
