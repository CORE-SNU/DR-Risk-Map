%% Model
model = {};
model.N = K;
model.nvar= nx+nu;
model.neq = nx;
model.npar(1) = ny;
model.npar(2:K) = ny + L*ny + L*ny*ny + Nobs*2;
model.nh(1)=0;
model.nh(2:K) = L + Nobs;

model.objective = @(z,p) z(1:nu)'*R*z(1:nu) + (z(nu+1:nu+ny)-p(1:2))'*Q*(z(nu+1:nu+ny)-p(1:2));
model.objectiveN = @(z,p) (z(nu+1:nu+ny)-p(1:2))'*Q*(z(nu+1:nu+ny)-p(1:2));
model.eq = @dynamics;
model.xinitidx = (nu+1):(nu+nx);
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

FORCES_NLP(model, codeoptions);

%%

function obj = LSobj(z,p)
    global nu ny Q R
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
    global nu ny L w b mean_train std_train Nobs P
    f = [];
    x = z(nu+1:nu+ny);
    % Direct implementation of eqs (20)-(22)
    for l=1:L
        mu_hat{l} = p(ny+(l-1)*ny+1:ny+l*ny);
        Sigma_hat{l} = p(ny+L*ny+(l-1)*ny*ny+1:ny+L*ny+l*ny*ny); 
        f = [f; subplus(subplus(subplus((([x(1:2);Sigma_hat{l}(1);Sigma_hat{l}(2);...
                Sigma_hat{l}(4);mu_hat{l}]'-mean_train)./std_train)*w{1}+b{1})*w{2}+b{2})*w{3}+b{3})*w{4}+b{4}];
    end
    % Constraints for static obstacle avoidance
    for i=1:Nobs
        obs{i} = p(ny+L*ny+L*ny*ny+(i-1)*2+1:ny+L*ny+L*ny*ny+i*2);
        f=[f; (x-obs{i})'*P*(x-obs{i})];
    end
end
