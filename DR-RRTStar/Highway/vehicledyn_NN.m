function [states,A,B]=vehicledyn_NN(u,x,w,b,mean_train,std_train)
    nx = size(x,2);
    nu = size(u,2);
    a1 = ([x u]-mean_train)./std_train*w{1}+b{1};
    h1 = subplus(a1);
    a2 = h1*w{2}+b{2};
    h2 = subplus(a2);
    a3 = h2*w{3}+b{3};
    h3 = subplus(a3);
    states = x+h3*w{4}+b{4};
    deriv = w{4}'*diag(subplus(sign(a3)))*w{3}'*diag(subplus(sign(a2)))*w{2}'*diag(subplus(sign(a1)))*w{1}'./std_train;
    A = eye(nx)+deriv(:,1:nx);
    B = deriv(:,nx+1:nx+nu);
end