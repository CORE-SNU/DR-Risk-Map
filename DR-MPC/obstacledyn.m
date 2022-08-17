function f = obstacledyn(x,u,Ts,K)
if nargin<=3
    K=size(u,2);
end
if size(u,2)<K
    u=repmat(u,1,K);
end

nx=size(x,1);
nu=size(u,1);
A = [1 0;0 1];
B = [Ts 0; 0 Ts];
f = x;
for k=1:K
   f(:,k+1) =  A*f(:,k)+B*u(:,k);
end
end