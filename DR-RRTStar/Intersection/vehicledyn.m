function [states,A,B,invB]=vehicledyn(u,s,remove)
global Ts L
    Dx=size(s,2)-1;
    Du=size(u,2);
    K=size(u,1);
    states(1,:)=s;
    
    A=zeros(Dx,Dx-1,K);
    B=zeros(Dx,Du,K);
    for t=1:K
%         states(t+1,3)=states(t,3)+Ts*u(t,1)/L*tan(u(t,2));
        states(t+1,3)=states(t,3);
        states(t+1,1)=states(t,1)+Ts*u(t,1)*cos(states(t,3));
        states(t+1,2)=states(t,2)+Ts*u(t,1)*sin(states(t,3));
    end
    A(1,1,:)=ones(1,1,K);
    A(2,2,:)=ones(1,1,K);
    B(1,1,:)=Ts*cos(states(1:end-1,3));
    B(2,1,:)=Ts*sin(states(1:end-1,3));
    states=states(:,1:2);
    invB = zeros(Du,Dx,K);
    for t=1:K
        invB(:,:,t) = pinv(B(:,:,t));
    end
    if nargin<3
    states(1,:)=[];
    end
    if nargout==1
        states=states';
    end
end

