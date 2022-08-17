function [states,A,B,invB]=vehicledyn(u,s,remove)
global Ts L
    Dx=size(s,2);
    Du=size(u,2);
    K=size(u,1);
    states(1,:)=s;
    
    A=zeros(Dx,Dx,K);
    B=zeros(Dx,Du,K);
    for t=1:K
%         states(t+1,3)=states(t,3)+Ts*u(t,1)/L*tan(u(t,2));
        states(t+1,3)=states(t,3)+Ts*u(t,2);
        states(t+1,1)=states(t,1)+Ts*u(t,1)*cos(states(t,3));
        states(t+1,2)=states(t,2)+Ts*u(t,1)*sin(states(t,3));
    end
    A(1,1,:)=ones(1,1,K);
    A(1,3,:)=-Ts*u(1:end,1).*sin(states(1:end-1,3));
    A(2,2,:)=ones(1,1,K);
    A(2,3,:)=Ts*u(1:end,1).*cos(states(1:end-1,3));
    A(3,3,:)=ones(1,1,K);
    B(1,1,:)=Ts*cos(states(1:end-1,3));
    B(2,1,:)=Ts*sin(states(1:end-1,3));
%     B(3,1,:)=Ts/L*tan(u(1:end,2));
%     B(3,2,:)=Ts/L*u(1:end,1)./(cos(u(1:end,2)).^2);
    B(3,2,:)=Ts*ones(1,1,K);
%     
%     for t=1:K
%         states(t+1,3)=states(t,3)+Ts*states(t,4)/L*tan(u(t,2));
%         states(t+1,1)=states(t,1)+Ts*states(t,4)*cos(states(t,3));
%         states(t+1,2)=states(t,2)+Ts*states(t,4)*sin(states(t,3));
%         states(t+1,4)=states(t,4)+Ts*u(t,1);
%     end
%     A(1,1,:)=ones(1,1,K);
%     A(1,3,:)=-Ts*states(1:end-1,4).*sin(states(1:end-1,3));
%     A(1,4,:)=Ts*cos(states(1:end-1,3));
%     A(2,2,:)=ones(1,1,K);
%     A(2,3,:)=Ts*states(1:end-1,4).*cos(states(1:end-1,3));
%     A(2,4,:)=Ts*sin(states(1:end-1,3));
%     A(3,3,:)=ones(1,1,K);
%     A(3,4,:)=Ts/L*tan(u(1:end,2));
%     A(4,4,:)=ones(1,1,K);
%     B(3,2,:)=Ts/L*states(1:end-1,4)./(cos(u(1:end,2)).^2);
%     B(4,1,:)=ones(1,1,K);
    
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

