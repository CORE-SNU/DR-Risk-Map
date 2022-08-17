if t~=1
    for kk=1:size(obs,1)
        xtr1{kk}=[xtr1{kk};    obs_state(1:3,t,kk)'];
        ytra1{kk}=[ytra1{kk};  obs_input(1,t,kk)'+normrnd(0,0.0001)];
        ytrr1{kk}=[ytrr1{kk};  obs_input(2,t,kk)'+normrnd(0,0.0001)];
    end
else
    xtr1=num2cell(permute(obs_state(1:3,t,:),[3 1 2]),2);
    ytra1=num2cell(permute(obs_input(1,t,:),[3 2 1])+normrnd(0,0.0001),2);
    ytrr1=num2cell(permute(obs_input(2,t,:),[3 2 1])+normrnd(0,0.0001),2);
end
if size(xtr1{1},1)>=10
    for obsj=1:length(xtr1)
        xtr1{obsj}=xtr1{obsj}(end-9:end,:);
        ytra1{obsj}=ytra1{obsj}(end-9:end,:);
        ytrr1{obsj}=ytrr1{obsj}(end-9:end,:);
    end
end        

for kk=1:size(obs,1)
        xt_mu1=obs_state(1:3,t,kk);
        xt_var1=zeros(3);
        xmu1(1)=obs_state(1,t,kk);
        ymu1(1)=obs_state(2,t,kk);
        thetamu1(1)=obs_state(3,t,kk);
%         vmu1(1)=obs_state(4,t,kk);
        xvar1(1)=0;
        yvar1(1)=0;
        thetavar1(1)=0; 
        vvar1(1)=0;
        for k=1:K
            M=size(xtr1{kk},1);
            if k==1         
                hyp11 = minimize(hyp, @gp, -2000, @infGaussLik, meanfunc, covfunc, likfunc, xtr1{kk}, ytra1{kk});
                hyp21 = minimize(hyp, @gp, -2000, @infGaussLik, meanfunc, covfunc, likfunc, xtr1{kk}, ytrr1{kk});
                Kxx11=covSEiso(hyp11(1).cov,xtr1{kk},xtr1{kk});
                Kxx21=covSEiso(hyp21(1).cov,xtr1{kk},xtr1{kk});
                Lf11=exp(-2*hyp11.cov(1));
                Lf21=exp(-2*hyp21.cov(1));
                sigmaf11=exp(hyp11.cov(2));
                sigmaf21=exp(hyp21.cov(2));
                sigman11=exp(hyp11.lik);
                sigman21=exp(hyp21.lik);
                W11=chol(Kxx11+sigman11^2*eye(M,M)+10^(-6)*eye(M,M));
                W21=chol(Kxx21+sigman21^2*eye(M,M)+10^(-6)*eye(M,M));
            end
            kxx11=covSEiso(hyp11.cov,xt_mu1(:,k)',xt_mu1(:,k)');
            kXx11=covSEiso(hyp11.cov,xtr1{kk},xt_mu1(:,k)');
            kxx21=covSEiso(hyp21.cov,xt_mu1(:,k)',xt_mu1(:,k)');
            kXx21=covSEiso(hyp21.cov,xtr1{kk},xt_mu1(:,k)');
            v11=W11'\kXx11;
            v21=W21'\kXx21;
            beta11=W11\(W11'\(ytra1{kk}));       
            beta21=W21\(W21'\(ytrr1{kk})); 
            amu1(k)=kXx11'*beta11;
            rmu1(k)=kXx21'*beta21;
            ut_mu(k,:)=[amu1(k); rmu1(k)];
            a_mud1(k,:)=((kXx11.*((xtr1{kk}-xt_mu1(:,k)')*Lf11))'*beta11)';
            r_mud1(k,:)=((kXx21.*((xtr1{kk}-xt_mu1(:,k)')*Lf21))'*beta21)';
            mud1(:,:,k)=[a_mud1(k,:);r_mud1(k,:)];
            avar1(k)=kxx11-v11'*v11+a_mud1(k,:)*(xt_var1(:,:,k))*a_mud1(k,:)';
            rvar1(k)=kxx21-v21'*v21+r_mud1(k,:)*(xt_var1(:,:,k))*r_mud1(k,:)';
            u_var1(:,:,k)=diag([avar1(k) rvar1(k)]);
            xvvar1(:,:,k)=xt_var1(:,:,k)*mud1(:,:,k)';
            [xt_mu1(:,k+1), x_mud1, u_mud1]=vehicledyn([amu1(k) rmu1(k)],xt_mu1(:,k)');
%             xt_var1(:,:,k+1)=x_mud1*xt_var1(:,:,k)*x_mud1'+u_mud1*u_var1(:,:,k)*u_mud1'+x_mud1*xvvar1(:,:,k)*u_mud1'+u_mud1*xvvar1(:,:,k)'*x_mud1';
            xt_var1(:,:,k+1)=x_mud1*xt_var1(:,:,k)*x_mud1'+u_mud1*u_var1(:,:,k)*u_mud1';
%             try
%                 if ~isdiag(xt_var1(1:2,1:2,k+1)) && sum(sum(xt_var1(1:2,1:2,k+1)<=10^(-7)))~=4
%                     xxxx=chol(xt_var1(1:2,1:2,k+1));
%                 end
%             catch
%                 warning('no')
%             end
            xvar1(k+1)=xt_var1(1,1,k+1);
            yvar1(k+1)=xt_var1(2,2,k+1);
            thetavar1(k+1)=xt_var1(3,3,k+1);
%             vvar1(k+1)=xt_var1(4,4,k+1);
            xmu1(k+1)=xt_mu1(1,k+1);
            ymu1(k+1)=xt_mu1(2,k+1);
            thetamu1(k+1)=xt_mu1(3,k+1);
%             vmu1(k+1)=xt_mu1(4,k+1);
        end
        for k=1:K+1
            xy21{t}(k,:,kk)=[xmu1(k) ymu1(k)];
%             if k==1
%                 xy21_var{t}{kk}(:,:,k)=diag([0 0]);
%             else
%                 xy21_var{t}{kk}(:,:,k)=xy21_var{t}{kk}(:,:,k-1)+diag([0.005 0.005]);
%             end
% %             if isdiag(xt_var1(1:2,1:2,k))
% %                 xy21_var{t}{kk}(:,:,k)=sqrt(xt_var1(1:2,1:2,k));
% %             elseif sum(sum(xt_var1(1:2,1:2,k)<=10^(-7)))==4
% %                 xy21_var{t}{kk}(:,:,k)=zeros(2);
% %             else
% %             xy21_var{t}{kk}(:,:,k)=chol(xt_var1(1:2,1:2,k));
% %             end
            xy21_var1{t}(:,:,k,kk)=diag([xt_var1(1,1,k) xt_var1(2,2,k)]);
            xy21_var{t}(:,:,k,kk)=diag([sqrt(xt_var1(1,1,k)) sqrt(xt_var1(2,2,k))]);
            xytheta_var{t}{kk}=xt_var1(:,:,k);
            xytheta_mu{t}{kk}=xt_mu1(:,k);
% % % %             Gp1{kk}(:,:,k)=[cos(thetamu1(k)) sin(thetamu1(k));cos(thetamu1(k)+pi) sin(thetamu1(k)+pi);cos(thetamu1(k)+pi/2) sin(thetamu1(k)+pi/2); cos(thetamu1(k)-pi/2) sin(thetamu1(k)-pi/2)];
% % % %             g_p1{kk}(1,k)=Gp1{kk}(1,:,k)*xy21{t}{kk}(k,:)'+obs(kk,end-2)/2;
% % % %             g_p1{kk}(2,k)=Gp1{kk}(2,:,k)*xy21{t}{kk}(k,:)'+obs(kk,end-2)/2;
% % % %             g_p1{kk}(3,k)=Gp1{kk}(3,:,k)*xy21{t}{kk}(k,:)'+obs(kk,end-1)/2;
% % % %             g_p1{kk}(4,k)=Gp1{kk}(4,:,k)*xy21{t}{kk}(k,:)'+obs(kk,end-1)/2;
% % % %             movobs11{kk}(1,:,k)=(inv(Gp1{kk}([1 3],:,k))*g_p1{kk}([1 3],k))';
% % % %             movobs11{kk}(2,:,k)=(inv(Gp1{kk}([1 4],:,k))*g_p1{kk}([1 4],k))';
% % % %             movobs11{kk}(3,:,k)=(inv(Gp1{kk}([2 4],:,k))*g_p1{kk}([2 4],k))';
% % % %             movobs11{kk}(4,:,k)=(inv(Gp1{kk}([2 3],:,k))*g_p1{kk}([2 3],k))';  
            if k~=1
                try
% % % %                     set(movv1{kk}(k),'xdata',movobs11{kk}(:,1,k));
% % % %                     set(movv1{kk}(k),'ydata',movobs11{kk}(:,2,k));
                            set(movv1{kk}(k),'Position',[xy21{t}(k,1,kk)-W/2 xy21{t}(k,2,kk)-W/2 W W]);
                catch
% % % %                     movv1{kk}(k)=fill(movobs11{kk}(:,1,k),movobs11{kk}(:,2,k),[0 .5 .5],'EdgeColor',[0 .5 .5],'FaceAlpha',0.05);
                            movv1{kk}(k)=rectangle('Position',[xy21{t}(k,1,kk)-W/2 xy21{t}(k,2,kk)-W/2 W W],'FaceColor',[0 .5 .5 0.05],'EdgeColor',[0 .5 .5],'Curvature',[1 1]);                
                end
            end
        end     
        try
% % % %             set(movv1{kk}(1),'xdata',movobs11{kk}(:,1,1));
% % % %             set(movv1{kk}(1),'ydata',movobs11{kk}(:,2,1));
            set(movv1{kk}(1),'Position',[xy21{t}(1,1,kk)-W/2 xy21{t}(1,2,kk)-W/2 W W]);        
        catch
            movv1{kk}(1)=rectangle('Position',[xy21{t}(1,1,kk)-W/2 xy21{t}(1,2,kk)-W/2 W W],'FaceColor',[0 .5 .5 1],'EdgeColor',[0 .5 .5],'Curvature',[1 1]);
        end
        drawnow
end