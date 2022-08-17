close all
noise = normrnd(0,0.001, 1000, 2, 4, 1000);
load('obs_traj1.mat') %Obstacle trajectories

for iter = 1:1000
%     close all
%     try
        clearvars -except iter xy21 xy21_var xy21_var1 obs_state obs_input xtr1 ytra1 ytrr1 xytheta_mu xytheta_var noise
        global M1 L W Ts K
        M1=50; L=2; W=0.2; Ts=0.1; K=10;
%         figure
%         set(gcf,'Position',[90,50,869,590]);
%         set(gca,'Position',[0.057537399309551,0.043227665706052,0.922899884925201,0.953890489913544]);
%         hold on
%         xlabel('X','FontSize',16)
%         ylabel('Y', 'FontSize',16)
%         obs_state{iter} = [10*rand();10*rand();pi*rand()];
%         obs_input{iter} = [repmat(5*rand(),1,60); normrnd(0, 0.1,1,60)];
%         obs_state{iter} = vehicledyn(obs_input{iter}',obs_state{iter}',1);
        meanfunc = {@meanZero};
        covfunc = {@covSEiso};
        likfunc ={@likGauss};
        hyp = struct('mean', [] , 'cov', [0 0], 'lik', -1);
%         axis([0 max(obs_state(1,:)) 0 max(obs_state(2,:))]);
%         daspect([1 1 1])
%         act = scatter(obs_state(1,:),obs_state(2,:),10,'MarkerFaceColor',[0.15,0.73,0.15],'MarkerEdgeColor',[0.14,0.72,0.14],'MarkerFaceAlpha',0.6,'MarkerEdgeAlpha',1);
%         stat = scatter(obs_state(1,1),obs_state(2,1),10,'MarkerEdgeColor',[0.66,0.06,0.21],'MarkerFaceColor',[0.82,0.11,0.24],'MarkerEdgeAlpha',1);
%         plot(obs_state(1,:),obs_state(2,:),'k', 'Marker','.','MarkerSize',8)

        for t=1:size(obs_input,2)
            if t~=1
                for kk=1:size(obs_state,3)
                    xtr1{iter}{kk}=[xtr1{iter}{kk};    obs_state(1:3,t,kk)'];
                    ytra1{iter}{kk}=[ytra1{iter}{kk};  obs_input(1,t,kk)'+noise(t,1,kk,iter)];
                    ytrr1{iter}{kk}=[ytrr1{iter}{kk};  obs_input(2,t,kk)'+noise(t,2,kk,iter)];
                end
            else
                xtr1{iter}=num2cell(permute(obs_state(1:3,t,:),[3 1 2]),2);
                ytra1{iter}=num2cell(permute(obs_input(1,t,:),[3 2 1])+permute(noise(t,1,1:size(obs_state,3),iter),[3 2 1]),2);
                ytrr1{iter}=num2cell(permute(obs_input(2,t,:),[3 2 1])+permute(noise(t,2,1:size(obs_state,3),iter),[3 2 1]),2);
            end
            if size(xtr1{iter}{1},1)>=M1
                for obsj=1:length(xtr1{iter})
                    xtr1{iter}{obsj}=xtr1{iter}{obsj}(end-M1+1:end,:);
                    ytra1{iter}{obsj}=ytra1{iter}{obsj}(end-M1+1:end,:);
                    ytrr1{iter}{obsj}=ytrr1{iter}{obsj}(end-M1+1:end,:);
                end
            end        
%             if t==51
                for kk=1:size(obs_state,3)
                        xt_mu1=obs_state(1:3,t,kk);
                        xt_var1=zeros(3);
                        xmu1(1)=obs_state(1,t,kk);
                        ymu1(1)=obs_state(2,t,kk);
                        thetamu1(1)=obs_state(3,t,kk);
                        xvar1(1)=0;
                        yvar1(1)=0;
                        thetavar1(1)=0; 
                        vvar1(1)=0;
                        for k=1:K
                            M=size(xtr1{iter}{kk},1);
                            if k==1         
                                hyp11 = minimize(hyp, @gp, -2000, @infGaussLik, meanfunc, covfunc, likfunc, xtr1{iter}{kk}, ytra1{iter}{kk});
                                hyp21 = minimize(hyp, @gp, -2000, @infGaussLik, meanfunc, covfunc, likfunc, xtr1{iter}{kk}, ytrr1{iter}{kk});
                                Kxx11=covSEiso(hyp11(1).cov,xtr1{iter}{kk},xtr1{iter}{kk});
                                Kxx21=covSEiso(hyp21(1).cov,xtr1{iter}{kk},xtr1{iter}{kk});
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
                            kXx11=covSEiso(hyp11.cov,xtr1{iter}{kk},xt_mu1(:,k)');
                            kxx21=covSEiso(hyp21.cov,xt_mu1(:,k)',xt_mu1(:,k)');
                            kXx21=covSEiso(hyp21.cov,xtr1{iter}{kk},xt_mu1(:,k)');
                            v11=W11'\kXx11;
                            v21=W21'\kXx21;
                            beta11=W11\(W11'\(ytra1{iter}{kk}));       
                            beta21=W21\(W21'\(ytrr1{iter}{kk})); 
                            amu1(k)=kXx11'*beta11;
                            rmu1(k)=kXx21'*beta21;
                            ut_mu(k,:)=[amu1(k); rmu1(k)];
                            a_mud1(k,:)=((kXx11.*((xtr1{iter}{kk}-xt_mu1(:,k)')*Lf11))'*beta11)';
                            r_mud1(k,:)=((kXx21.*((xtr1{iter}{kk}-xt_mu1(:,k)')*Lf21))'*beta21)';
                            mud1(:,:,k)=[a_mud1(k,:);r_mud1(k,:)];
                            avar1(k)=kxx11-v11'*v11+a_mud1(k,:)*(xt_var1(:,:,k))*a_mud1(k,:)';
                            rvar1(k)=kxx21-v21'*v21+r_mud1(k,:)*(xt_var1(:,:,k))*r_mud1(k,:)';
                            u_var1(:,:,k)=diag([avar1(k) rvar1(k)]);
                            xvvar1(:,:,k)=xt_var1(:,:,k)*mud1(:,:,k)';
                            [xt_mu1(:,k+1), x_mud1, u_mud1]=vehicledyn([amu1(k) rmu1(k)],xt_mu1(:,k)');
                            xt_var1(:,:,k+1)=x_mud1*xt_var1(:,:,k)*x_mud1'+u_mud1*u_var1(:,:,k)*u_mud1'+x_mud1*xvvar1(:,:,k)*u_mud1'+u_mud1*xvvar1(:,:,k)'*x_mud1';
                            xvar1(k+1)=xt_var1(1,1,k+1);
                            yvar1(k+1)=xt_var1(2,2,k+1);
                            thetavar1(k+1)=xt_var1(3,3,k+1);
                            xmu1(k+1)=xt_mu1(1,k+1);
                            ymu1(k+1)=xt_mu1(2,k+1);
                            thetamu1(k+1)=xt_mu1(3,k+1);
                        end
                        for k=1:K+1
                            xy21{iter}{t}(k,:,kk)=[xmu1(k) ymu1(k)];
                            xy21_var1{iter}{t}(:,:,k,kk)=xt_var1(1:2,1:2,k);
                            xy21_var{iter}{t}(:,:,k,kk)=diag([sqrt(xt_var1(1,1,k)) sqrt(xt_var1(2,2,k))]);
                            xytheta_var{iter}{t}{kk}=xt_var1(:,:,k);
                            xytheta_mu{iter}{t}{kk}=xt_mu1(:,k);

                        end     
%                         try
%                             set(movv1{kk}, 'xdata',xy21{iter}{t}(:,1,kk), 'ydata', xy21{iter}{t}(:,2,kk));
%                             set(movv11{kk}, 'xdata',xy21{iter}{t}(:,1,kk), 'ydata', xy21{iter}{t}(:,2,kk));
%                         catch
%                             movv1{kk}=scatter(xy21{iter}{t}(:,1,kk),xy21{iter}{t}(:,2,kk),300,'MarkerFaceAlpha',0.8,'MarkerEdgeAlpha',1,'MarkerFaceColor',[1.00,0.96,0.12],'MarkerEdgeColor',[0.64,0.48,0.11]);
%                             movv11{kk}=plot(xy21{iter}{t}(:,1,kk),xy21{iter}{t}(:,2,kk),'k', 'Marker','.','MarkerSize',8);            
%                         end
%                         drawnow
                end
%             end
        end
%     catch
%         iter=iter-1;
%         continue
%     end
%     set(stat,'xdata',obs_state(1,t),'ydata',obs_state(2,t));
%     uistack(movv1{1},'top')
%     uistack(movv11{1},'top')
%     uistack(stat,'top')
%     legend([movv1{1} act], 'GPR', 'Actual','location','northwest','interpreter','latex','FontSize',14);
end
save('MC_data_noise', 'xy21_m', 'xy21_var1_m', 'xy21_var_m', 'noise');
