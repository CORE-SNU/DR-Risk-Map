function risk=ComputeRisk(node,k)
global xy21 xy21_var controller r2 t theta;
% % % % % for i=1:size(statobs,1)
% % % % %     risk(i)=-(node.coord(1:2)'-statobs(i,1:2))*(node.coord(1:2)'-statobs(i,1:2))';
% % % % % end
% % % % % risk=risk';
% tic
if ~isempty(xy21)
for i=1:size(xy21{t},2)
    [risk(i),diagnos]=controller({round(node.coord(1:2),6),xy21_var{t}(:,:,k+1,i),xy21{t}(k+1,:,i)'});
    if diagnos~=0 && diagnos~=4
        error(yalmiperror(diagnos));
    end
end
% disp(risk)
risk=subplus(max(risk+r2));
else
    risk=0;
end
% fprintf('Time Required For Computing Riks: %d\n',toc);
% disp(risk)
% tic
% if ~isempty(xy21)
% for i=1:size(xy21{t},2)
%     risk(i)=Risk_Map(theta, round(node.coord(1:2),6),xy21{t}(k+1,:,i)',[xy21_var{t}(1,1,k+1,i), xy21_var{t}(1,2,k+1,i), xy21_var{t}(2,2,k+1,i)]);
% end
% risk=subplus(max(risk+r2));
% else
%     risk=0;
% end
% fprintf('Time Required For Computing Riks1: %d\n',toc);
% disp(risk)
end

% function risk=ComputeRisk(node,k)
% global xy21 xy21_var controller r2 t;
% % % % % % for i=1:size(statobs,1)
% % % % % %     risk(i)=-(node.coord(1:2)'-statobs(i,1:2))*(node.coord(1:2)'-statobs(i,1:2))';
% % % % % % end
% % % % % % risk=risk';
% tic
% if ~isempty(xy21)
% [risk,diagnos]=controller({round(node.coord(1:2),6),permute(xy21_var{t}(:,:,2,:),[1 2 4 3]),permute(xy21{t}(2,:,:),[2 3 1])',r2});
%     if diagnos~=0 && diagnos~=4
%         error(yalmiperror(diagnos));
%     end
% else
%     risk=0;
% end
% %fprintf('Time Required For Computing Riks: %d\n',toc)
% end