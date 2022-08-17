function risk=ComputeRisk(node,k)
global xy21 xy21_var controller r2 t;
% % % % % for i=1:size(statobs,1)
% % % % %     risk(i)=-(node.coord(1:2)'-statobs(i,1:2))*(node.coord(1:2)'-statobs(i,1:2))';
% % % % % end
% % % % % risk=risk';
if ~isempty(xy21)
for i=1:length(xy21{t})
    [risk(i),diagnos]=controller({round(node.coord(1:2),6),xy21_var{t}{i}(:,:,k+1),xy21{t}{i}(k+1,:)', r2(i)});
    if diagnos~=0 && diagnos~=4
        error(yalmiperror(diagnos));
    end
end
risk=subplus(max(r2+risk));
else
    risk=0;
end
end