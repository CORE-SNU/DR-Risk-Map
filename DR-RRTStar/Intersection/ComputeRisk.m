function risk=ComputeRisk(node,k)
global pos_mu pos_var controller r2 t;

if ~isempty(pos_mu)
for i=1:size(pos_mu{t},3)
    [risk(i),diagnos] = controller({round(node.coord(1:2),6),pos_var{t}(:,:,k+1,i),pos_mu{t}(:,k+1,i)});
    if diagnos~=0 && diagnos~=4
        error(yalmiperror(diagnos));
    end
end
risk=subplus(max(risk+r2));
else
    risk=0;
end

end
