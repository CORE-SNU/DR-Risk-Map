function [current_ref, ref, refnum] = choose_ref(x, ref, t, K, oldrefnum)
    if t==1
        refnum = 1;
    else
        if oldrefnum < size(ref,2)-10
            dist = vecnorm(x(1:2,t)-ref(:,oldrefnum:oldrefnum+10));
            [~,refnum] = min(dist);
            refnum = oldrefnum+refnum-1;
        else
            dist = vecnorm(x(1:2,t)-ref(:,oldrefnum:end));
            [~,refnum] = min(dist);
            refnum = oldrefnum+refnum-1;
        end
    end
    
    if refnum+K+1 > size(ref,2)
      ref(:,refnum+K+1) = ref(:,refnum+K);
    end
    current_ref = ref(:,refnum+1:refnum+K);
end