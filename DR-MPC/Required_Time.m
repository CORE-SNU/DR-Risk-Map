function t = Required_Time(x,vmax)
t(1) = 0;
for k=1:size(x,2)-1
    dist = norm(x(:,k+1)-x(:,k));
    if dist<=10^(-5)
        t(k+1)=t(k)+1;
    else
    t(k+1)=t(k)+norm(x(:,k+1)-x(:,k))/vmax;
    end
end
end