function nc = noCollision(n2, n1, o)
    m = (n1(2)-n2(2))/(n1(1)-n2(1));
    b = n1(2)-m*n1(1);
    b2 = n2(2)-m*n2(1);
    if abs(b-b2)>=10e-6
        error("Wrong Slope!")
    end
    n = size(o,1);
    dist12=sqrt((n2(1:2)-n1(1:2))*(n2(1:2)-n1(1:2))');
    for i=1:n
        if (n1-o(i,1:2))*(n1-o(i,1:2))'<=o(i,3)^2
            c(i)=1;
        else
            [xout,yout]=Intersection(m,b,o(i,:));
            if isnan(xout(1))
                c(i)=0;
            else
                for j=1:size(xout,2)
                    dist1o(j)=sqrt((n1(1:2)-[xout(j) yout(j)])*(n1(1:2)-[xout(j) yout(j)])');
                    dist2o(j)=sqrt((n2(1:2)-[xout(j) yout(j)])*(n2(1:2)-[xout(j) yout(j)])');
                end
                if (dist1o(1)+dist2o(1)==dist12) || (dist1o(2)+dist2o(2)==dist12) 
                    c(i)=1;
                else
                    c(i)=0;
                end
            end
        end
    end
    if any(c)
        nc = 0;
    else
        nc = 1;
    end
end