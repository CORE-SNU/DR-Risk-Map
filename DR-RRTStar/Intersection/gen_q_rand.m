Set_Params

for t=1:180
for i=1:2000
if rand()<=0.1
    q_rand_all{t}(:,i)=goalstate;
else
     rand1 = [x_min+(x_max-x_min)*rand() y_min+(y_max-y_min)*rand() 2*pi*rand()]';
     if (rand1(1)>0 && rand1(1)<13 && rand1(2)>16.3 && rand1(2)<23.7) || (rand1(1)>27 && rand1(1)<40 && rand1(2)>16.3 && rand1(2)<23.7) || ...
        (rand1(1)>16.3 && rand1(1)<23.7 && rand1(2)>0 && rand1(2)<13) || (rand1(1)>16.3 && rand1(1)<23.7 && rand1(2)>27 && rand1(2)<40) || ...
        (rand1(1)>13 && rand1(1)<27 && rand1(2)>13 && rand1(2)<27 && norm(rand1(1:2)-[13;27])>3.3 && norm(rand1(1:2)-[13;13])>3.3 && norm(rand1(1:2)-[27;27])>3.3 && norm(rand1(1:2)-[27;13])>3.3)
        t1=true;
     else
        t1=false;
     end
     while t1==false
        rand1 = [x_min+(x_max-x_min)*rand() y_min+(y_max-y_min)*rand() 2*pi*rand()]';
        if (rand1(1)>0 && rand1(1)<13 && rand1(2)>16.3 && rand1(2)<23.7) || (rand1(1)>27 && rand1(1)<40 && rand1(2)>16.3 && rand1(2)<23.7) || ...
            (rand1(1)>16.3 && rand1(1)<23.7 && rand1(2)>0 && rand1(2)<13) || (rand1(1)>16.3 && rand1(1)<23.7 && rand1(2)>27 && rand1(2)<40) || ...
            (rand1(1)>13 && rand1(1)<27 && rand1(2)>13 && rand1(2)<27 && norm(rand1(1:2)-[13;27])>3.3 && norm(rand1(1:2)-[13;13])>3.3 && norm(rand1(1:2)-[27;27])>3.3 && norm(rand1(1:2)-[27;13])>3.3)
            t1=true;
        else
            t1=false;
        end
     end 
     q_rand_all{t}(:,i)=rand1;
end
end
end
save('q_rand.mat', 'q_rand_all')