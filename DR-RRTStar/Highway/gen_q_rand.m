clear all
close all
Set_Params

for t=1:180
for i=1:2000
if rand()<=tryGoalProbability
    q_rand_all{t}(:,i) = goalstate;
else
    q_rand_all{t}(:,i) = [x_min+(x_max-x_min)*rand() y_min+(y_max-y_min)*rand() 2*pi*rand()]';
end
end
end
save('q_rand.mat', 'q_rand_all')