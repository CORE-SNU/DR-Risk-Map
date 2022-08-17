function o=generateobstacle(obstacle,color,amin,amax,bmin,bmax,cmin,cmax)
a=obstacle(1,:);
b=obstacle(2,:);
c=obstacle(3,:);
o(1,:)=[a(1),b(1),c(1),a(2),b(2),c(1)];
o(2,:)=[a(1),b(1),c(2),a(2),b(2),c(2)];
o(3,:)=[a(2),b(1),c(1),a(2),b(2),c(2)];
o(4,:)=[a(1),b(1),c(1),a(1),b(2),c(2)];
o(5,:)=[a(1),b(1),c(1),a(2),b(1),c(2)];
o(6,:)=[a(1),b(2),c(1),a(2),b(2),c(2)];
drawObstacles(a,b,c,color)
axis([amin amax bmin bmax cmin cmax])
end