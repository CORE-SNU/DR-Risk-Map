function l=drawObstacles(a,b,c,color)
l(1)=fill3([a(2),a(2),a(1),a(1)],[b(1),b(2),b(2),b(1)],[c(1),c(1),c(1),c(1)],color,'FaceAlpha',0.2,'EdgeColor',color,'EdgeAlpha',0.5);
l(2)=fill3([a(2),a(2),a(1),a(1)],[b(1),b(2),b(2),b(1)],[c(2),c(2),c(2),c(2)],color,'FaceAlpha',0.2,'EdgeColor',color,'EdgeAlpha',0.5);
l(3)=fill3([a(2),a(2),a(2),a(2)],[b(2),b(2),b(1),b(1)],[c(1),c(2),c(2),c(1)],color,'FaceAlpha',0.2,'EdgeColor',color,'EdgeAlpha',0.5);
l(4)=fill3([a(1),a(1),a(1),a(1)],[b(2),b(2),b(1),b(1)],[c(1),c(2),c(2),c(1)],color,'FaceAlpha',0.2,'EdgeColor',color,'EdgeAlpha',0.5);
l(5)=fill3([a(2),a(2),a(1),a(1)],[b(1),b(1),b(1),b(1)],[c(1),c(2),c(2),c(1)],color,'FaceAlpha',0.2,'EdgeColor',color,'EdgeAlpha',0.5);
l(6)=fill3([a(2),a(2),a(1),a(1)],[b(2),b(2),b(2),b(2)],[c(1),c(2),c(2),c(1)],color,'FaceAlpha',0.2,'EdgeColor',color,'EdgeAlpha',0.5);
end