
%plot existing discrete obstacle map
clear all
close all
load polygObst_lawnmowerMap


hold on
polygon = [];
firstEdge=1;
j = 1;
for(i=1:length(x1))
    
    edges(i,1)=i;
    if (buttonCopy(i)==1)
        edges(i,2)=i+1;
    elseif (buttonCopy(i)==3)
        edges(i,2)=firstEdge;
        firstEdge=i+1;
    end
    plot([x1(edges(i,1)),x1(edges(i,2))],[y1(edges(i,1)),y1(edges(i,2))]);
    axis([-20 20 -20 20])
    
    hold on
    drawnow
    
    
    polygon(j, 1) = x1(edges(i,1));
    polygon(j, 2) = y1(edges(i,1));
    
    j = j + 1;
    if (buttonCopy(i)==3)
        polygon
        extendedPolygon = extendPoly ( polygon , 1 );
        for k=1:length(extendedPolygon)
           toPlot=extendedPolygon{k};
           line(toPlot(:,1),toPlot(:,2),'Color','b');
        end
        hold on
        clear polygon
        j = 1;
    end
end
%extendedPolygon = extendPoly ( polygon , range, resolution )


%plot(startPos(1),startPos(2),'x')
%plot(goalPos(1),goalPos(2),'*')

hold off

%creating new maps
% figure
% plot([0 100 100 0 0],[0 0 100 100 0])
% 
% [x,y,button] = ginput();
% startPos=[30 70];
% goalPos=[80 30];

%save('polygObst','x','y','button','startPos','goalPos')

%print -deps2 polygonalMap.eps












