function [trimmed] = plotDepth(dep, rangex, rangey)

trimmed = dep(rangey, rangex);
[x,y] = ndgrid(rangey, rangex);

figure(2);
surf(x,y,trimmed);
zlim([600 1000])
end

%300:357, 205:282