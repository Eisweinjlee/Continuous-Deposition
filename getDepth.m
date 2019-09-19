function [dep] = getDepth(camera, max)

%dep = step(camera);

dep = fliplr(step(camera));


figure(1);
imshow(dep, [0 max]);


end
