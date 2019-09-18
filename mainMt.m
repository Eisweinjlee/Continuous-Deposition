Mtcounter = Mtcounter + 1;

disp('take 1st Mt image ...(1/6)');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 1, Mtcounter) = trimmedM;

disp('take 2nd Mt image ...(2/6)');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 2, Mtcounter) = trimmedM;

disp('take 3rd Mt image ...(3/6)');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 3, Mtcounter) = trimmedM;

disp('take 4th Mt image ...(4/6)');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 4, Mtcounter) = trimmedM;

input('take 5th Mt image ...(5/6)');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 5, Mtcounter) = trimmedM;

disp('take 6th Mt image ...(6/6)');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 6, Mtcounter) = trimmedM;

%picarray2(:,:,size(picarray2,3)+1) = trimmedM;