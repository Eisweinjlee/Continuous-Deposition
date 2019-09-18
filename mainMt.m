Mtcounter = Mtcounter + 1;

input('take 1st Mt image >>');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 1, Mtcounter) = trimmedM;

input('take 2nd Mt image >>');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 2, Mtcounter) = trimmedM;

input('take 3rd Mt image >>');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 3, Mtcounter) = trimmedM;

input('take 4th Mt image >>');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 4, Mtcounter) = trimmedM;

input('take 5th Mt image >>');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 5, Mtcounter) = trimmedM;

input('take 6th Mt image >>');
dep = getDepth(depthDevice, 1000);
r = size(dep);
trimmedM = plotDepth(dep,  1:r(2), 1:r(1) );
picMt(:,:, 6, Mtcounter) = trimmedM;

%picarray2(:,:,size(picarray2,3)+1) = trimmedM;