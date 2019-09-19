% The soil shape sensing with Kinect camera
% This program is applied on the specified system in my proj.
% Author: Li Yang
% Date: September 18, 2019
close all
clear

%% 1. Initialization

% Connect the power supply of kinect camera
% Connect the USB cable to USB 3.0 port on your PC
% If the device is recognized successfully, we can run the following

if ~exist('colorDevice', 'var') 
    colorDevice = imaq.VideoDevice('kinect',1);
end % find the color device

if ~exist('depthDevice', 'var')
    depthDevice = imaq.VideoDevice('kinect',2);
end % find the depth device

input('Press Enter to continue:');

%% 2. retrieve the data from Kinect (from demo code)

dep = getDepth(depthDevice, 1000);

% specify range of hand
if ~exist('xrangeHand', 'var') && ~exist('yrangeHand', 'var')
% xrangeHand = input('input range of Hand (horizon) >> ');
% yrangeHand = input('input range of Hand (vertica) >> '); % not used
xrangeHand = [];
yrangeHand = [];
end

% % specify range of Mt
% if ~exist('xrangeMt', 'var') && ~exist('yrangeMt', 'var')
% xrangeMt = input('input range of Mountain (horizon) >> ');
% yrangeMt = input('input range of Mountain (vertica) >> ');
% end

% take first pic of hand
% input('take a picture (press Enter):');
dep = getDepth(depthDevice, 1000);
trimmedH = plotDepth(dep, xrangeHand, yrangeHand);

% % take first pic of Mt
% input('take a picture of Mt');
% dep = getDepth(depthDevice, 1000);
% trimmedM = plotDepth(dep, xrangeMt  , yrangeMt  );

% delete previous picarray and picarray2 array
picHand = trimmedH;
% picMt = trimmedM;
% picMt = [];
picMt = zeros(424, 512);
Mtcounter = 0;

run mainMt 

%% 3. Ask to save the data

if exist('picHand', 'var') || exist('picMt', 'var')
    if input('save the data? >> [y/n]', 's') == 'y'
        % save picarray and picarray2 data into file
        
        name = input('pic name >> ', 's');
        save(sprintf('data/%s-hand.mat', name), 'picHand');                
        save(sprintf('data/%s-mt.mat', name), 'picMt');
    end
end

%% 4. Normalize the data to what we want (Optional)
% (http://www.codeproject.com/Articles/317974/KinectDepthSmoothing)
% load('data\%s-mt.mat', name)

if input('Normalize the data? >> [y/n]', 's') == 'y'
    depthImage = picMt(:,:,1);
    [row,col] = size(depthImage);
    widthBound = row - 1;
    heightBound = col - 1;
    
    % initializing working image; leave original matrix aside
    filledDepth = depthImage;
    
    %initializing the filter matrix
    filterBlock5x5 = zeros(5,5);
    
    % to keep count of zero pixels found
    zeroPixels = 0;
    
    %The main loop
    for x = 1:row
        for y = 1:col
            % Only for pixels with 0 depth value; else skip
            if filledDepth(x,y) == 0
                zeroPixels = zeroPixels+1;
                % values set to identify a positive filter result.
                p = 1;
                % Taking a cube of 5x5 around the 0 depth pixel
                % q = index
                % select two pixels behind and two ahead in a row
                % select two pixels behind and two ahead in a column
                % leave the center pixel (as its the one to be filled)
                for xi = -2 : 1 : 2
                    q = 1;
                    for yi = -2 : 1 : 2
                        % updating index for next pass
                        xSearch = x + xi;
                        ySearch = y + yi;
                        % xSearch and ySearch to avoid edges
                        if (xSearch > 0 && xSearch < widthBound && ySearch > 0 && ySearch < heightBound)
                            % save values from depth image into filter
                            filterBlock5x5(p,q) = filledDepth(xSearch,ySearch);
                        end
                        q = q+1;
                    end
                    p = p+1;
                end
                % Now that we have the 5x5 filter, with values surrounding
                % the zero valued fixel, in 'filterBlock5x5' ; we can now
                % Calculate statistical mode of the 5x5 matrix
                X = sort(filterBlock5x5(:));
                % find all non-zero entries in the sorted filter block
                [~,~,v] = find(X);
                % indices where repeated values change
                if (isempty(v))
                    filledDepth(x,y) = 0;
                else
                    indices   =  find(diff([v; realmax]) > 0);
                    % finding longest persistent length of repeated values
                    [~,i] =  max (diff([0; indices]));
                    % The value that is repeated is the mode
                    mode      =  v(indices(i));
                    
                    % fill in the x,y value with the statistical mode of the values
                    filledDepth(x,y) = mode;
                end
            end
        end
        % end for
    end
    
    dep = filledDepth;   %change dep to variable name you want
    
    %-------------------test plot to ensure b4 save data-----------------------
    
    l = 1.6667;    % pixel edge length
    PX1 = 63; % 93 pixel same plane 160 mm 42
    PX2 = 159; % 135
    PY1 = 184; % 99 pixel diffent plane 170 mm 165
    PY2 = 286; % 263
    % zplane start 560
    PX = PX1:PX2;
    PY = PY1:PY2;
    
    dep = dep(PX,PY);
    [m,n] = size(dep);
    [X,Y] = meshgrid(0:l:l*(n-1), 0:l:l*(m-1));
    Y = Y - 80;
    
    dep = -dep;
    MEAN = mean(dep);
    
    first_col = MEAN(1); % cal first_col from empty vessel
    dep = dep - (-5.756391752577320e+02); %change first_col to const. for next time
    dep = flipud(dep);
    
    for times = 1:1:100
        dep = imgaussfilt(dep);
    end
    
    figure
    mesh(X,Y,dep)
    xlabel('x[mm]')
    ylabel('y[mm]')
    zlabel('h[mm]')
    % zlim([-50 40])
    % zlim([500 650])
    %---------------------save data--------------------------------------------
    %save('dep.mat','dep')  %'file name','variable name'
    dep3 = dep;
    % save('','dep3')
    % this saved data has been cut to 93x99 and filtered. ready for finding
    % error.
    
end
