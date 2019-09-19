if ~exist('colorDevice', 'var') 
    colorDevice = imaq.VideoDevice('kinect',1);
end
if ~exist('depthDevice', 'var')
    depthDevice = imaq.VideoDevice('kinect',2);
end

init

%Initiallize the camera

% step(colorDevice);
% step(depthDevice);
% 
% %Grab one frame from the devices
% 
% colorImage = step(colorDevice);
% depthImage = step(depthDevice);
% 
% %Extract the point cloud
% 
% ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);
% 
% %Initialize a player to visualize 3D point cloud data.
% %The axis is set appropriately to visualize the point cloud from kinect
% 
% player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
%     'VerticalAxis','y','VerticalaxisDir','down');
% xlabel(player.Axes,'X(m)');
% ylabel(player.Axes,'Y(m)');
% zlabel(player.Axes,'Z(m)');
% 
% % acqire and view kinect point cloud data.
% 
% while isOpen(player)
%     colorImage = step(colorDevice);
%     depthImage = step(depthDevice); 
%     
%     ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);
%     
%     view(player, ptCloud);
% end 
% release(colorDevice);
% release(depthDevice);
% 
