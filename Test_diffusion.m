%% Initialization
clear all
close all

excavator_data;

%------- Diffusion Section------

% Diffusion coefficient
wl = 0.15;
wr = wl;
wu = 0.15;
wd = wu;
wc = 1-wl-wr-wu-wd;

H_n = R;
[mH,nH] = size(H_n);
diffussion_times = 50;

% Modified version - LI - Sep 5th,2019

for k = 1:1:diffussion_times
    
    H_r = horzcat(zeros(mH,1),H_n(:,:,k));
    add_H_r = H_r(:,nH+1) * wr;
    H_r(:,nH+1) = []; % towards right
    
    H_l = horzcat(H_n(:,:,k),zeros(mH,1));
    add_H_l = H_l(:,1) * wl;
    H_l(:,1) = []; % towards left
    
    H_d = vertcat(zeros(1,nH),H_n(:,:,k));
    add_H_d = H_d(mH+1,:) * wd;
    H_d(mH+1,:) = []; % towards down
    
    H_u = vertcat(H_n(:,:,k),zeros(1,nH));
    add_H_u = H_u(1,:) * wu;
    H_u(1,:) = []; % towards up
    
    H_n(:,:,k+1) = wc * H_n(:,:,k) + wl * H_l + wr * H_r +...
        wu * H_u + wd * H_d; % diffusion
    
    H_n(:,nH,k+1) = H_n(:,nH,k+1) + add_H_r; % out of boundary
    H_n(:,1,k+1) = H_n(:,1,k+1) + add_H_l;
    H_n(mH,:,k+1) = H_n(mH,:,k+1) + add_H_d;
    H_n(1,:,k+1) = H_n(1,:,k+1) + add_H_u;
    
%     % test
%     diffusion_error = H_n(:,:,k+1) - H(:,:,i+1);
%     diffusion_error_sum(k,i) = sum(H_n(:,:,k+1) - H(:,:,i+1),'all');
%     
    
end

H(:,:) = H_n(:,:,k+1);



% Previous
H_n = R;
for k = 1:1:diffussion_times

H_r = horzcat(zeros(mH,1),H_n(:,:,k));
H_r(:,nH+1) = [];
H_l = horzcat(H_n(:,:,k),zeros(mH,1));
H_l(:,1) = [];

H_d = vertcat(zeros(1,nH),H_n(:,:,k));
H_d(mH+1,:) = [];
H_u = vertcat(H_n(:,:,k),zeros(1,nH));
H_u(1,:) = [];

H_n(:,:,k+1) = wc*H_n(:,:,k)+wl*H_l+wr*H_r+wu*H_u+wd*H_d;

H_n(:,1,k+1) = H_n(:,2,k+1);
H_n(:,nH,k+1) = H_n(:,nH-1,k+1);
H_n(1,:,k+1) = H_n(2,:,k+1);
H_n(mH,:,k+1) = H_n(mH-1,:,k+1);

end
H2(:,:) = H_n(:,:,k+1);

% Volume error
sum_R = sum(R,'all');
sum_H = sum(H,'all');
sum_H2 = sum(H2,'all');
error1 = sum_H - sum_R
error2 = sum_H2 - sum_R


% test
figure
mesh(X,Y,R)
zlim([-50 40])
xlabel('x[cm]')
ylabel('y[cm]')
zlabel('h[cm]')

figure
mesh(X,Y,H)
zlim([-50 40])
xlabel('x[cm]')
ylabel('y[cm]')
zlabel('h[cm]')

figure
mesh(X,Y,H2)
zlim([-50 40])
xlabel('x[cm]')
ylabel('y[cm]')
zlabel('h[cm]')