%% Initialization
clear all
close all

excavator_data;

%% parameters
P = eye((nx+1)*(ny+1));   % P matrix for MPC

Vd = sum(sum(abs(R-H0))*(lx/nx)*(ly/ny));    % Volume of desired shape
Vv = sum(sum(abs(vessel-H0))*(lx/nx)*(ly/ny));    % Vessel volume

nl = 2.5;   % Vd/ƒoƒPƒbƒg“à“y»‚ÌÅ‘å‘ÌÏ
Vmin = 0;   %ƒoƒPƒbƒg“à“y»‚ÌÅ¬‘ÌÏ
Vmax = Vv/nl;  %ƒoƒPƒbƒg“à“y»‚ÌÅ‘å‘ÌÏ
%Vmax = 1.793466009964965e+05; % old bucket
%Vmax = 2.6e+05; % new bucket
cxmin = xr+blx/4;   %“y»”r“yˆÊ’u‚Ì x Å¬’l
cxmax = xf-blx/4;   %“y»”r“yˆÊ’u‚Ì x Å‘å’l
cymin = yr+bly/2;   %“y»”r“yˆÊ’u‚Ì y Å¬’l
cymax = yl-bly/2;   %“y»”r“yˆÊ’u‚Ì y Å‘å’l

cmin = [cxmin cymin]; 
cmax = [cxmax cymax]; 


xpc = 20;  %ƒVƒ‡ƒxƒ‹‚ÌƒA[ƒ€‚Æƒ_ƒ“ƒv•Ç–Ê‚Æ‚ÌŒð“_‚Ì x À•W
Pc = [xpc yr];  %ƒVƒ‡ƒxƒ‹‚ÌƒA[ƒ€‚Æƒ_ƒ“ƒv•Ç–Ê‚Æ‚ÌŒð“_

V0 = Vmax; %deposit volume (1 bucket)
c0 = Pc;

%@ŽÀÛ‚Ì•ªŽU
lambdax = 740; %À•W•ÏŠ·‘O‚Ì x •ûŒü‚Ì•ªŽU
lambday = 1277; %À•W•ÏŠ·‘O‚Ì y •ûŒü‚Ì•ªŽU
% lambdax = 750; %À•W•ÏŠ·‘O‚Ì x •ûŒü‚Ì•ªŽU
% lambday = 1400; %À•W•ÏŠ·‘O‚Ì y •ûŒü‚Ì•ªŽU

Sigma = [lambdax 0;0 lambday];

% —\‘ª•ªŽU
Sigma_bar = 0.5 * Sigma;
the = atan((Pc(2)-Pe(2))/(Pc(1)-Pe(1)));    %ƒVƒ‡ƒxƒ‹‚ÌƒA[ƒ€‚Æƒ_ƒ“ƒv•Ç–Ê‚Ì‚È‚·Šp

T = 10; %time per one trajectory
dt = 0.1; %time difference of continuous dep
SPmax = 100; %percentage of soil on the bucket
Np = 3; %”r“y‰ñ”
n = 3;  %“ü—Í‚ÌŽŸ”

%% Å“K‰»‚É‚æ‚é“ü—Í‚ÌŒˆ’è
H_now = H0;
w0 = [1 1];       %w1 w2 --> w(t) = w1t + w2t^2
p0 = [0 0 0 0 0]; %px1 px2 py1 py2 py3 --> px(t) = px1t + px2t^2
u0 = [w0 p0];     %                    --> py(t) = py1t + py2t^2 + py3
u0 = repmat(u0,1,Np); %u0 = 1*21 vectors

umin = [cmin, Vmin];
umax = [cmax, Vmax];

A = [];  % Ax <= b
b = [];
for index=1:Np
    A(index,((index-1)*(size(u0,2)/Np)+3)) = T;         %at t=10 x<cxmax
    A(index,((index-1)*(size(u0,2)/Np)+4)) = T^2;
    
    A(index+3,((index-1)*(size(u0,2)/Np)+3)) = -T;      %at t=10 x>cxmin
    A(index+3,((index-1)*(size(u0,2)/Np)+4)) = -T^2;
    
    A(index+6,((index-1)*(size(u0,2)/Np)+5)) = T;       %at t=10 y<cymax
    A(index+6,((index-1)*(size(u0,2)/Np)+6)) = T^2;     
    A(index+6,((index-1)*(size(u0,2)/Np)+7)) = 1;
    
    A(index+9,((index-1)*(size(u0,2)/Np)+5)) = -T;      %at t=10 y>cymin
    A(index+9,((index-1)*(size(u0,2)/Np)+6)) = -T^2; 
    A(index+9,((index-1)*(size(u0,2)/Np)+7)) = -1;
    
    A(index+12,((index-1)*(size(u0,2)/Np)+7)) = 1;      %py3 < cymax initial y
    
    A(index+15,((index-1)*(size(u0,2)/Np)+7)) = -1;     %py3 > cymin initial y
   
    A(index+18,((index-1)*(size(u0,2)/Np)+1)) = -T;     %at t=10 soil drop > 0
    A(index+18,((index-1)*(size(u0,2)/Np)+2)) = -T^2;
end

b = [cxmax;cxmax;cxmax;
    -cxmin;-cxmin;-cxmin;
    cymax;cymax;cymax;
     -cymin;-cymin;-cymin;
     cymax;cymax;cymax;
     -cymin;-cymin;-cymin;
     0;0;0];

Aeq = []; % Ax = b
beq = [];

for i = 1:Np
    Aeq(i,((i-1)*(size(u0,2)/Np)+3)) = 1;       %at T=10 vx = 0
    Aeq(i,((i-1)*(size(u0,2)/Np)+4)) = 2*T;
    
    Aeq(i+3,((i-1)*(size(u0,2)/Np)+5)) = 1;     %at T=10 vy = 0
    Aeq(i+3,((i-1)*(size(u0,2)/Np)+6)) = 2*T;
    Aeq(i+3,((i-1)*(size(u0,2)/Np)+7)) = 0;
end
beq = [0;0;0;0;0;0];

lb = []; % u >= lb
ub = []; % u <= ub
options = optimoptions(@fmincon,'MaxFunctionEvaluations',6000);
tic
u0 = fmincon(@(u)objfun_2d_fl_diffusion(R,X,Y,u,Sigma,H_now,Np,P,Pe,xf,yr,yl,dt,T,V0),u0,A,b,Aeq,beq,lb,ub,[],options);
toc

%% ó‘Ô‘JˆÚ

for t = 0:dt:T %drop T/dT points per trajectory
    TT = int16(t*10)+1;
    depv1(TT) = (t)*u0(1) + (t^2)*u0(2); 
    depx1(TT) = (t)*u0(3) + (t^2)*u0(4); 
    depy1(TT) = (t)*u0(5) + (t^2)*u0(6) + u0(7); 
    depv2(TT) = (t)*u0(8) + (t^2)*u0(9); 
    depx2(TT) = (t)*u0(10) + (t^2)*u0(11); 
    depy2(TT) = (t)*u0(12) + (t^2)*u0(13) + u0(4); 
    depv3(TT) = (t)*u0(15) + (t^2)*u0(16); 
    depx3(TT) = (t)*u0(17) + (t^2)*u0(18); 
    depy3(TT) = (t)*u0(19) + (t^2)*u0(20) + u0(21); 
end

figure
plot(depx1,depy1)
hold on
plot(depx2,depy2)
plot(depx3,depy3)
legend('1st dep','2nd dep','3rd dep')
title('Soil Deposition Trajectory')
xlabel('x[mm]')
ylabel('y[mm]')
rectangle('Position',[0 -80 170 160],'EdgeColor','r'); %vessel
xlim([0 200])
ylim([-100 100])
hold off

N = 0:1:100;
figure
plot(N,depv1)
hold on
plot(N,depv2)
plot(N,depv3)
legend('1st depv','2nd depv','3rd depv')
title('Soil Deposition Volume')
hold off

H(:,:,1)= H0;
nd = 7;
for i = 1:Np
    s(:,:,i) = H0*0;
    Vmax = 0;
    for t = 0:dt:T
        Vmax = Vmax + (t)*u0((i-1)*nd+1) + (t^2)*u0((i-1)*nd+2); 
    end   
    for t = 0:dt:T
        TT = int16(t*10)+1;
        depx = (t)*u0((i-1)*nd+3) + (t^2)*u0((i-1)*nd+4); 
        depy = (t)*u0((i-1)*nd+5) + (t^2)*u0((i-1)*nd+6) + u0((i-1)*nd+7);
        depv   = (t)*u0((i-1)*nd+1) + (t^2)*u0((i-1)*nd+2); 
        the(i) = atan((depy-Pe(2))/(depx-Pe(1)));
        V = [depv/Vmax*V0];  
        c = [depx,depy];
        %V = [V0*dt/T];
        g(:,:,i) = function_input_2d(X,Y,c,V,Sigma,the(i),xf,yr,yl);
        s(:,:,i) = s(:,:,i) + g(:,:,i); 
    end
    H(:,:,i+1) = H(:,:,i) + s(:,:,i);
% ŠgŽU
H_n = H(:,:,i+1);
wl = 0.15;
wr = wl;
wu = 0.15;
wd = wu;
wc = 1-wl-wr-wu-wd;

[mH,nH] = size(H_n);
j = 50;

%figure
%mesh(X,Y,H(:,:,i+1))
%zlim([-50 40])
%figure
for k = 1:1:50

    
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



%mesh(X,Y,H_n(:,:,k+1))
%zlim([-50 40])
%xlabel('x[cm]')
%ylabel('y[cm]')
%zlabel('h[cm]')
end

H(:,:,i+1) = H_n(:,:,j+1);
sum(sum(H(:,:,i+1) - H(:,:,i)))*(lx/nx)*(ly/ny)/V; 
end
dimH = size(H);

%% Œë·ŒvŽZ
err1 = abs(R-H(:,:,dimH(3)));
err2 = sum(sum(err1))*(lx/nx)*(ly/ny)/Vd;
sum(sum(H(:,:,dimH(3))))
MedH =  imgaussfilt(H(:,:,dimH(3)));
sum(sum(MedH))

%% •`‰æ
for i = 1:dimH(3)
figure
mesh(X,Y,H(:,:,i))
xlabel('x[mm]')
ylabel('y[mm]')
zlabel('h[mm]')
zlim([-50 40])
end


figure
mesh(X,Y,R)
xlabel('x[mm]')
ylabel('y[mm]')
zlabel('h[mm]')
zlim([-50 40])

figure
mesh(X,Y,vessel)
xlabel('x[mm]')
ylabel('y[mm]')
zlabel('h[mm]')
zlim([-50 40])