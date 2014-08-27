% function plot_barrett(PHand,Fjoint)
function plot_barrett(cnh1,cnh2,cnh3,snh1,snh2,snh3, Hx, Hy, Hz,  cnf11, cnf21, cnf31, cnf12, cnf22, cnf32, cnt1, cnt2, snf11, snf21, snf31, snf12, snf22, snf32,  snt1, snt2);   
% PHand: 6x1 vector, the hand position and orientation;
% Fjoint: 8x1 vector, the finger joint angles;

% cnh1=cos(PHand(1));
% cnh2=cos(PHand(2));
% cnh3=cos(PHand(3));
% snh1=sin(PHand(1));
% snh2=sin(PHand(2));
% snh3=sin(PHand(3));
% Hx=PHand(4);
% Hy=PHand(5);
% Hz=PHand(6);
% 
% cnf11=cos(Fjoint(1));  % finger 1;
% cnf21=cos(Fjoint(2));  
% cnf31=cos(Fjoint(3));
% cnf12=cos(Fjoint(4));  % finger 2
% cnf22=cos(Fjoint(5));
% cnf32=cos(Fjoint(6));
% cnt1=cos(Fjoint(7));   %thumb
% cnt2=cos(Fjoint(8));
% 
% snf11=sin(Fjoint(1));
% snf21=sin(Fjoint(2));
% snf31=sin(Fjoint(3));
% snf12=sin(Fjoint(4));
% snf22=sin(Fjoint(5));
% snf32=sin(Fjoint(6));
% snt1=sin(Fjoint(7));
% snt2=sin(Fjoint(8));

% syms cnh1 cnh2 cnh3 snh1 snh2 snh3 Hx Hy Hz;  %% The Z-Y-X Euler angle and position of hand relative to object
% syms cnf11 cnf21 cnf31 cnf12 cnf22 cnf32 snf11 snf21 snf31 snf12 snf22 snf32;
% syms cnt1 cnt2 snt1 snt2;


Hhand=[    cnh1*cnh2 cnh1*snh2*snh3-snh1*cnh3 cnh1*snh2*cnh3+snh1*snh3 Hx;
           snh1*cnh2 snh1*snh2*snh3+cnh1*cnh3 snh1*snh2*cnh3-cnh1*snh3 Hy;
           -snh2     cnh2*snh3                cnh2*cnh3                Hz;
             0          0                        0                     1;];  % the hand frame relative to the object frame
         
Tf11 = Hhand*[1 0 0 0; 0 -1 0 -0.025; 0 0 -1 -0.041; 0 0 0 1]*[cnf11 -snf11 0 0; snf11 cnf11 0 0; 0 0 1 0; 0 0 0 1]; % 0 < theta < 180 
Tf21 = [1 0 0 0.05; 0 0 1 0; 0 -1 0 -0.034; 0 0 0 1]*[cnf21 -snf21 0 0; snf21 cnf21 0 0; 0 0 1 0; 0 0 0 1]; % 0 < theta < 140
Tf31 = [1 0 0 0.07; 0 1 0 0.003; 0 0 1 0; 0 0 0 1]*[cnf31 -snf31 0 0; snf31 cnf31 0 0; 0 0 1 0; 0 0 0 1]; % 42 < theta < 90
Tf1 = Tf11*Tf21*Tf31*[1 0 0 0.056; 0 1 0 0.0024; 0 0 1 0; 0 0 0 1]; %fingertip1 in the object reference frame

Tf12 = Hhand*[1 0 0 0; 0 1 0 0.025; 0 0 1 -0.041; 0 0 0 1]*[cnf12 -snf12 0 0; snf12 cnf12 0 0; 0 0 1 0; 0 0 0 1]; % 0 < theta < 180 
Tf22 = [1 0 0 0.05; 0 0 -1 0; 0 1 0 0.034; 0 0 0 1]*[cnf22 -snf22 0 0; snf22 cnf22 0 0; 0 0 1 0; 0 0 0 1]; % 0 < theta < 140
Tf32 = [1 0 0 0.07; 0 1 0 0.003; 0 0 1 0; 0 0 0 1]*[cnf32 -snf32 0 0; snf32 cnf32 0 0; 0 0 1 0; 0 0 0 1]; % 42 < theta < 90
Tf2 = Tf12*Tf22*Tf32*[1 0 0 0.056; 0 1 0 0.0024; 0 0 1 0; 0 0 0 1]; %fingertip2 in the object reference frame

Tt1 = Hhand*[-1 0 0 -0.05; 0 0 1 0; 0 1 0 -0.007; 0 0 0 1]*[cnt1 -snt1 0 0; snt1 cnt1 0 0; 0 0 1 0; 0 0 0 1]; % 0 < theta < 140 
Tt2 = [1 0 0 0.07; 0 1 0 0.003; 0 0 1 0; 0 0 0 1]*[cnt2 -snt2 0 0; snt2 cnt2 0 0; 0 0 1 0; 0 0 0 1]; % 42 < theta < 90
Tt = Tt1*Tt2*[1 0 0 0.056; 0 1 0 0.0024; 0 0 1 0; 0 0 0 1];       %thump in the object reference frame

%fingertips in object reference frame

fingertip1 = Tf1;
fingertip1 = fingertip1(1:3,end);

fingertip2 = Tf2;
fingertip2 = fingertip2(1:3,end);

fingertip3 = Tt;
fingertip3 = fingertip3(1:3,end);


% % the fingertip normal

fingertipn1 = Tf1;
nf1 = fingertipn1(1:3,2);

fingertipn2 = Tf2;
nf2 = fingertipn2(1:3,2);

fingertipn3 = Tt;
nf3 = fingertipn3(1:3,2);

%%
f_hand=Hhand(1:3,4);

f1_p1=Tf11;
f1_p1=f1_p1(1:3,4);

f1_p2=Tf11*Tf21;
f1_p2=f1_p2(1:3,4);

f1_p3=Tf11*Tf21*Tf31;
f1_p3=f1_p3(1:3,4);

f2_p1=Tf12;
f2_p1=f2_p1(1:3,4);

f2_p2=Tf12*Tf22;
f2_p2=f2_p2(1:3,4);

f2_p3=Tf12*Tf22*Tf32;
f2_p3=f2_p3(1:3,4);

f3_p1=Tt1;
f3_p1=f3_p1(1:3,4);

f3_p2=Tt1*Tt2;
f3_p2=f3_p2(1:3,4);

%%

plot3(Hhand(1,end),Hhand(2,end),Hhand(3,end),'r.','MarkerSize',20), hold on;
quiver3(Hhand(1,4),Hhand(2,4),Hhand(3,4),Hhand(1,1),Hhand(2,1),Hhand(3,1), 0.02,'r-', 'LineWidth',1), hold on;
quiver3(Hhand(1,4),Hhand(2,4),Hhand(3,4),Hhand(1,2),Hhand(2,2),Hhand(3,2), 0.02,'g-', 'LineWidth',1), hold on;
quiver3(Hhand(1,4),Hhand(2,4),Hhand(3,4),Hhand(1,3),Hhand(2,3),Hhand(3,3), 0.02,'b-', 'LineWidth',1), hold on;

%
FL=[f1_p1,f1_p2];
plot3(FL(1,:),FL(2,:),FL(3,:),'g*-','LineWidth',5), hold on;
FL=[f1_p2,f1_p3];
plot3(FL(1,:),FL(2,:),FL(3,:),'g*-','LineWidth',5), hold on;
FL=[f1_p3,fingertip1];
plot3(FL(1,:),FL(2,:),FL(3,:),'g*-','LineWidth',5), hold on;
quiver3(fingertip1(1),fingertip1(2),fingertip1(3),nf1(1),nf1(2),nf1(3), 0.02,'g-', 'LineWidth',2), hold on;
% [x,y,z] = sphere;
% x=0.005*x;
% y=0.005*y;
% z=0.005*z;
% surf(x+fingertip1(1),y+fingertip1(2),z+fingertip1(3), 'FaceColor','g', 'EdgeColor','none'), hold on;

grid on;
%
FL=[f2_p1,f2_p2];
plot3(FL(1,:),FL(2,:),FL(3,:),'b*-','LineWidth',5), hold on;
FL=[f2_p2,f2_p3];
plot3(FL(1,:),FL(2,:),FL(3,:),'b*-','LineWidth',5), hold on;
FL=[f2_p3,fingertip2];
plot3(FL(1,:),FL(2,:),FL(3,:),'b*-','LineWidth',5), hold on;
quiver3(fingertip2(1),fingertip2(2),fingertip2(3),nf2(1),nf2(2),nf2(3), 0.02,'b-', 'LineWidth',2), hold on;
% [x,y,z] = sphere;
% x=0.005*x;
% y=0.005*y;
% z=0.005*z;
% surf(x+fingertip2(1),y+fingertip2(2),z+fingertip2(3), 'FaceColor','b', 'EdgeColor','none'), hold on;
%
FL=[f3_p1,f3_p2];
plot3(FL(1,:),FL(2,:),FL(3,:),'r*-','LineWidth',5), hold on;
FL=[f3_p2,fingertip3];
plot3(FL(1,:),FL(2,:),FL(3,:),'r*-','LineWidth',5), hold on;
quiver3(fingertip3(1),fingertip3(2),fingertip3(3),nf3(1),nf3(2),nf3(3), 0.02,'r-', 'LineWidth',2), hold on;
% [x,y,z] = sphere;
% x=0.005*x;
% y=0.005*y;
% z=0.005*z;
% surf(x+fingertip3(1),y+fingertip3(2),z+fingertip3(3), 'FaceColor','r', 'EdgeColor','none'), hold on;

