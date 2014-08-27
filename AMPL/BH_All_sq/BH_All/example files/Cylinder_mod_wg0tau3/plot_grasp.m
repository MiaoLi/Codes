function plot_grasp(filename)

% Shape object
s1 = 0.0375;
s2 = 0.0375;
s3 = 0.065;
e1 = 0.25;
e2 = 1;
kx = 0;
ky = 0;
points = plot_object(e1,e2,24,s1,s2,s3,kx,ky,0,0);
hold on , plot3(points(:,1),points(:,2),points(:,3),'color',[139,139,131]/255);
% tri=delaunay(points(:,1),points(:,2),points(:,3));
% trisurf(tri,points(:,1),points(:,2),points(:,3),'EdgeColor','none','FaceColor','g');
[solve_time,distance, Hand_pos, Finger_joint] = readResult(filename);
cnh1 = Hand_pos(1);
cnh2 = Hand_pos(2);
cnh3 = Hand_pos(3);
snh1 = Hand_pos(4);
snh2 = Hand_pos(5);
snh3 = Hand_pos(6);
Hx =  Hand_pos(7);
Hy = Hand_pos(8);
Hz = Hand_pos(9);

cnf11 = Finger_joint(1);
cnf21 = Finger_joint(2);
cnf31 = Finger_joint(3);
cnf12 = Finger_joint(4);
cnf22 = Finger_joint(5);
cnf32 = Finger_joint(6);
cnt1 = Finger_joint(7);
cnt2 = Finger_joint(8);
snf11 = Finger_joint(9);
snf21 = Finger_joint(10);
snf31 = Finger_joint(11);
snf12 = Finger_joint(12);
snf22 = Finger_joint(13);
snf32 = Finger_joint(14);
snt1 = Finger_joint(15);
snt2 = Finger_joint(16);

plot_barrett(cnh1,cnh2,cnh3,snh1,snh2,snh3, Hx, Hy, Hz,  cnf11, cnf21, cnf31, cnf12, cnf22, cnf32, cnt1, cnt2, snf11, snf21, snf31, snf12, snf22, snf32, snt1, snt2);   

