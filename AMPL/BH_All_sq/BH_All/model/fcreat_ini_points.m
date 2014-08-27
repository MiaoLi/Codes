function fcreat_ini_points
% creat the initial points for hand orientation
i=1;
for rot_angle1=0:pi/3:2*pi
   for rot_angle2=0:pi/3:2*pi
       for rot_angle3=0:pi/5:pi
cnh1 = cos(rot_angle1);
cnh2 = cos(rot_angle2);
cnh3 = cos(rot_angle3);
snh1 = sin(rot_angle1);
snh2 = sin(rot_angle2);
snh3 = sin(rot_angle3);
Rh(1:6,i)=[cnh1;cnh2;cnh3;snh1;snh2;snh3];
i=i+1;
       end
   end
end
save Rh
% size(Rh)
