function RTK_sol
K=0;
for position=1:3
     for orient=1:48

     txtFilename=['sol_cylinder_p',num2str(position),'_o',num2str(orient),'.txt'];    
        fid = fopen(txtFilename,'r');
        line = fgetl(fid);
        if ( strfind(line, 'solve_message =') )
             line = fgetl(fid);
             if ( strfind(line, 'Optimal') )
                K=K+1;
               [solve_time,distance, hand_pos, Finger_joint] = readResult(txtFilename);
             
               cnh1=hand_pos(1);
               cnh2=hand_pos(2);
               cnh3=hand_pos(3);
               snh1=hand_pos(4);
               snh2=hand_pos(5);
               snh3=hand_pos(6);
               Hx=hand_pos(7);
               Hy=hand_pos(8);
               Hz=hand_pos(9);
             
             Hhand=[    cnh1*cnh2 cnh1*snh2*snh3-snh1*cnh3 cnh1*snh2*cnh3+snh1*snh3 Hx;
                        snh1*cnh2 snh1*snh2*snh3+cnh1*cnh3 snh1*snh2*cnh3-cnh1*snh3 Hy;
                        -snh2     cnh2*snh3                cnh2*cnh3                Hz;
                         0          0                        0                     1;];
             fj11=atan2(Finger_joint(9),Finger_joint(1));
             fj12=atan2(Finger_joint(10),Finger_joint(2));
             fj13=atan2(Finger_joint(11),Finger_joint(3));
             fj21=atan2(Finger_joint(12),Finger_joint(4));
             fj22=atan2(Finger_joint(13),Finger_joint(5));
             fj23=atan2(Finger_joint(14),Finger_joint(6));
             fj31=atan2(Finger_joint(15),Finger_joint(7));
             fj32=atan2(Finger_joint(16),Finger_joint(8));
             OptConfig(K,:)=[Hhand(1:3,1);Hhand(1:3,2);Hhand(1:3,3);Hhand(1:3,4);fj11;...
                 fj12;fj13;fj21;fj22;fj23;fj31;fj32]';
            
             end        
        
        end
        
     end
fid_rtk=fopen('OptConfig.txt','w');
fprintf(fid_rtk, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f;\n',OptConfig');
 fclose(fid); 
end

%Finger_joint=[cnf11;cnf21;cnf31;cnf12;cnf22;cnf32;cnt1;cnt2;snf11;snf21;snf31;snf12;snf22;snf32;snt1;snt2];

