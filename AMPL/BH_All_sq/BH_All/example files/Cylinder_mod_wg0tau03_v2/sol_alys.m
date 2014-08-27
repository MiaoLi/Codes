%% analysis the result
clc;
clear;
close all
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
               [solve_time,distance, Hand_pos, Finger_joint] = readResult(txtFilename);
                              

% for j=1:8
%     teta = [teta atan2(sn(j), cn(j))*180/pi;];
% end
% RotMat=[h1,h2,h3];
% RotVec=vrrotmat2vec(RotMat);
% RTKinput_mu05w06(K,:)=[qhand',RotVec, teta(1,1:3),0,teta(1,4:6),0,teta(1,7:8),0,distance,torque];

             
             end        
        
        end
         fclose(fid); 
    end
end
