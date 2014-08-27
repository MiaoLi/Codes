function plot_all_grasp
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
                figure(K)
                plot_grasp(txtFilename), hold on;
                figuretitle=['P',num2str(position),'O',num2str(orient)];
                title(figuretitle);
             end        
        
        end
         fclose(fid); 
    end
end
