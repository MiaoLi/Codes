function fcreate_mod_file(rootname, shape)

%shape: parameters for the object;
e1 = shape(1);
e2 = shape(2);
s1 = shape(3);
s2 = shape(4);
s3 = shape(5);
kx = shape(6);
ky = shape(7);


%rootname = 'cylinder_p';
%rootname = 'shpere_p';
%rootname = 'rcube_p';
%rootname = 'cube_p';
%rootname = 'small_rcube_p';

%%%%% Realistic objects
%rootname = 'bottle_part4_p';
%rootname = 'pencil_part3_p';
% rootname = 'glass_part2_p';

%rootname = 'test_p';



orient = '_o';
extension = '.mod';


% Initial position and orientation of the hand
load Rh;
% Rh 6xsample vector for initial orientation

s = s1;
% p1 = [-3*s; 3*s; 3*s];
% p2 = [-2*s; 2*s; 2*s];
% p3 = [-s; s; s];
% p4 = [0; 3*s; 0];
% p5 = [0;3*s ; -s];

p1 = [3*s; 3*s; 3*s];
p2 = [0; 3*s; 0];
p3 = [0;0 ; 3*s];


qh = [p1 p2 p3];



for i=1:size(Rh,2)
    for j=1:3
       
       filename = [rootname, num2str(j), orient, num2str(i), extension];
       sol_filename = ['sol_',rootname, num2str(j), orient, num2str(i), '.txt'];
      
       fid  = fopen(filename, 'w+');
       %fid2 = fopen('empty_script.mod','r');
      fid2 = fopen('BH_tem1.mod','r');
      
       %copyfile('empty_script.mod', filename);
       while(~feof(fid2))
            line = fgetl(fid2);
            if ( strfind(line, 'param s1 :=') )
                fprintf(fid, '%s %f ;\n',line, s1);
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, s2);
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, s3);
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, e1);
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, e2);
            else if ( strfind(line, 'let cnh1 :=') ) 
                fprintf(fid, '%s %f ;\n',line, Rh(1,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, Rh(2,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, Rh(3,i))
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, Rh(4,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, Rh(5,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, Rh(6,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, qh(1,j));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, qh(2,j));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, qh(3,j));
                
                else if ( strfind(line, 'display solve_message') )
                        fprintf(fid, '%s %s  %s ;\n',line, '>>', sol_filename);
                        for k=1:49
                            line = fgetl(fid2);
                            fprintf(fid, '%s %s  %s ;\n',line, '>>', sol_filename);
                        end
                else
                  fprintf(fid, '%s \n',line);
                    end
                end
            end
            

       end
       
       fclose(fid);
       fclose(fid2);
       
    end
end





