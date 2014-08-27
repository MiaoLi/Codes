function fcreate_mod_file(rootname, shape)

%shape: parameters for the object;
e1 = shape(1);
e2 = shape(2);
s1 = shape(3);
s2 = shape(4);
s3 = shape(5);
kx = shape(6);
ky = shape(7);


rootname = 'cylinder_p';
%rootname = 'shpere_p';
%rootname = 'rcube_p';
%rootname = 'cube_p';
%rootname = 'small_rcube_p';


extension = '.mod';


% Initial positions
load IniPoint;


s = s1;
% p1 = [-3*s; 3*s; 3*s];
% p2 = [-2*s; 2*s; 2*s];
% p3 = [-s; s; s];
% p4 = [0; 3*s; 0];
% p5 = [0;3*s ; -s];



for i=1:size(IniPoint,2)
       
       filename = [rootname, num2str(i), extension];
       sol_filename = ['sol_',rootname, num2str(i),'.txt'];
     
       fid  = fopen(filename, 'w+');
       
      fid2 = fopen('point_temple.mod','r');
      
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
            else if ( strfind(line, 'let p[1,1] :=') ) 
                fprintf(fid, '%s %f ;\n',line, IniPoint(1,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, IniPoint(2,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, IniPoint(3,i))
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, IniPoint(4,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, IniPoint(5,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, IniPoint(6,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, IniPoint(7,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, IniPoint(8,i));
                line = fgetl(fid2);
                fprintf(fid, '%s %f ;\n',line, IniPoint(9,i));
                
                else if ( strfind(line, 'display solve_message') )
                        fprintf(fid, '%s %s  %s ;\n',line, '>>', sol_filename);
                        for k=1:8
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





