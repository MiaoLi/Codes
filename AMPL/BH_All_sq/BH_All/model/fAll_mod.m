

function fAll_mod (rootname)

%filename = ['All_mod_',rootname,'.txt'];

 fid  = fopen('All_mod.txt', 'w+');
%fid  = fopen(filename, 'w+');




% %%%% O2
% rootname = 'O2_p';
% orient = '_o';
% extension = '.mod';
% 
% 
% for i=1:42
%     for j=1:3
%        filename = [rootname, num2str(j), orient, num2str(i), extension];
%        fprintf(fid, '%s ;\n','reset');
%        fprintf(fid, '%s %s;\n','model', filename);
%     end
% end

%%%% Cylinder
%rootname = 'cylinder_p';
orient = '_o';
extension = '.mod';
load Rh;

for i=1:size(Rh,2)
    for j=1:3
       filename = [rootname, num2str(j), orient, num2str(i), extension];
       fprintf(fid, '%s ;\n','reset');
       fprintf(fid, '%s %s;\n','model', filename);
    end
end

% %%%% Sphere
% rootname = 'sphere_p';
% orient = '_o';
% extension = '.mod';
% 
% 
% for i=1:42
%     for j=1:4
%        filename = [rootname, num2str(j), orient, num2str(i), extension];
%        fprintf(fid, '%s ;\n','reset');
%        fprintf(fid, '%s %s;\n','model', filename);
%     end
% end
% 




% %%%% Cube
% rootname = 'cube_p';
% orient = '_o';exit;
% extension = '.mod';
% 
% 
% for i=1:42
%     for j=1:4
%        filename = [rootname, num2str(j), orient, num2str(i), extension];
%        fprintf(fid, '%s ;\n','reset');
%        fprintf(fid, '%s %s;\n','model', filename);
%     end
% end


% %%%% rCube
% rootname = 'rcube_p';
% orient = '_o';
% extension = '.mod';
% 
% 
% for i=1:42
%     for j=1:4
%        filename = [rootname, num2str(j), orient, num2str(i), extension];
%        fprintf(fid, '%s ;\n','reset');
%        fprintf(fid, '%s %s;\n','model', filename);
%     end
% end


% %%%% rCube
% rootname = 'test_p';
% orient = '_o';
% extension = '.mod';


% for i=1:42
%     for j=1:1 %1:4
%        filename = [rootname, num2str(j), orient, num2str(i), extension];
%        fprintf(fid, '%s ;\n','reset');
%        fprintf(fid, '%s %s;\n','model', filename);
%     end
% end



fclose(fid);
