function plot_grasp()

% Shape object
rootname = 'sol_cylinder_p';
nbFC=0;
GraspPoint=[];

for i=1:1:10
    figure(i)
    s1 = 0.035;
    s2 = 0.035;
    s3 = 0.065;
    e1 = 0.25;
    e2 = 1;
    kx = 0;
    ky = 0;
    points = plot_object(e1,e2,24,s1,s2,s3,kx,ky,0,0);
    hold on , plot3(points(:,1),points(:,2),points(:,3),'color',[139,139,131]/255);
    
    filename=[rootname,num2str(i),'.txt'];
    fid = fopen(filename,'r');
    line = fgetl(fid);
    if ( strfind(line, 'solve_message =') )
        line = fgetl(fid);
        if ( strfind(line, 'Optimal') )
             nbFC=nbFC+1
            [solve_time,points] = readResult(filename);
%             GraspingPoint(1:9,nbFC)=points(:);
            [x,y,z]= sphere;
            for i=1:3
                surf(0.005*x+points(i,1),0.005*y+points(i,2),0.005*z+points(i,3),'EdgeColor','none');hold on;
            end
            box on; axis equal;
            view(30,45);
        end
    end
    fclose(fid);
end
save GraspingPoint;
% print('-opengl', '-dpdf', '-r600', filename);