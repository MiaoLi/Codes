%function points = superquad_echellebt(n,e,m,a1,a2,a3,kx,ky)
function points = plot_object(e1,e2,m,a1,a2,a3,kx,ky,k,alpha)
% function [xx,yy,zz] = superquad_echelle(n,e,m,a1,a2,a3,k,alpha)

%SUPERQUAD Barr's "superquadrics" ellipsoid.
%   [x,y,z] = SUPERQUAD(n,e,m) is a generalized ellipsoid with
%   n = vertical roundness, e = horizontal roundness and m facets.
%   If values of n and e are not given, random values are supplied.
%   The default value of m is 24.
%
%   SUPERQUAD(...) , with no output arguments, does a SURF plot.
%
%   Ref: A. H. Barr, IEEE Computer Graphics and Applications, 1981,
%        or, Graphics Gems III, David Kirk, editor, 1992.
%
%   See also XPQUAD.

%   Copyright 1984-2003 The MathWorks, Inc.
%   $Revision: 5.8.4.2 $  $Date: 2004/04/10 23:25:49 $

if nargin < 3+5, m = 24; end
if nargin < 2+5, e = max(0,1+randn); end
if nargin < 1+5, n = max(0,1+randn); end
  
s1 = -1:0.05:0;
s2 = 0:0.05:1;
eta= [- asin(s1.^(1/e1)) asin(s2.^(1/e1))];

uu=((-m:1:m)*pi)/m   ;
omega = uu ;

u = [eta omega];
v = u';%eta' ;


size(u)
size(v)


cosv = cos(v)   ; sinv = sin(v)     ;
cosu = cos(u)   ; sinu = sin(u)     ;
% cosv(1) = 0     ; cosv(m+1) = 0     ;
% sinu(1) = 0     ; sinu(m+1) = 0     ;

t = sign(cosv) .* abs(cosv).^e1          ;
x = a1*t * (sign(cosu) .* abs(cosu).^e2 );
y = a2*t * (sign(sinu) .* abs(sinu).^e2 );
z = a3*(sign(sinv) .* abs(sinv).^e1 ) *  ones(size(u));

size(x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% en tenant compte du tapering
for i=1:size(x,1)
    for j=1:size(x,2)
    x(i,j)      =   x(i,j)*( (kx*z(i,j)/a3)+1 );
    y(i,j)      =   y(i,j)*( (ky*z(i,j)/a3)+1 );
    end
end

%gammat=[];
% %pour le bending
% for i=1:size(x,1)
%     for j=1:size(x,2)
%         
%         R       =   cos(alpha-atan(y(i,j)/x(i,j)))*sqrt(x(i,j)^2+y(i,j)^2)  ;  
%         gamma   =   atan(z(i,j)/(inv(k)-R))                  ;
%         gammat  =   [gammat gamma];
%         r       =   inv(k)-sqrt(z(i,j)^2+(inv(k)-R)^2)       ;
%      
%         x(i,j)    =   x(i,j)+cos(alpha)*(R-r)   ;
%         y(i,j)    =   y(i,j)+sin(alpha)*(R-r)   ;
%         z(i,j)    =   sin(gamma)*(inv(k)-r)     ;
%         
%     end
%  end
% mini=min(gammat);
% maxi=max(gammat);
% gammav=maxi-mini;


points=[];
for i=1:size(z,1)
    for j=1:size(z,2)
        points=[points;x(i,j) y(i,j) z(i,j)];
    end
end
% hold on , plot3(points(:,1),points(:,2),points(:,3),'y');


% %%% the normal
% p = points(500,:)
% plot3(p(1),p(2),p(3),'*r')
%     np = [ -( (p(1)/a1)^(2/e2) + (p(2)/a2)^(2/e2) )^(e2/e1-1)*(p(1)/a1)^(2/e2-1) ;
%         -( (p(1)/a1)^(2/e2) + (p(2)/a2)^(2/e2) )^(e2/e1-1)*(p(2)/a2)^(2/e2-1); -(p(3)/a3)^(2/e1-1)  ] ;
%     t = 100;
%     s = 1;
%     quiver3(p(1)*s, p(2)*s, p(3)*s, np(1)/t, np(2)/t, np(3)/t, 'r'); 
%     
% p = points(2000,:)
% plot3(p(1),p(2),p(3),'*r')
%     np = [ -( (p(1)/a1)^(2/e2) + (p(2)/a2)^(2/e2) )^(e2/e1-1)*(p(1)/a1)^(2/e2-1) ;
%         -( (p(1)/a1)^(2/e2) + (p(2)/a2)^(2/e2) )^(e2/e1-1)*(p(2)/a2)^(2/e2-1); -(p(3)/a3)^(2/e1-1)  ] ;
%     t = 100;
%     s = 1;
%     quiver3(p(1)*s, p(2)*s, p(3)*s, np(1)/t, np(2)/t, np(3)/t, 'r');     
    

% figure(2),
% surf(x,y,z,z);