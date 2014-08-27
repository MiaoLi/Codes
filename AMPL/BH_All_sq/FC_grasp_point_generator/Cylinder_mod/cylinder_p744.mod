reset;  
option solver ipopt;  
  
option ipopt_options "max_iter=5000 tol=10e-3 acceptable_tol=10e-2 mu_strategy=adaptive bound_relax_factor=.1 dual_inf_tol=1 halt_on_ampl_error yes";  
  
param pi := 3.14159 ;  
 
# number of fingers 
param nb_doigts := 3 ;  
 
# the slack variable 
param dv := 0.02;  
  
param nb_segments := 4 ; #nb de segments aveclequel on segmente le cone de frottement  
param mu := 0.8; #coefficient de frottement  
  
# Superquadric shape and size:   
param s1 :=   0.035000 ;
param s2 :=   0.035000 ;
param s3 :=    0.065000 ;
param epsilon1 :=    0.250000 ;
param epsilon2 :=    1.000000 ;
  
# The fingertips positions in the object reference frame are given by pij:  
var  p {j in 1..3, k in 1..3};  
  
# The outward normal vector at a point p of the superquadric is given by np: 
#j: finger; k: vector component;  
var np{j in 1..3, k in 1..3} ;  
 
# The contact frame: 
var vp{j in 1..3, k in 1..3}; 
var vt1{j in 1..3, k in 1..3}; 
var vt2{j in 1..3, k in 1..3};  
  
var phi{ i in 1..nb_segments, j in 1..nb_doigts} >= 0 ;  
var l{i in 1..nb_segments, j in 1..nb_doigts, k in 1..3} ;  
var W{i in 1..nb_segments, j in 1..nb_doigts, k in 1..6} ; 									 
  
## the objective function  
   minimize distance:  1000*(1/nb_doigts)*( sum{k in 1..3} ( sum{j in 1..3} p[j,k])^2  )  ;  
  
# fingertips on the superquadric shaped object  
 
subject to constraint_SQ_position{j in 1..3}: 1-dv <= ((p[j,1]/s1)^(2/epsilon2) + (p[j,2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (p[j,3]/s3)^(2/epsilon1) <= 1+dv ;  
 
# the normal direction: 
subject to contraint_np_1{j in 1..3}: np[j,1] = ((((p[j,1]/s1)^(2/epsilon2)) + ((p[j,2]/s2)^(2/epsilon2)))^(epsilon2/epsilon1 - 1))*((p[j,1]/s1)^(2/epsilon2-1));  
subject to contraint_np_2{j in 1..3}: np[j,2] = ((((p[j,1]/s1)^(2/epsilon2)) + ((p[j,2]/s2)^(2/epsilon2)))^(epsilon2/epsilon1 - 1))*((p[j,2]/s2)^(2/epsilon2-1));  
subject to contraint_np_3{j in 1..3}: np[j,3] = (p[j,3]/s3)^(2/epsilon1-1);  
 
#constraints for the contact frame: 
 
subject to constraint_vp_norm{j in 1..3}: 1-dv<=vp[j,1]^2+vp[j,2]^2+vp[j,3]^2<=1+dv; 
subject to constraint_vp_direction{j in 1..3}: vp[j,1]*np[j,1]+vp[j,2]*np[j,2]+vp[j,3]*np[j,3]>=0; 
subject to constraint_vp_1{j in 1..3}: 0-dv<= vp[j,2]*np[j,3]-vp[j,3]*np[j,2]<=0+dv; 
subject to constraint_vp_2{j in 1..3}: 0-dv<= vp[j,3]*np[j,1]-vp[j,1]*np[j,1]<=0+dv; 
subject to constraint_vp_3{j in 1..3}: 0-dv<= vp[j,1]*np[j,2]-vp[j,2]*np[j,1]<=0+dv; 
 
 
subject to constraint_vt1_1{j in 1..3}: 1-dv<=vt1[j,1]^2+vt1[j,2]^2+vt1[j,3]^2<=1+dv; 
subject to constraint_vt1_2{j in 1..3}: 0-dv<=vt1[j,1]*vp[j,1]+vt1[j,2]*vp[j,2]+vt1[j,3]*vp[j,3]<=0+dv; 
 
subject to constraint_vt2{j in 1..3}: 1-dv<=vt2[j,1]^2+vt2[j,2]^2+vt2[j,3]^2<=1+dv; 
subject to constraint_vt2_2{j in 1..3}: 0-dv<=vt2[j,1]*vp[j,1]+vt2[j,2]*vp[j,2]+vt2[j,3]*vp[j,3]<=0+dv; 
 
subject to constraint_cross_product_1{j in 1..3}: 0-dv<=vt1[j,2]*vt2[j,3]-vt1[j,3]*vt2[j,2]-vp[j,1]<=0+dv; 
subject to constraint_cross_product_2{j in 1..3}: 0-dv<=vt1[j,3]*vt2[j,1]-vt1[j,1]*vt2[j,3]-vp[j,2]<=0+dv; 
subject to constraint_cross_product_3{j in 1..3}: 0-dv<=vt1[j,1]*vt2[j,2]-vt1[j,2]*vt2[j,1]-vp[j,3]<=0+dv; 
 
# the contact primitive: 
# l[i,j,k] i frition segment, j finger, k vector components.  
subject to constraint_contact_primitive_1{i in 1..nb_segments, j in 1..3, k in 1..3}: l[i,j,k] = vt1[j,k]*mu*cos(6.28*i/nb_segments)+vt2[j,k]*mu*sin(6.28*i/nb_segments)+vp[j,k]; 
 
# the contact primitive wrench: 
 
subject to contraint_W_12_1{i in 1..nb_segments, j in 1..3, k in 1..3}: W[i,j,k] = l[i,j,k];  
subject to contraint_W_12_4{i in 1..nb_segments, j in 1..3}: W[i,j,4] = l[i,j,3]*p[j,2]-l[i,j,2]*p[j,3];  
subject to contraint_W_12_5{i in 1..nb_segments, j in 1..3}: W[i,j,5] = -l[i,j,3]*p[j,1]+l[i,j,1]*p[j,3];  
subject to contraint_W_12_6{i in 1..nb_segments, j in 1..3}: W[i,j,6] = l[i,j,2]*p[j,1]-l[i,j,1]*p[j,2];  
 
# Force closure constraint  
 
subject to constraint_fc1:   1-dv <= sum{i in 1..nb_segments} sum{j in 1..nb_doigts} phi[i,j]  <= 1+dv ;  
subject to constraint_fc2{k in 1..6}:   0-dv <= sum{i in 1..nb_segments} sum{j in 1..nb_doigts} phi[i,j]*W[i,j,k]  <= 0+dv ;  
 
#force closure constraints end  
  
  
#initialisation  
  
let {i in 1..nb_segments, j in 1..nb_doigts} phi[i,j] := 0.0417;  
#let {j in 1..3, k in 1..3} p[j,k] := 0.3;  
let p[1,1] := -0.053260 ;
let p[1,2] := -0.045177 ;
let p[1,3] := -0.045722 ;
let p[2,1] := 0.020234 ;
let p[2,2] := -0.003354 ;
let p[2,3] := -0.039514 ;
let p[3,1] := 0.062396 ;
let p[3,2] := 0.045957 ;
let p[3,3] := 0.024000 ;
 
let {j in 1..3} vp[j,1] :=0; 
let {j in 1..3} vp[j,2] :=0; 
let {j in 1..3} vp[j,3] :=1; 
 
let {j in 1..3} vt1[j,1] :=0.7071; 
let {j in 1..3} vt1[j,2] :=-0.7071; 
let {j in 1..3} vt1[j,3] :=0; 
 
let {j in 1..3} vt2[j,1] :=0.7071; 
let {j in 1..3} vt2[j,2] :=0.7071; 
let {j in 1..3} vt2[j,3] :=0;  
 
#Solve the problem   
solve;   
   
# Print the solution  
display solve_message  >>  sol_cylinder_p744.txt ;
print 'Solve time:' >>  sol_cylinder_p744.txt ;
display _solve_time   >>  sol_cylinder_p744.txt ;
print 'Objective:'  >>  sol_cylinder_p744.txt ;
display  p   >>  sol_cylinder_p744.txt ;
display phi  >>  sol_cylinder_p744.txt ;
display l  >>  sol_cylinder_p744.txt ;
display np  >>  sol_cylinder_p744.txt ;
display vp  >>  sol_cylinder_p744.txt ;
  
  
  
