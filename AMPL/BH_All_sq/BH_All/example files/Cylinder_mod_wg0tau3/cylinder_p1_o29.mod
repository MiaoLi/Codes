reset; 
option solver ipopt; 
 
option ipopt_options "max_iter=5000 tol=10e-4 acceptable_tol=10e-2 mu_strategy=adaptive bound_relax_factor=.1 dual_inf_tol=1"; 
 
param pi := 3.14159 ; 
 
param nb_doigts := 3 ; 
 
param dv := 0.02; 
param dv2 := 0.02; 
 
param nb_segments := 4 ; #nb de segments aveclequel on segmente le cone de frottement 
param mu := 0.5 ; #coefficient de frottement 
param wg := 0; #object weight 
param tasktau:=3; #task torque 
 
# Superquadric shape and size:  
param s1 :=   0.035000 ;
param s2 :=   0.035000 ;
param s3 :=    0.065000 ;
param epsilon1 :=    0.250000 ;
param epsilon2 :=  1.000000 ;
 
#the variables for the barrett hand 
var cnh1; 
var cnh2; 
var cnh3; 
var snh1; 
var snh2; 
var snh3; 
var Hx; 
var Hy;  
var Hz; 
var cnf11; 
var cnf21; 
var cnf31; 
var cnf12; 
var cnf22; 
var cnf32; 
var snf11; 
var snf21; 
var snf31; 
var snf12; 
var snf22; 
var snf32; 
var cnt1; 
var cnt2; 
var snt1; 
var snt2; 
 
#the normal vector of the fingertip 
var mf{j in 1..2,k in 1..3}; 
var mt{k in 1..3}; 
 
# The fingertips positions in the object reference frame are given by pij: 
var  p {j in 1..3, k in 1..3}; 
# the key point for the collision detection 
var f1q1{i in 1..3}; 
var f1q2{i in 1..3}; 
var f1q3{i in 1..3}; 
var f2q1{i in 1..3}; 
var f2q2{i in 1..3}; 
var f2q3{i in 1..3}; 
var f3q1{i in 1..3}; 
var f3q2{i in 1..3}; 
 
# The internal normal vector at a point p of the superquadric is given by np: 
var np{j in 1..3, k in 1..3}  ; 
 
var phi{ i in 1..nb_segments, j in 1..nb_doigts} >= 0 ; 
var l{i in 1..nb_segments, j in 1..nb_doigts, k in 1..3} ; 
var W{i in 1..nb_segments, j in 1..nb_doigts, k in 1..6} ; 
 
#joint torque of each finger  
var tau11;  
var tau12 >=0;  
var tau13 >=0;  
var tau21;  
var tau22 >=0;  
var tau23 >=0;  
var tau31 >=0;  
var tau32 >=0;  
 
var f11; 
var f12 ;  # the normal on the fingertip along y-axis; 
var f13; 
var f21; 
var f22; 
var f23 ; 
var f31; 
var f32 ; 
var f33; 
 
#the contact forces constraints 
subject to constraint_force1:mu^2*f12^2-(f11^2+f13^2)>=0-dv2; 
subject to constraint_force2:mu^2*f22^2-(f21^2+f23^2)>=0-dv2; 
subject to constraint_force3:mu^2*f32^2-(f31^2+f33^2)>=0-dv2; 
subject to constraint_force_n1: 0<= f12 <=15; 
subject to constraint_force_n2: 0<= f22 <=15;	  
subject to constraint_force_n3: 0<= f32 <=15; 
#the joint torque condition  
subject to constraint_tau11: tau11=f11*0+f12*0+f13*(0.07*cnf21 - 0.003*snf21 + 0.056*cnf21*cnf31 - 0.0024*cnf21*snf31 - 0.0024*cnf31*snf21 - 0.056*snf21*snf31 + 0.05); 
 
subject to constraint_tau12: tau12=f11*(snf31*(0.056*cnf31 - 0.0024*snf31 + 0.07) - 1.0*cnf31*(0.0024*cnf31 + 0.056*snf31 + 0.003))+(cnf31*(0.056*cnf31 - 0.0024*snf31 + 0.07) + snf31*(0.0024*cnf31 + 0.056*snf31 + 0.003))*f12+f13*0; 
  
subject to constraint_tau13: tau13=f11*(-0.0024)+f12*(0.056)+f13*0; 
 
subject to constraint_tau21: tau21=f21*0+f22*0+f23*(0.003*snf22 - 0.07*cnf22 - 0.056*cnf22*cnf32 + 0.0024*cnf22*snf32 + 0.0024*cnf32*snf22 + 0.056*snf22*snf32 - 0.05); 
  
subject to constraint_tau22: tau22=f21*(snf32*(0.056*cnf32 - 0.0024*snf32 + 0.07) - 1.0*cnf32*(0.0024*cnf32 + 0.056*snf32 + 0.003))+f22*(cnf32*(0.056*cnf32 - 0.0024*snf32 + 0.07) + snf32*(0.0024*cnf32 + 0.056*snf32 + 0.003))+f23*0; 
  
subject to constraint_tau23: tau23=f21*(-0.0024)+f22*(0.056)+f23*(0); 
  
subject to constraint_tau31: tau31=f31*(snt2*(0.056*cnt2 - 0.0024*snt2 + 0.07) - 1.0*cnt2*(0.0024*cnt2 + 0.056*snt2 + 0.003))+f32*(cnt2*(0.056*cnt2 - 0.0024*snt2 + 0.07) + snt2*(0.0024*cnt2 + 0.056*snt2 + 0.003))+f33*(0); 
  
subject to constraint_tau32: tau32=f31*(-0.0024)+f32*(0.056)+f33*(0); 
 
#object force equilibrium, only the object weight is considered here. 
 
subject to constraint_force_obj1:	0-dv2 <= f13*(cnf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnh1*cnh2*snf11) - 1.0*f33*(1.0*cnh3*snh1 - 1.0*cnh1*snh2*snh3) + f11*(snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2))) + f31*(cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1)) - 1.0*f32*(1.0*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 1.0*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1)) + 12.0*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + f23*(cnf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnh1*cnh2*snf12) - 12.0*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + f21*(cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2))) - 1.0*f22*(1.0*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 1.0*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2))) <= 0+0.02; 
 																	 
subject to constraint_force_obj2: 0-dv2 <= f33*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*f32*(1.0*cnt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) - 1.0*snt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1)) - 1.0*f11*(1.0*cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 1.0*snf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2))) - 1.0*f13*(1.0*cnf11*(cnh1*cnh3 + snh1*snh2*snh3) + 1.0*cnh2*snf11*snh1) + 12.0*cnf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 1.0*f31*(1.0*snt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + 1.0*cnt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1)) + 12.0*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + f21*(1.0*cnf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 1.0*snf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1))) - 1.0*f23*(1.0*cnf12*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnh2*snf12*snh1) - 1.0*f22*(1.0*cnf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + snf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2))) <=0+dv2; 
  
subject to constraint_force_obj3: wg-dv2<= 12.0*snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) - 1.0*f11*(1.0*cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) - 1.0*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3)) - 1.0*f23*(1.0*snf12*snh2 + 1.0*cnf12*cnh2*snh3) - 1.0*f32*(1.0*snt2*(cnt1*snh2 + cnh2*cnh3*snt1) + 1.0*cnt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1)) + 12.0*cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + f13*(snf11*snh2 - 1.0*cnf11*cnh2*snh3) - 1.0*f31*(1.0*snt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) - 1.0*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1)) + f22*(snf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + cnf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3)) - 1.0*f21*(1.0*cnf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) - 1.0*snf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3)) + cnh2*f33*snh3 <=wg+dv2; 
  
subject to constraint_force_obj4: 0-dv2 <= f22*((cnf32*(cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + 1.0000e0*snf32*(1.0000e0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0000e0*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)))*(Hz - 7.0000e-3*cnh2*cnh3 - 5.0000e-2*cnf12*snh2 + 2.5000e-2*cnh2*snh3 - 5.6000e-2*cnf32*(cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) - 1.0000e0*cnh2*cnh3*snf22) + 2.4000e-3*snf32*(cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) - 1.0000e0*cnh2*cnh3*snf22) + 2.4000e-3*cnf32*(snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) + 5.6000e-2*snf32*(snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) - 7.0000e-2*cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + 3.0000e-3*snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + 3.0000e-3*cnf22*cnh2*cnh3 + 7.0000e-2*cnh2*cnh3*snf22 + 5.0000e-2*cnh2*snf12*snh3) + (snf32*(cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) - 1.0000e0*cnh2*cnh3*snf22) + cnf32*(snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3))*(Hy + 5.0000e-2*snf12*(cnh1*cnh3 + snh1*snh2*snh3) + 2.5000e-2*cnh1*cnh3 + 7.0000e-3*cnh1*snh3 - 3.0000e-3*cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 2.4000e-3*cnf32*(cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + 5.6000e-2*cnf32*(1.0000e0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0000e0*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) + 7.0000e-2*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 7.0000e-2*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 5.6000e-2*snf32*(cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) - 2.4000e-3*snf32*(1.0000e0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0000e0*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 3.0000e-3*snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) + 5.0000e-2*cnf12*cnh2*snh1 - 7.0000e-3*cnh3*snh1*snh2 + 2.5000e-2*snh1*snh2*snh3)) - 1.0000e0*f13*(1.0000e0*(snf11*snh2 - 1.0000e0*cnf11*cnh2*snh3)*(5.0000e-2*snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*Hy + 2.5000e-2*cnh1*cnh3 - 7.0000e-3*cnh1*snh3 + 5.6000e-2*cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 2.4000e-3*cnf31*(1.0000e0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 1.0000e0*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) + 7.0000e-2*cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 2.4000e-3*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 5.6000e-2*snf31*(1.0000e0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 1.0000e0*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 3.0000e-3*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + 3.0000e-3*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + 7.0000e-2*snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 5.0000e-2*cnf11*cnh2*snh1 + 7.0000e-3*cnh3*snh1*snh2 + 2.5000e-2*snh1*snh2*snh3) - 1.0000e0*(cnf11*(cnh1*cnh3 + snh1*snh2*snh3) + cnh2*snf11*snh1)*(Hz - 7.0000e-2*cnf21*(cnf11*snh2 + cnh2*snf11*snh3) + 3.0000e-3*snf21*(cnf11*snh2 + cnh2*snf11*snh3) - 7.0000e-3*cnh2*cnh3 - 5.0000e-2*cnf11*snh2 - 2.5000e-2*cnh2*snh3 - 5.6000e-2*cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0000e0*cnh2*cnh3*snf21) + 2.4000e-3*snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0000e0*cnh2*cnh3*snf21) + 2.4000e-3*cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 5.6000e-2*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 3.0000e-3*cnf21*cnh2*cnh3 + 7.0000e-2*cnh2*cnh3*snf21 - 5.0000e-2*cnh2*snf11*snh3)) - f23*(1.0000e0*(snf12*snh2 + cnf12*cnh2*snh3)*(Hy + 5.0000e-2*snf12*(cnh1*cnh3 + snh1*snh2*snh3) + 2.5000e-2*cnh1*cnh3 + 7.0000e-3*cnh1*snh3 - 3.0000e-3*cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 2.4000e-3*cnf32*(cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + 5.6000e-2*cnf32*(1.0000e0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0000e0*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) + 7.0000e-2*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 7.0000e-2*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 5.6000e-2*snf32*(cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) - 2.4000e-3*snf32*(1.0000e0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0000e0*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 3.0000e-3*snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) + 5.0000e-2*cnf12*cnh2*snh1 - 7.0000e-3*cnh3*snh1*snh2 + 2.5000e-2*snh1*snh2*snh3) - (cnf12*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnh2*snf12*snh1)*(Hz - 7.0000e-3*cnh2*cnh3 - 5.0000e-2*cnf12*snh2 + 2.5000e-2*cnh2*snh3 - 5.6000e-2*cnf32*(cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) - 1.0000e0*cnh2*cnh3*snf22) + 2.4000e-3*snf32*(cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) - 1.0000e0*cnh2*cnh3*snf22) + 2.4000e-3*cnf32*(snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) + 5.6000e-2*snf32*(snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) - 7.0000e-2*cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + 3.0000e-3*snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + 3.0000e-3*cnf22*cnh2*cnh3 + 7.0000e-2*cnh2*cnh3*snf22 + 5.0000e-2*cnh2*snf12*snh3)) - 1.0000e0*f33*(1.0000e0*(cnh1*cnh3 + snh1*snh2*snh3)*(Hz + 5.0000e-2*snh2 - 2.4000e-3*snt2*(cnt1*snh2 + cnh2*cnh3*snt1) - 7.0000e-3*cnh2*cnh3 + 7.0000e-2*cnt1*snh2 - 3.0000e-3*snh2*snt1 - 2.4000e-3*cnt2*(snh2*snt1 - 1.0000e0*cnh2*cnh3*cnt1) - 5.6000e-2*snt2*(snh2*snt1 - 1.0000e0*cnh2*cnh3*cnt1) + 5.6000e-2*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1) + 3.0000e-3*cnh2*cnh3*cnt1 + 7.0000e-2*cnh2*cnh3*snt1) + 1.0000e0*cnh2*snh3*(5.0000e-2*cnh2*snh1 - 1.0000e0*Hy - 7.0000e-3*cnh1*snh3 + 2.4000e-3*cnt2*(cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 1.0000e0*cnh2*snh1*snt1) + 5.6000e-2*snt2*(cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 1.0000e0*cnh2*snh1*snt1) + 5.6000e-2*cnt2*(snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) - 2.4000e-3*snt2*(snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) + 3.0000e-3*cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + 7.0000e-2*snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + 7.0000e-2*cnh2*cnt1*snh1 + 7.0000e-3*cnh3*snh1*snh2 - 3.0000e-3*cnh2*snh1*snt1)) - 1.2000e1*(cnf31*(1.0000e0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 1.0000e0*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) + 1.0000e0*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)))*(Hz - 7.0000e-2*cnf21*(cnf11*snh2 + cnh2*snf11*snh3) + 3.0000e-3*snf21*(cnf11*snh2 + cnh2*snf11*snh3) - 7.0000e-3*cnh2*cnh3 - 5.0000e-2*cnf11*snh2 - 2.5000e-2*cnh2*snh3 - 5.6000e-2*cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0000e0*cnh2*cnh3*snf21) + 2.4000e-3*snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0000e0*cnh2*cnh3*snf21) + 2.4000e-3*cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 5.6000e-2*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 3.0000e-3*cnf21*cnh2*cnh3 + 7.0000e-2*cnh2*cnh3*snf21 - 5.0000e-2*cnh2*snf11*snh3) - 1.0000e0*f21*(1.0000e0*(cnf32*(cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) - 1.0000e0*cnh2*cnh3*snf22) - 1.0000e0*snf32*(snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3))*(Hy + 5.0000e-2*snf12*(cnh1*cnh3 + snh1*snh2*snh3) + 2.5000e-2*cnh1*cnh3 + 7.0000e-3*cnh1*snh3 - 3.0000e-3*cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 2.4000e-3*cnf32*(cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + 5.6000e-2*cnf32*(1.0000e0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0000e0*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) + 7.0000e-2*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 7.0000e-2*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 5.6000e-2*snf32*(cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) - 2.4000e-3*snf32*(1.0000e0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0000e0*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 3.0000e-3*snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) + 5.0000e-2*cnf12*cnh2*snh1 - 7.0000e-3*cnh3*snh1*snh2 + 2.5000e-2*snh1*snh2*snh3) + (cnf32*(1.0000e0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0000e0*snf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 1.0000e0*snf32*(cnf22*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)))*(Hz - 7.0000e-3*cnh2*cnh3 - 5.0000e-2*cnf12*snh2 + 2.5000e-2*cnh2*snh3 - 5.6000e-2*cnf32*(cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) - 1.0000e0*cnh2*cnh3*snf22) + 2.4000e-3*snf32*(cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) - 1.0000e0*cnh2*cnh3*snf22) + 2.4000e-3*cnf32*(snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) + 5.6000e-2*snf32*(snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) - 7.0000e-2*cnf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + 3.0000e-3*snf22*(cnf12*snh2 - 1.0000e0*cnh2*snf12*snh3) + 3.0000e-3*cnf22*cnh2*cnh3 + 7.0000e-2*cnh2*cnh3*snf22 + 5.0000e-2*cnh2*snf12*snh3)) + f11*((cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 1.0000e0*snf31*(1.0000e0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 1.0000e0*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)))*(Hz - 7.0000e-2*cnf21*(cnf11*snh2 + cnh2*snf11*snh3) + 3.0000e-3*snf21*(cnf11*snh2 + cnh2*snf11*snh3) - 7.0000e-3*cnh2*cnh3 - 5.0000e-2*cnf11*snh2 - 2.5000e-2*cnh2*snh3 - 5.6000e-2*cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0000e0*cnh2*cnh3*snf21) + 2.4000e-3*snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0000e0*cnh2*cnh3*snf21) + 2.4000e-3*cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 5.6000e-2*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 3.0000e-3*cnf21*cnh2*cnh3 + 7.0000e-2*cnh2*cnh3*snf21 - 5.0000e-2*cnh2*snf11*snh3) + (cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0000e0*cnh2*cnh3*snf21) - 1.0000e0*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3))*(5.0000e-2*snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*Hy + 2.5000e-2*cnh1*cnh3 - 7.0000e-3*cnh1*snh3 + 5.6000e-2*cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 2.4000e-3*cnf31*(1.0000e0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 1.0000e0*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) + 7.0000e-2*cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 2.4000e-3*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 5.6000e-2*snf31*(1.0000e0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 1.0000e0*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 3.0000e-3*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + 3.0000e-3*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + 7.0000e-2*snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 5.0000e-2*cnf11*cnh2*snh1 + 7.0000e-3*cnh3*snh1*snh2 + 2.5000e-2*snh1*snh2*snh3)) - 1.2000e1*(snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0000e0*cnh2*cnh3*snf21) + cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3))*(5.0000e-2*snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*Hy + 2.5000e-2*cnh1*cnh3 - 7.0000e-3*cnh1*snh3 + 5.6000e-2*cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 2.4000e-3*cnf31*(1.0000e0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 1.0000e0*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) + 7.0000e-2*cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 2.4000e-3*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 5.6000e-2*snf31*(1.0000e0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) - 1.0000e0*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2)) - 3.0000e-3*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0000e0*cnf11*cnh2*snh1) + 3.0000e-3*cnf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + 7.0000e-2*snf21*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 5.0000e-2*cnf11*cnh2*snh1 + 7.0000e-3*cnh3*snh1*snh2 + 2.5000e-2*snh1*snh2*snh3) + f31*((snt2*(cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 1.0000e0*cnh2*snh1*snt1) + cnt2*(snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + cnh2*cnt1*snh1))*(Hz + 5.0000e-2*snh2 - 2.4000e-3*snt2*(cnt1*snh2 + cnh2*cnh3*snt1) - 7.0000e-3*cnh2*cnh3 + 7.0000e-2*cnt1*snh2 - 3.0000e-3*snh2*snt1 - 2.4000e-3*cnt2*(snh2*snt1 - 1.0000e0*cnh2*cnh3*cnt1) - 5.6000e-2*snt2*(snh2*snt1 - 1.0000e0*cnh2*cnh3*cnt1) + 5.6000e-2*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1) + 3.0000e-3*cnh2*cnh3*cnt1 + 7.0000e-2*cnh2*cnh3*snt1) + (snt2*(snh2*snt1 - 1.0000e0*cnh2*cnh3*cnt1) - 1.0000e0*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1))*(5.0000e-2*cnh2*snh1 - 1.0000e0*Hy - 7.0000e-3*cnh1*snh3 + 2.4000e-3*cnt2*(cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 1.0000e0*cnh2*snh1*snt1) + 5.6000e-2*snt2*(cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 1.0000e0*cnh2*snh1*snt1) + 5.6000e-2*cnt2*(snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) - 2.4000e-3*snt2*(snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) + 3.0000e-3*cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + 7.0000e-2*snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + 7.0000e-2*cnh2*cnt1*snh1 + 7.0000e-3*cnh3*snh1*snh2 - 3.0000e-3*cnh2*snh1*snt1)) + f32*((snt2*(cnt1*snh2 + cnh2*cnh3*snt1) + cnt2*(snh2*snt1 - 1.0000e0*cnh2*cnh3*cnt1))*(5.0000e-2*cnh2*snh1 - 1.0000e0*Hy - 7.0000e-3*cnh1*snh3 + 2.4000e-3*cnt2*(cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 1.0000e0*cnh2*snh1*snt1) + 5.6000e-2*snt2*(cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 1.0000e0*cnh2*snh1*snt1) + 5.6000e-2*cnt2*(snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) - 2.4000e-3*snt2*(snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) + 3.0000e-3*cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + 7.0000e-2*snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + 7.0000e-2*cnh2*cnt1*snh1 + 7.0000e-3*cnh3*snh1*snh2 - 3.0000e-3*cnh2*snh1*snt1) + (cnt2*(cnt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) - 1.0000e0*cnh2*snh1*snt1) - 1.0000e0*snt2*(snt1*(cnh1*snh3 - 1.0000e0*cnh3*snh1*snh2) + cnh2*cnt1*snh1))*(Hz + 5.0000e-2*snh2 - 2.4000e-3*snt2*(cnt1*snh2 + cnh2*cnh3*snt1) - 7.0000e-3*cnh2*cnh3 + 7.0000e-2*cnt1*snh2 - 3.0000e-3*snh2*snt1 - 2.4000e-3*cnt2*(snh2*snt1 - 1.0000e0*cnh2*cnh3*cnt1) - 5.6000e-2*snt2*(snh2*snt1 - 1.0000e0*cnh2*cnh3*cnt1) + 5.6000e-2*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1) + 3.0000e-3*cnh2*cnh3*cnt1 + 7.0000e-2*cnh2*cnh3*snt1)) <=0+dv2; 
  
subject to constraint_force_obj5: 0-dv2<=f11*((snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)))*(Hz - 0.07*cnf21*(cnf11*snh2 + cnh2*snf11*snh3) + 0.003*snf21*(cnf11*snh2 + cnh2*snf11*snh3) - 0.007*cnh2*cnh3 - 0.05*cnf11*snh2 - 0.025*cnh2*snh3 - 0.056*cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + 0.0024*snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + 0.0024*cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 0.056*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 0.003*cnf21*cnh2*cnh3 + 0.07*cnh2*cnh3*snf21 - 0.05*cnh2*snf11*snh3) + (cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) - 1.0*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3))*(Hx + 0.003*cnf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.025*cnh3*snh1 - 0.007*snh1*snh3 + 0.0024*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.07*cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.003*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.0024*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.05*snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf11*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3)) + f21*((cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)))*(Hz - 0.007*cnh2*cnh3 - 0.05*cnf12*snh2 + 0.025*cnh2*snh3 - 0.056*cnf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + 0.0024*snf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + 0.0024*cnf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) + 0.056*snf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) - 0.07*cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*cnf22*cnh2*cnh3 + 0.07*cnh2*cnh3*snf22 + 0.05*cnh2*snf12*snh3) + (cnf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) - 1.0*snf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3))*(Hx + 0.003*cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.056*cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.025*cnh3*snh1 - 0.07*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) - 0.0024*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.007*snh1*snh3 + 0.003*snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) + 0.0024*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + 0.056*snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.05*snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf12*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3)) + 12.0*(1.0*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 1.0*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)))*(Hz - 0.07*cnf21*(cnf11*snh2 + cnh2*snf11*snh3) + 0.003*snf21*(cnf11*snh2 + cnh2*snf11*snh3) - 0.007*cnh2*cnh3 - 0.05*cnf11*snh2 - 0.025*cnh2*snh3 - 0.056*cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + 0.0024*snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + 0.0024*cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 0.056*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 0.003*cnf21*cnh2*cnh3 + 0.07*cnh2*cnh3*snf21 - 0.05*cnh2*snf11*snh3) - 1.0*f13*(1.0*(snf11*snh2 - 1.0*cnf11*cnh2*snh3)*(Hx + 0.003*cnf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.025*cnh3*snh1 - 0.007*snh1*snh3 + 0.0024*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.07*cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.003*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.0024*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.05*snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf11*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3) - 1.0*(cnf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnh1*cnh2*snf11)*(Hz - 0.07*cnf21*(cnf11*snh2 + cnh2*snf11*snh3) + 0.003*snf21*(cnf11*snh2 + cnh2*snf11*snh3) - 0.007*cnh2*cnh3 - 0.05*cnf11*snh2 - 0.025*cnh2*snh3 - 0.056*cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + 0.0024*snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + 0.0024*cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 0.056*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 0.003*cnf21*cnh2*cnh3 + 0.07*cnh2*cnh3*snf21 - 0.05*cnh2*snf11*snh3)) - 1.0*f22*(1.0*(snf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + cnf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3))*(Hx + 0.003*cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.056*cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.025*cnh3*snh1 - 0.07*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) - 0.0024*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.007*snh1*snh3 + 0.003*snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) + 0.0024*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + 0.056*snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.05*snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf12*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3) + 1.0*(snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 1.0*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)))*(Hz - 0.007*cnh2*cnh3 - 0.05*cnf12*snh2 + 0.025*cnh2*snh3 - 0.056*cnf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + 0.0024*snf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + 0.0024*cnf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) + 0.056*snf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) - 0.07*cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*cnf22*cnh2*cnh3 + 0.07*cnh2*cnh3*snf22 + 0.05*cnh2*snf12*snh3)) - 1.0*f33*(1.0*(cnh3*snh1 - 1.0*cnh1*snh2*snh3)*(Hz + 0.05*snh2 - 0.0024*snt2*(cnt1*snh2 + cnh2*cnh3*snt1) - 0.007*cnh2*cnh3 + 0.07*cnt1*snh2 - 0.003*snh2*snt1 - 0.0024*cnt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) - 0.056*snt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) + 0.056*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1) + 0.003*cnh2*cnh3*cnt1 + 0.07*cnh2*cnh3*snt1) + 1.0*cnh2*snh3*(Hx + 0.003*cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 0.05*cnh1*cnh2 - 0.007*snh1*snh3 + 0.056*cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 0.0024*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + 0.0024*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) + 0.056*snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) - 0.07*cnh1*cnh2*cnt1 - 0.007*cnh1*cnh3*snh2 + 0.003*cnh1*cnh2*snt1)) - 12.0*(snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3))*(Hx + 0.003*cnf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.025*cnh3*snh1 - 0.007*snh1*snh3 + 0.0024*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.07*cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.003*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.0024*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.05*snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf11*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3) + f31*((snt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) - 1.0*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1))*(Hx + 0.003*cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 0.05*cnh1*cnh2 - 0.007*snh1*snh3 + 0.056*cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 0.0024*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + 0.0024*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) + 0.056*snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) - 0.07*cnh1*cnh2*cnt1 - 0.007*cnh1*cnh3*snh2 + 0.003*cnh1*cnh2*snt1) + (cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1))*(Hz + 0.05*snh2 - 0.0024*snt2*(cnt1*snh2 + cnh2*cnh3*snt1) - 0.007*cnh2*cnh3 + 0.07*cnt1*snh2 - 0.003*snh2*snt1 - 0.0024*cnt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) - 0.056*snt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) + 0.056*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1) + 0.003*cnh2*cnh3*cnt1 + 0.07*cnh2*cnh3*snt1)) + f32*((snt2*(cnt1*snh2 + cnh2*cnh3*snt1) + cnt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1))*(Hx + 0.003*cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 0.05*cnh1*cnh2 - 0.007*snh1*snh3 + 0.056*cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 0.0024*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + 0.0024*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) + 0.056*snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) - 0.07*cnh1*cnh2*cnt1 - 0.007*cnh1*cnh3*snh2 + 0.003*cnh1*cnh2*snt1) - 1.0*(1.0*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 1.0*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1))*(Hz + 0.05*snh2 - 0.0024*snt2*(cnt1*snh2 + cnh2*cnh3*snt1) - 0.007*cnh2*cnh3 + 0.07*cnt1*snh2 - 0.003*snh2*snt1 - 0.0024*cnt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) - 0.056*snt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) + 0.056*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1) + 0.003*cnh2*cnh3*cnt1 + 0.07*cnh2*cnh3*snt1)) + f23*((snf12*snh2 + cnf12*cnh2*snh3)*(Hx + 0.003*cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.056*cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.025*cnh3*snh1 - 0.07*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) - 0.0024*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.007*snh1*snh3 + 0.003*snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) + 0.0024*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + 0.056*snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.05*snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf12*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3) + (cnf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnh1*cnh2*snf12)*(Hz - 0.007*cnh2*cnh3 - 0.05*cnf12*snh2 + 0.025*cnh2*snh3 - 0.056*cnf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + 0.0024*snf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + 0.0024*cnf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) + 0.056*snf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) - 0.07*cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*cnf22*cnh2*cnh3 + 0.07*cnh2*cnh3*snf22 + 0.05*cnh2*snf12*snh3))<=0+dv2; 
  
subject to constraint_force_obj6: tasktau-dv2 <= f33*((cnh1*cnh3 + snh1*snh2*snh3)*(Hx + 0.003*cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 0.05*cnh1*cnh2 - 0.007*snh1*snh3 + 0.056*cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 0.0024*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + 0.0024*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) + 0.056*snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) - 0.07*cnh1*cnh2*cnt1 - 0.007*cnh1*cnh3*snh2 + 0.003*cnh1*cnh2*snt1) - 1.0*(cnh3*snh1 - 1.0*cnh1*snh2*snh3)*(0.05*cnh2*snh1 - 1.0*Hy - 0.007*cnh1*snh3 + 0.0024*cnt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + 0.056*snt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + 0.056*cnt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) - 0.0024*snt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) + 0.003*cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*cnh2*cnt1*snh1 + 0.007*cnh3*snh1*snh2 - 0.003*cnh2*snh1*snt1)) - 1.0*f32*(1.0*(cnt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) - 1.0*snt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1))*(Hx + 0.003*cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 0.05*cnh1*cnh2 - 0.007*snh1*snh3 + 0.056*cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 0.0024*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + 0.0024*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) + 0.056*snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) - 0.07*cnh1*cnh2*cnt1 - 0.007*cnh1*cnh3*snh2 + 0.003*cnh1*cnh2*snt1) + (1.0*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 1.0*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1))*(0.05*cnh2*snh1 - 1.0*Hy - 0.007*cnh1*snh3 + 0.0024*cnt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + 0.056*snt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + 0.056*cnt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) - 0.0024*snt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) + 0.003*cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*cnh2*cnt1*snh1 + 0.007*cnh3*snh1*snh2 - 0.003*cnh2*snh1*snt1)) - 1.0*f23*(1.0*(cnf12*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnh2*snf12*snh1)*(Hx + 0.003*cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.056*cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.025*cnh3*snh1 - 0.07*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) - 0.0024*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.007*snh1*snh3 + 0.003*snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) + 0.0024*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + 0.056*snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.05*snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf12*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3) + 1.0*(cnf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnh1*cnh2*snf12)*(Hy + 0.05*snf12*(cnh1*cnh3 + snh1*snh2*snh3) + 0.025*cnh1*cnh3 + 0.007*cnh1*snh3 - 0.003*cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.0024*cnf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + 0.056*cnf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.07*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 0.07*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.056*snf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) - 0.0024*snf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.003*snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) + 0.05*cnf12*cnh2*snh1 - 0.007*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3)) - 1.0*f22*(1.0*(cnf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + 1.0*snf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)))*(Hx + 0.003*cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.056*cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.025*cnh3*snh1 - 0.07*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) - 0.0024*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.007*snh1*snh3 + 0.003*snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) + 0.0024*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + 0.056*snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.05*snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf12*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3) - 1.0*(snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 1.0*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)))*(Hy + 0.05*snf12*(cnh1*cnh3 + snh1*snh2*snh3) + 0.025*cnh1*cnh3 + 0.007*cnh1*snh3 - 0.003*cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.0024*cnf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + 0.056*cnf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.07*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 0.07*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.056*snf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) - 0.0024*snf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.003*snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) + 0.05*cnf12*cnh2*snh1 - 0.007*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3)) - f11*(1.0*(cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 1.0*snf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)))*(Hx + 0.003*cnf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.025*cnh3*snh1 - 0.007*snh1*snh3 + 0.0024*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.07*cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.003*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.0024*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.05*snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf11*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3) - (snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)))*(0.05*snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*Hy + 0.025*cnh1*cnh3 - 0.007*cnh1*snh3 + 0.056*cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.0024*cnf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.07*cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 0.0024*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.056*snf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.003*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + 0.003*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.05*cnf11*cnh2*snh1 + 0.007*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3)) + 12.0*(cnf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 1.0*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)))*(Hx + 0.003*cnf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.025*cnh3*snh1 - 0.007*snh1*snh3 + 0.0024*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.07*cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.003*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.0024*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.05*snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf11*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3) + 12.0*(1.0*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 1.0*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)))*(0.05*snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*Hy + 0.025*cnh1*cnh3 - 0.007*cnh1*snh3 + 0.056*cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.0024*cnf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.07*cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 0.0024*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.056*snf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.003*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + 0.003*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.05*cnf11*cnh2*snh1 + 0.007*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3) + f31*((cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1))*(0.05*cnh2*snh1 - 1.0*Hy - 0.007*cnh1*snh3 + 0.0024*cnt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + 0.056*snt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + 0.056*cnt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) - 0.0024*snt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) + 0.003*cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*cnh2*cnt1*snh1 + 0.007*cnh3*snh1*snh2 - 0.003*cnh2*snh1*snt1) - 1.0*(snt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + cnt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1))*(Hx + 0.003*cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 0.05*cnh1*cnh2 - 0.007*snh1*snh3 + 0.056*cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 0.0024*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + 0.0024*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) + 0.056*snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) - 0.07*cnh1*cnh2*cnt1 - 0.007*cnh1*cnh3*snh2 + 0.003*cnh1*cnh2*snt1)) - 1.0*f13*(1.0*(cnf11*(cnh1*cnh3 + snh1*snh2*snh3) + cnh2*snf11*snh1)*(Hx + 0.003*cnf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.025*cnh3*snh1 - 0.007*snh1*snh3 + 0.0024*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.07*cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.003*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.0024*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.05*snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf11*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3) - 1.0*(cnf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnh1*cnh2*snf11)*(0.05*snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*Hy + 0.025*cnh1*cnh3 - 0.007*cnh1*snh3 + 0.056*cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.0024*cnf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.07*cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 0.0024*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.056*snf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 1.0*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.003*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + 0.003*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.05*cnf11*cnh2*snh1 + 0.007*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3)) - f21*(1.0*(cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)))*(Hy + 0.05*snf12*(cnh1*cnh3 + snh1*snh2*snh3) + 0.025*cnh1*cnh3 + 0.007*cnh1*snh3 - 0.003*cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.0024*cnf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + 0.056*cnf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.07*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 0.07*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.056*snf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) - 0.0024*snf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.003*snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) + 0.05*cnf12*cnh2*snh1 - 0.007*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3) - 1.0*(cnf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 1.0*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 1.0*snf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)))*(Hx + 0.003*cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.056*cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.025*cnh3*snh1 - 0.07*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) - 0.0024*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.007*snh1*snh3 + 0.003*snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) + 0.0024*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + 0.056*snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.05*snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf12*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3)) <=tasktau+dv2; 
  
 
## the objective function 
##minimize distance:  1000*(1/nb_doigts)*( sum{k in 1..3} ( sum{j in 1..3} p[j,k])^2  )  ; 
 
minimize torque: tau11+tau12+tau13+tau21+tau22+tau23+tau31+tau32;  
 
# fingertips calculated in the object reference frame 
subject to contraint_p11: p[1,1] = Hx + 0.003*cnf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.025*cnh3*snh1 - 0.007*snh1*snh3 + 0.0024*cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.07*cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.003*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.056*cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 0.0024*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + 0.05*snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf11*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3; 
  
subject to contraint_p12: p[1,2] = Hy - 0.05*snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 0.025*cnh1*cnh3 + 0.007*cnh1*snh3 - 0.056*cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.07*cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + 0.0024*snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.003*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 0.003*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.0024*cnf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 0.07*snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.056*snf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.05*cnf11*cnh2*snh1 - 0.007*cnh3*snh1*snh2 - 0.025*snh1*snh2*snh3; 
  
subject to contraint_p13: p[1,3] = Hz - 0.07*cnf21*(cnf11*snh2 + cnh2*snf11*snh3) + 0.003*snf21*(cnf11*snh2 + cnh2*snf11*snh3) - 0.007*cnh2*cnh3 - 0.05*cnf11*snh2 - 0.025*cnh2*snh3 - 0.056*cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + 0.0024*snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + 0.0024*cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 0.056*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + 0.003*cnf21*cnh2*cnh3 + 0.07*cnh2*cnh3*snf21 - 0.05*cnh2*snf11*snh3; 
  
subject to contraint_p21: p[2,1] =  Hx + 0.003*cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.056*cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.025*cnh3*snh1 - 0.07*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) - 0.0024*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.007*snh1*snh3 + 0.003*snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) + 0.0024*cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + 0.056*snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 0.05*snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf12*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3; 
 
subject to contraint_p22: p[2,2] = Hy + 0.056*cnf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.05*snf12*(cnh1*cnh3 + snh1*snh2*snh3) - 0.0024*snf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 0.025*cnh1*cnh3 + 0.007*cnh1*snh3 - 0.003*cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.0024*cnf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) + 0.07*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 0.07*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.056*snf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) - 0.003*snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) + 0.05*cnf12*cnh2*snh1 - 0.007*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3; 
 
subject to contraint_p23: p[2,3] = Hz - 0.007*cnh2*cnh3 - 0.05*cnf12*snh2 + 0.025*cnh2*snh3 - 0.056*cnf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + 0.0024*snf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + 0.0024*cnf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) + 0.056*snf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) - 0.07*cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*cnf22*cnh2*cnh3 + 0.07*cnh2*cnh3*snf22 + 0.05*cnh2*snf12*snh3; 
  
subject to contraint_p31: p[3,1] =  Hx + 0.003*cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 0.05*cnh1*cnh2 - 0.007*snh1*snh3 + 0.056*cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 0.0024*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + 0.0024*cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) + 0.056*snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) - 0.07*cnh1*cnh2*cnt1 - 0.007*cnh1*cnh3*snh2 + 0.003*cnh1*cnh2*snt1; 
 
subject to contraint_p32: p[3,2] =  Hy - 0.05*cnh2*snh1 + 0.007*cnh1*snh3 - 0.0024*cnt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) - 0.056*snt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) - 0.056*cnt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) + 0.0024*snt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) - 0.003*cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.07*snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.07*cnh2*cnt1*snh1 - 0.007*cnh3*snh1*snh2 + 0.003*cnh2*snh1*snt1; 
 
subject to contraint_p33: p[3,3] =Hz + 0.05*snh2 - 0.0024*snt2*(cnt1*snh2 + cnh2*cnh3*snt1) - 0.007*cnh2*cnh3 + 0.07*cnt1*snh2 - 0.003*snh2*snt1 - 0.0024*cnt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) - 0.056*snt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) + 0.056*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1) + 0.003*cnh2*cnh3*cnt1 + 0.07*cnh2*cnh3*snt1;  
 
# fingertips on the superquadric shaped object 
 
subject to constraint_SQ_position{j in 1..3}: 1-dv <= ((p[j,1]/s1)^(2/epsilon2) + (p[j,2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (p[j,3]/s3)^(2/epsilon1) <= 1+dv ; 
 
subject to contraint_np_1{j in 1..3}: np[j,1] = -((((p[j,1]/s1)^(2/epsilon2)) + ((p[j,2]/s2)^(2/epsilon2)))^(epsilon2/epsilon1 - 1))*((p[j,1]/s1)^(2/epsilon2-1)); 
subject to contraint_np_2{j in 1..3}: np[j,2] = -((((p[j,1]/s1)^(2/epsilon2)) + ((p[j,2]/s2)^(2/epsilon2)))^(epsilon2/epsilon1 - 1))*((p[j,2]/s2)^(2/epsilon2-1)); 
subject to contraint_np_3{j in 1..3}: np[j,3] = -(p[j,3]/s3)^(2/epsilon1-1); 
# Force closure constraint 
#l[ijk] i frition segment, j finger, k vector components. 
subject to contraint_lf1k1{ith in 1..nb_segments}: l[ith,1,1] =  cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 1.0*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + mu*cos((6.28*ith)/nb_segments)*(cnf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnh1*cnh2*snf11) + mu*cos((6.28*ith)/nb_segments)*(snf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) + cnf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2))); 
 
subject to contraint_lf1k2{ith in 1..nb_segments}: l[ith,1,2] =  snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 1.0*cnf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 1.0*mu*cos((6.28*ith)/nb_segments)*(cnf11*(cnh1*cnh3 + snh1*snh2*snh3) + cnh2*snf11*snh1) - 1.0*mu*cos((6.28*ith)/nb_segments)*(cnf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - snf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2))); 
  
subject to contraint_lf1k3{ith in 1..nb_segments}: l[ith,1,3] = snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3) + mu*cos((6.28*ith)/nb_segments)*(snf11*snh2 - 1.0*cnf11*cnh2*snh3) - 1.0*mu*cos((6.28*ith)/nb_segments)*(cnf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) - 1.0*snf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3)); 
  
subject to contraint_lf2k1{ith in 1..nb_segments}: l[ith,2,1] =  cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 1.0*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + mu*cos((6.28*ith)/nb_segments)*(cnf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) + snf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2))) + mu*cos((6.28*ith)/nb_segments)*(cnf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnh1*cnh2*snf12); 
  
subject to contraint_lf2k2{ith in 1..nb_segments}: l[ith,2,2] =  1.0*mu*cos((6.28*ith)/nb_segments)*(cnf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - snf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1))) - 1.0*cnf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)) - snf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 1.0*mu*cos((6.28*ith)/nb_segments)*(cnf12*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnh2*snf12*snh1); 
  
subject to contraint_lf2k3{ith in 1..nb_segments}: l[ith,2,3] = snf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + cnf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3) - 1.0*mu*cos((6.28*ith)/nb_segments)*(cnf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) - 1.0*snf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3)) - 1.0*mu*cos((6.28*ith)/nb_segments)*(snf12*snh2 + cnf12*cnh2*snh3); 
  
subject to contraint_ltk1{ith in 1..nb_segments}: l[ith,3,1] =  cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) - 1.0*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) - 1.0*mu*cos((6.28*ith)/nb_segments)*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + mu*cos((6.28*ith)/nb_segments)*(cnt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1) + snt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1)); 
  
subject to contraint_ltk2{ith in 1..nb_segments}: l[ith,3,2] = snt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) - 1.0*cnt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + mu*cos((6.28*ith)/nb_segments)*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*mu*cos((6.28*ith)/nb_segments)*(snt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1) + cnt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1)); 
  
subject to contraint_ltk3{ith in 1..nb_segments}: l[ith,3,3] = cnh2*mu*snh3*cos((6.28*ith)/nb_segments) - 1.0*cnt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) - 1.0*mu*cos((6.28*ith)/nb_segments)*(snt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1) - 1.0*cnt2*(cnt1*snh2 + cnh2*cnh3*snt1)) - 1.0*snt2*(cnt1*snh2 + cnh2*cnh3*snt1); 
 
subject to contraint_W_12_1{i in 1..nb_segments, j in 1..3, k in 1..3}: W[i,j,k] = l[i,j,k]; 
subject to contraint_W_12_4{i in 1..nb_segments, j in 1..3}: W[i,j,4] = l[i,j,3]*p[j,2]-l[i,j,2]*p[j,3]; 
subject to contraint_W_12_5{i in 1..nb_segments, j in 1..3}: W[i,j,5] = -l[i,j,3]*p[j,1]+l[i,j,1]*p[j,3]; 
subject to contraint_W_12_6{i in 1..nb_segments, j in 1..3}: W[i,j,6] = l[i,j,2]*p[j,1]-l[i,j,1]*p[j,2]; 
 
subject to constraint_fc1:   1-dv <= sum{i in 1..nb_segments} sum{j in 1..nb_doigts} phi[i,j]  <= 1+dv ; 
subject to constraint_fc2{k in 1..6}:   0-dv <= sum{i in 1..nb_segments} sum{j in 1..nb_doigts} phi[i,j]*W[i,j,k]  <= 0+dv ; 
#force closure constraints end 
 
 
 
# Normal vector constraints 
subject to constraint_mf11: mf[1,1]=cnf31*(cnf21*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)) - 1.0*snf31*(snf21*(snh1*snh3 + cnh1*cnh3*snh2) + cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2)); 
   
subject to constraint_mf12: mf[1,2]=  snf31*(cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) + 1.0*cnf31*(1.0*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)); 
  
subject to constraint_mf13: mf[1,3]=   snf31*(cnf21*(cnf11*snh2 + cnh2*snf11*snh3) - 1.0*cnh2*cnh3*snf21) + cnf31*(snf21*(cnf11*snh2 + cnh2*snf11*snh3) + cnf21*cnh2*cnh3); 
 
subject to constraint_mf21: mf[2,1]=  cnf32*(cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)) - 1.0*snf32*(snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2)); 
  
subject to constraint_mf22: mf[2,2]= - snf32*(1.0*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2)) - 1.0*cnf32*(cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1)); 
  
subject to constraint_mf23: mf[2,3]= snf32*(cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) - 1.0*cnh2*cnh3*snf22) + cnf32*(snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + cnf22*cnh2*cnh3); 
  
subject to constraint_mt1: mt[1]=  cnt2*(cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + cnh1*cnh2*snt1) - 1.0*snt2*(snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 1.0*cnh1*cnh2*cnt1); 
  
subject to constraint_mt2: mt[2]= snt2*(snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + cnh2*cnt1*snh1) - 1.0*cnt2*(cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 1.0*cnh2*snh1*snt1); 
  
subject to constraint_mt3: mt[3]= - 1.0*snt2*(cnt1*snh2 + cnh2*cnh3*snt1) - 1.0*cnt2*(snh2*snt1 - 1.0*cnh2*cnh3*cnt1); 
 
subject to constraint_normal1{j in 1..2}:  np[j,1]*mf[j,1] + np[j,2]*mf[j,2] + np[j,3]*mf[j,3] >= 0 ; 
 
subject to constraint_normal1b:  np[3,1]*mt[1] + np[3,2]*mt[2] + np[3,3]*mt[3] >= 0 ; 
 
subject to constraint_normal2: 0-dv <= np[3,2]*mt[3] - np[3,3]*mt[2] <= 0+dv2; 
 
subject to constraint_normal2b{j in 1..2}:0-dv <= np[j,2]*mf[j,3] - np[j,3]*mf[j,2] <= 0+dv2 ; 
 
subject to constraint_normal3: 0-dv <= - np[3,1]*mt[3] + np[3,3]*mt[1] <= 0+dv2; 
 
subject to constraint_normal3b{j in 1..2}: 0-dv <= - np[j,1]*mf[j,3] + np[j,3]*mf[j,1] <= 0+dv2 ; 
 
subject to constraint_normal4: 0-dv <= np[3,1]*mt[2] - np[3,2]*mt[1] <= 0+dv2; 
 
subject to constraint_normal4b{j in 1..2}: 0-dv <= np[j,1]*mf[j,2] - np[j,2]*mf[j,1] <= 0+dv2 ; 
 
#normal constraints end 
 
 
# Constraints/definition of joint angles (cos and sin) 
 
subject to constraint_cnh1: 1-dv <=cnh1*cnh1+snh1*snh1 <= 1+dv; 
subject to constraint_cnh2: 1-dv <=cnh2*cnh2+snh2*snh2 <= 1+dv; 
subject to constraint_cnh3: 1-dv <=cnh3*cnh3+snh3*snh3 <= 1+dv; 
 
subject to constraint_cnf11: 1-dv <=cnf11*cnf11+snf11*snf11 <= 1+dv; 
subject to constraint_cnf21: 1-dv <=cnf21*cnf21+snf21*snf21 <= 1+dv; 
subject to constraint_cnf31: 1-dv <=cnf31*cnf31+snf31*snf31 <= 1+dv; 
subject to constraint_cnf12: 1-dv <=cnf12*cnf12+snf12*snf12 <= 1+dv; 
subject to constraint_cnf22: 1-dv <=cnf22*cnf22+snf22*snf22 <= 1+dv; 
subject to constraint_cnf32: 1-dv <=cnf32*cnf32+snf32*snf32 <= 1+dv; 
subject to constraint_cnt1: 1-dv <=cnt1*cnt1+snt1*snt1 <= 1+dv; 
subject to constraint_cnt2: 1-dv <=cnt2*cnt2+snt2*snt2 <= 1+dv; 
 
 
#    0<theta<180 
subject to constraint_cnf11_2:  -1 <= cnf11 <= 1; 
subject to constraint_snf11_2:  0 <= snf11 <= 1; 
 
#  0 < theta < 140 
subject to constraint_cnf21_2:  -0.766 <= cnf21 <= 1; 
subject to constraint_snf21_2:  0 <= snf21 <= 1; 
 
#  42 < theta < 90 
subject to constraint_cnf31_2:  0 <= cnf31 <= 0.7431; 
subject to constraint_snf31_2:  0.6691 <= snf31 <= 1; 
 
# theta3 = theta2/3 
subject to constraint_coupling_f1_23_1: cnf21 = -3*(cnf31*0.7431 + snf31*0.6691) + 4*((cnf31*0.7431 + snf31*0.6691)^3); 
subject to constraint_coupling_f1_23_2: snf21 = 3*(snf31*0.7431 - cnf31*0.6691) - 4*((snf31*0.7431 - cnf31*0.6691)^3); 
# symetric constraint 
subject to constraint_symetric_1: cnf11=cnf12; 
subject to constraint_symetric_2: snf11=snf12; 
#  0 < theta < 180 
subject to constraint_cnf12_2:  -1 <= cnf12 <= 1; 
subject to constraint_snf12_2:  0 <= snf12 <= 1; 
 
#  0 < theta < 140 
 
subject to constraint_cnf22_2:  -0.766 <= cnf22 <= 1; 
subject to constraint_snf22_2:  0 <= snf22 <= 1; 
 
#  42 < theta < 90 
subject to constraint_cnf32_2:  0 <= cnf32 <= 0.7431; 
subject to constraint_snf32_2:  0.6691 <= snf32 <= 1; 
 
# theta3 = theta2/3 
subject to constraint_coupling_f2_23_1: cnf22  = -3*(cnf32*0.7431 + snf32*0.6691) + 4*((cnf32*0.7431 + snf32*0.6691)^3); 
subject to constraint_coupling_f2_23_2: snf22  = 3*(snf32*0.7431 - cnf32*0.6691) - 4*((snf32*0.7431 - cnf32*0.6691)^3); 
 
# 40 < theta < 180 
subject to constraint_cnt1_2:  -1 <= cnt1 <= 0.766; 
subject to constraint_snt1_2:  0 <= snt1 <= 1; 
 
# 42 < theta < 90 
subject to constraint_cnt2_2:  0 <= cnt2 <= 0.7431; 
subject to constraint_snt2_2:  0.6691 <= snt2 <= 1; 
 
# theta2 = theta1/3 
subject to constraint_coupling_t1_1: cnt1  = -3*(cnt2*0.7431 + snt2*0.6691) + 4*((cnt2*0.7431 + snt2*0.6691)^3); 
subject to constraint_coupling_t1_2: snt1  = 3*(snt2*0.7431 - cnt2*0.6691) - 4*((snt2*0.7431 - cnt2*0.6691)^3); 
 
# collision avoidance 
 
subject to constraint_f1q1_1: f1q1[1] = Hx + 0.025*cnh3*snh1 - 0.041*snh1*snh3 - 0.041*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3; 
subject to constraint_f1q1_2: f1q1[2] = Hy - 0.025*cnh1*cnh3 + 0.041*cnh1*snh3 - 0.041*cnh3*snh1*snh2 - 0.025*snh1*snh2*snh3; 
subject to constraint_f1q1_3: f1q1[3] = Hz - 0.041*cnh2*cnh3 - 0.025*cnh2*snh3; 
 
subject to constraint_f1q2_1: f1q2[1]= Hx + 0.025*cnh3*snh1 - 0.007*snh1*snh3 + 0.05*snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf11*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3; 
subject to constraint_f1q2_2: f1q2[2]=Hy - 0.05*snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 0.025*cnh1*cnh3 + 0.007*cnh1*snh3 + 0.05*cnf11*cnh2*snh1 - 0.007*cnh3*snh1*snh2 - 0.025*snh1*snh2*snh3; 
subject to constraint_f1q2_3: f1q2[3]=Hz - 0.007*cnh2*cnh3 - 0.05*cnf11*snh2 - 0.025*cnh2*snh3 - 0.05*cnh2*snf11*snh3; 
 
subject to constraint_f1q3_1: f1q3[1]= Hx + 0.003*cnf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf21*(snh1*snh3 + cnh1*cnh3*snh2) + 0.025*cnh3*snh1 - 0.007*snh1*snh3 + 0.07*cnf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) - 0.003*snf21*(snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + cnf11*cnh1*cnh2) + 0.05*snf11*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf11*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 - 0.025*cnh1*snh2*snh3; 
subject to constraint_f1q3_2: f1q3[2]=Hy - 0.05*snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 0.025*cnh1*cnh3 + 0.007*cnh1*snh3 - 0.07*cnf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) + 0.003*snf21*(snf11*(cnh1*cnh3 + snh1*snh2*snh3) - 1.0*cnf11*cnh2*snh1) - 0.003*cnf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.07*snf21*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.05*cnf11*cnh2*snh1 - 0.007*cnh3*snh1*snh2 - 0.025*snh1*snh2*snh3; 
subject to constraint_f1q3_3: f1q3[3]=Hz - 0.07*cnf21*(cnf11*snh2 + cnh2*snf11*snh3) + 0.003*snf21*(cnf11*snh2 + cnh2*snf11*snh3) - 0.007*cnh2*cnh3 - 0.05*cnf11*snh2 - 0.025*cnh2*snh3 + 0.003*cnf21*cnh2*cnh3 + 0.07*cnh2*cnh3*snf21 - 0.05*cnh2*snf11*snh3; 
 
subject to constraint_f2q1_1: f2q1[1]= Hx - 0.025*cnh3*snh1 - 0.041*snh1*snh3 - 0.041*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3; 
subject to constraint_f2q1_2: f2q1[2]= Hy + 0.025*cnh1*cnh3 + 0.041*cnh1*snh3 - 0.041*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3; 
subject to constraint_f2q1_3: f2q1[3]= Hz - 0.041*cnh2*cnh3 + 0.025*cnh2*snh3;			 
  
subject to constraint_f2q2_1: f2q2[1]= Hx - 0.025*cnh3*snh1 - 0.007*snh1*snh3 - 0.05*snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf12*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3; 
subject to constraint_f2q2_2: f2q2[2]= Hy + 0.05*snf12*(cnh1*cnh3 + snh1*snh2*snh3) + 0.025*cnh1*cnh3 + 0.007*cnh1*snh3 + 0.05*cnf12*cnh2*snh1 - 0.007*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3; 
subject to constraint_f2q2_3: f2q2[3]= Hz - 0.007*cnh2*cnh3 - 0.05*cnf12*snh2 + 0.025*cnh2*snh3 + 0.05*cnh2*snf12*snh3; 
 
subject to constraint_f2q3_1: f2q3[1]= Hx + 0.003*cnf22*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snf22*(snh1*snh3 + cnh1*cnh3*snh2) - 0.025*cnh3*snh1 - 0.07*cnf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) - 0.007*snh1*snh3 + 0.003*snf22*(snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) - 1.0*cnf12*cnh1*cnh2) - 0.05*snf12*(cnh3*snh1 - 1.0*cnh1*snh2*snh3) + 0.05*cnf12*cnh1*cnh2 - 0.007*cnh1*cnh3*snh2 + 0.025*cnh1*snh2*snh3; 
subject to constraint_f2q3_2: f2q3[2]=Hy + 0.05*snf12*(cnh1*cnh3 + snh1*snh2*snh3) + 0.025*cnh1*cnh3 + 0.007*cnh1*snh3 - 0.003*cnf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) + 0.07*cnf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) - 0.07*snf22*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.003*snf22*(snf12*(cnh1*cnh3 + snh1*snh2*snh3) + cnf12*cnh2*snh1) + 0.05*cnf12*cnh2*snh1 - 0.007*cnh3*snh1*snh2 + 0.025*snh1*snh2*snh3; 
subject to constraint_f2q3_3: f2q3[3]= Hz - 0.007*cnh2*cnh3 - 0.05*cnf12*snh2 + 0.025*cnh2*snh3 - 0.07*cnf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*snf22*(cnf12*snh2 - 1.0*cnh2*snf12*snh3) + 0.003*cnf22*cnh2*cnh3 + 0.07*cnh2*cnh3*snf22 + 0.05*cnh2*snf12*snh3; 
 
subject to constraint_f3q1_1: f3q1[1]= Hx - 0.05*cnh1*cnh2 - 0.007*snh1*snh3 - 0.007*cnh1*cnh3*snh2; 
subject to constraint_f3q1_2: f3q1[2]= Hy - 0.05*cnh2*snh1 + 0.007*cnh1*snh3 - 0.007*cnh3*snh1*snh2;  
subject to constraint_f3q1_3: f3q1[3]= Hz + 0.05*snh2 - 0.007*cnh2*cnh3; 
 
subject to constraint_f3q2_1: f3q2[1]= Hx + 0.003*cnt1*(snh1*snh3 + cnh1*cnh3*snh2) + 0.07*snt1*(snh1*snh3 + cnh1*cnh3*snh2) - 0.05*cnh1*cnh2 - 0.007*snh1*snh3 - 0.07*cnh1*cnh2*cnt1 - 0.007*cnh1*cnh3*snh2 + 0.003*cnh1*cnh2*snt1; 
subject to constraint_f3q2_2: f3q2[2]=Hy - 0.05*cnh2*snh1 + 0.007*cnh1*snh3 - 0.003*cnt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.07*snt1*(cnh1*snh3 - 1.0*cnh3*snh1*snh2) - 0.07*cnh2*cnt1*snh1 - 0.007*cnh3*snh1*snh2 + 0.003*cnh2*snh1*snt1; 
subject to constraint_f3q2_3: f3q2[3]= Hz + 0.05*snh2 - 0.007*cnh2*cnh3 + 0.07*cnt1*snh2 - 0.003*snh2*snt1 + 0.003*cnh2*cnh3*cnt1 + 0.07*cnh2*cnh3*snt1;  
 
subject to constraint_qhand: ((	Hx/s1)^(2/epsilon2) + (Hy/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (Hz/s3)^(2/epsilon1) >=1; 
 
subject to constraint_f1q1: ((f1q1[1]/s1)^(2/epsilon2) + (f1q1[2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (f1q1[3]/s3)^(2/epsilon1) >= 1 ; 
 
subject to constraint_f1q2: ((f1q2[1]/s1)^(2/epsilon2) + (f1q2[2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (f1q2[3]/s3)^(2/epsilon1) >= 1 ; 
 
subject to constraint_f1q3: ((f1q3[1]/s1)^(2/epsilon2) + (f1q3[2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (f1q3[3]/s3)^(2/epsilon1) >= 1 ; 
 
subject to constraint_f2q1: ((f2q1[1]/s1)^(2/epsilon2) + (f2q1[2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (f2q1[3]/s3)^(2/epsilon1) >= 1 ; 
subject to constraint_f2q2: ((f2q2[1]/s1)^(2/epsilon2) + (f2q2[2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (f2q2[3]/s3)^(2/epsilon1) >= 1 ; 
subject to constraint_f2q3: ((f2q3[1]/s1)^(2/epsilon2) + (f2q3[2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (f2q3[3]/s3)^(2/epsilon1) >= 1 ; 
 
subject to constraint_f3q1: ((f3q1[1]/s1)^(2/epsilon2) + (f3q1[2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (f3q1[3]/s3)^(2/epsilon1) >= 1 ; 
subject to constraint_f3q2: ((f3q2[1]/s1)^(2/epsilon2) + (f3q2[2]/s2)^(2/epsilon2))^(epsilon2/epsilon1) + (f3q2[3]/s3)^(2/epsilon1) >= 1 ; 
 
#collision avoidance end 
 
#initialisation 
 
let cnh1 := -0.500000 ;
let cnh2 := -0.500000 ;
let cnh3 := 0.309017 ;
let snh1 := -0.866025 ;
let snh2 := 0.866025 ;
let snh3 := 0.951057 ;
let Hx := 0.105000 ;
let Hy := 0.105000 ;
let Hz := 0.105000 ;
 
let cnf11 := 0.7071; 
let cnf21 := 0.7071; 
let cnf31 := 0.7071; 
let snf11 := 0.7071; 
let snf21 := 0.7071; 
let snf31 := 0.7071; 
 
let cnf12 := 1; 
let cnf22 := 1; 
let cnf32 := 1; 
let snf12 := 0; 
let snf22 := 0; 
let snf32 := 0; 
 
let cnt1 := 1; 
let cnt2 := 1; 
let snt1 := 0; 
let snt2 := 0; 
 
let {i in 1..nb_segments, j in 1..nb_doigts} phi[i,j] := 0.0417; 
 
# Solve the problem  
solve;  
  
# Print the solution 
display solve_message  >>  sol_cylinder_p1_o29.txt ;
print 'Solve time:' >>  sol_cylinder_p1_o29.txt ;
display _solve_time  >>  sol_cylinder_p1_o29.txt ;
print 'Objective:' >>  sol_cylinder_p1_o29.txt ;
display torque  >>  sol_cylinder_p1_o29.txt ;
display cnh1  >>  sol_cylinder_p1_o29.txt ;
display cnh2 >>  sol_cylinder_p1_o29.txt ;
display cnh3 >>  sol_cylinder_p1_o29.txt ;
display snh1 >>  sol_cylinder_p1_o29.txt ;
display snh2 >>  sol_cylinder_p1_o29.txt ;
display snh3  >>  sol_cylinder_p1_o29.txt ;
display Hx  >>  sol_cylinder_p1_o29.txt ;
display Hy  >>  sol_cylinder_p1_o29.txt ;
display Hz   >>  sol_cylinder_p1_o29.txt ;
display cnf11 >>  sol_cylinder_p1_o29.txt ;
display cnf21 >>  sol_cylinder_p1_o29.txt ;
display cnf31  >>  sol_cylinder_p1_o29.txt ;
display cnf12 >>  sol_cylinder_p1_o29.txt ;
display cnf22  >>  sol_cylinder_p1_o29.txt ;
display cnf32  >>  sol_cylinder_p1_o29.txt ;
display cnt1 >>  sol_cylinder_p1_o29.txt ;
display cnt2 >>  sol_cylinder_p1_o29.txt ;
display snf11  >>  sol_cylinder_p1_o29.txt ;
display snf21  >>  sol_cylinder_p1_o29.txt ;
display snf31 >>  sol_cylinder_p1_o29.txt ;
display snf12 >>  sol_cylinder_p1_o29.txt ;
display snf22 >>  sol_cylinder_p1_o29.txt ;
display snf32  >>  sol_cylinder_p1_o29.txt ;
display snt1 >>  sol_cylinder_p1_o29.txt ;
display snt2  >>  sol_cylinder_p1_o29.txt ;
display  p  >>  sol_cylinder_p1_o29.txt ;
display mf >>  sol_cylinder_p1_o29.txt ;
display mt  >>  sol_cylinder_p1_o29.txt ;
display phi  >>  sol_cylinder_p1_o29.txt ;
display l >>  sol_cylinder_p1_o29.txt ;
display {i in 1..nb_segments, j in 1..1, k in 1..3} W[i,j,k]  >>  sol_cylinder_p1_o29.txt ;
display {i in 1..nb_segments, j in 1..1, k in 4..6} W[i,j,k]   >>  sol_cylinder_p1_o29.txt ;
display {i in 1..nb_segments, j in 2..2, k in 1..3} W[i,j,k]   >>  sol_cylinder_p1_o29.txt ;
display {i in 1..nb_segments, j in 2..2, k in 4..6} W[i,j,k]   >>  sol_cylinder_p1_o29.txt ;
display {i in 1..nb_segments, j in 3..3, k in 1..3} W[i,j,k]   >>  sol_cylinder_p1_o29.txt ;
display {i in 1..nb_segments, j in 3..3, k in 4..6} W[i,j,k]   >>  sol_cylinder_p1_o29.txt ;
display f11  >>  sol_cylinder_p1_o29.txt ;
display f12 >>  sol_cylinder_p1_o29.txt ;
display f13  >>  sol_cylinder_p1_o29.txt ;
display f21  >>  sol_cylinder_p1_o29.txt ;
display f22 >>  sol_cylinder_p1_o29.txt ;
display f23  >>  sol_cylinder_p1_o29.txt ;
display f31 >>  sol_cylinder_p1_o29.txt ;
display f32  >>  sol_cylinder_p1_o29.txt ;
display f33  >>  sol_cylinder_p1_o29.txt ;
 
 
 
 
