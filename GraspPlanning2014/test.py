#!/usr/bin/python

"""
We want to compare several different methods for grasp planing:
--- two-step grasp planning: (1) generate contact points (2) find hand configuration
--- one-shot grasp planning: solve the problem in one shot
"""

"""
# nb_contacts: number of fingers
# nb_segments: number of segments for fiction cone

# variables to optimize:
(1) contact points: x[0:3*nb_contacts]
(2) contact normal(outward): x[3*nb_contacts:6*nb_contacts]
(3) contact frame: x[6*nb_contacts:15*nb_contacts]
(4) coefficient:   x[15*nb_contacts:15*nb_contacts+nb_segments*nb_contacts]
(5) contact primitive: x[15*nb_contacts+nb_segments*nb_contacts:15*nb_contacts+nb_segments*nb_contacts*4]
(5) contact wrench: x[15*nb_contacts+nb_segments*nb_contacts*4:15*nb_contacts+nb_segments*nb_contacts*10]
"""

import pyipopt
from numpy import *
import numpy as np
import numdifftools as nd

nb_contacts = 3
nb_segments = 4

nvar = 15*nb_contacts+nb_segments*nb_contacts*10

print("There are "+repr(nvar)+" variables to optimize")

x_L = ones((nvar), dtype = float_)*(-1.0)

x_L[15*nb_contacts:15*nb_contacts+nb_segments*nb_contacts] = ones((nb_segments*nb_contacts), dtype = float_)*(0)

x_U = ones((nvar), dtype = float_)*(1.0)

# objective function
def eval_f(x, user_data = None):
	assert len(x) == nvar
	return sum(x[0:3*nb_contacts]**2)/nb_contacts*1000

def eval_grad_f(x, user_data = None):
	assert len(x) == nvar
	dfun = nd.Gradient(eval_f)
	return dfun(x)

# constraints:
dv = 0.02			# slack variable
mu = 1  			# coefficient of friction
s1 = 0.035 			# param for superquadrics
s2 = 0.035 			# param for superquadrics
s3 = 0.065 			# param for superquadrics
epsilon1 = 0.250 	# param for superquadrics
epsilon2 = 1.0 	    # param for superquadrics


ncon = nb_contacts+ 3*nb_contacts+3*nb_contacts+nb_contacts+8*nb_contacts+3*nb_contacts*nb_segments+\
	   nb_contacts*nb_segments*6+1+6

g_L_contact = ones(nb_contacts) + ones(nb_contacts)*(-dv)
g_U_contact = ones(nb_contacts) + ones(nb_contacts)*(dv)

g_L_normal= ones(3*nb_contacts)*(-dv)
g_U_normal= ones(3*nb_contacts)*(dv)

g_L_frame_norm = ones(3*nb_contacts) + ones(3*nb_contacts)*(-dv)
g_U_frame_norm = ones(3*nb_contacts) + ones(3*nb_contacts)*(dv)

g_L_normal_aligment_inner = zeros(nb_contacts)
g_U_normal_aligment_inner = ones(nb_contacts)*2.0*pow(10.0,19)

g_L_normal_aligment_cross = ones(8*nb_contacts)*(-dv)
g_U_normal_aligment_cross = ones(8*nb_contacts)*(dv)

g_L_contact_primitive = zeros(3*nb_contacts*nb_segments)
g_U_contact_primitive = zeros(3*nb_contacts*nb_segments)

g_L_contact_wrench = zeros(6*nb_contacts*nb_segments)
g_U_contact_wrench = zeros(6*nb_contacts*nb_segments)

g_L_coefficient = ones(1.0)-ones(1.0)*dv
g_U_coefficient = ones(1.0)+ones(1.0)*dv

g_L_force_closure = zeros(6)-ones(6)*dv
g_U_force_closure = zeros(6)+ones(6)*dv


g_L = concatenate([g_L_contact, g_L_normal,g_L_frame_norm,\
	g_L_normal_aligment_inner,g_L_normal_aligment_cross,\
	g_L_contact_primitive,g_L_contact_wrench,g_L_coefficient,g_L_force_closure])

g_U = concatenate([g_U_contact, g_U_normal,g_U_frame_norm,\
	g_U_normal_aligment_inner,g_U_normal_aligment_cross,\
	g_U_contact_primitive,g_U_contact_wrench,g_U_coefficient,g_U_force_closure])

#specify the structure of Jacobian matrix:
spRow= ones((12*nb_segments*nb_contacts),dtype = float_).reshape(nb_contacts,nb_segments,12)
spCol= ones((12*nb_segments*nb_contacts),dtype = float_).reshape(nb_contacts,nb_segments,12)
ind = 0
for i in range(nb_contacts):
	for j in range(nb_segments):
			spRow[i,j,:] = array([48+ind,48+ind,48+ind,48+ind,49+ind,49+ind,49+ind,49+ind,50+ind,50+ind,50+ind,50+ind])
			spCol[i,j,:] = array([57+ind,18+i*nb_contacts*3,21+i*nb_contacts*3,24+i*nb_contacts*3,\
				                  58+ind,19+i*nb_contacts*3,22+i*nb_contacts*3,25+i*nb_contacts*3,\
				                  59+ind,20+i*nb_contacts*3,23+i*nb_contacts*3,26+i*nb_contacts*3]) 
			ind= ind+3
spRow = spRow.reshape(-1)
spCol = spCol.reshape(-1)

spRow1= ones((21*nb_segments*nb_contacts),dtype = float_).reshape(nb_contacts,nb_segments,21)
spCol1= ones((21*nb_segments*nb_contacts),dtype = float_).reshape(nb_contacts,nb_segments,21)
ind = 0
for i in range(nb_contacts):
		for j in range(nb_segments):
			spRow1[i,j,:] = array([84+ind,84+ind,85+ind,85+ind,86+ind,86+ind,\
									87+ind,87+ind,87+ind,87+ind,87+ind,\
									88+ind,88+ind,88+ind,88+ind,88+ind,\
									89+ind,89+ind,89+ind,89+ind,89+ind])

			spCol1[i,j,:] = array([57+12*i+3*j,93+ind,\
								   58+12*i+3*j,94+ind,\
								   59+12*i+3*j,95+ind,\
								   1+3*i,2+3*i,58+12*i+3*j,59+12*i+3*j,96+ind,\
								   0+3*i,2+3*i,57+12*i+3*j,59+12*i+3*j,97+ind,\
								   0+3*i,1+3*i,57+12*i+3*j,58+12*i+3*j,98+ind,
								   ]) 
			ind= ind+6
spRow1 = spRow1.reshape(-1)
spCol1 = spCol1.reshape(-1)

SparseJacRow = np.concatenate([array([0, 0, 0, 1, 1, 1, 2, 2, 2]),\
							   array([3, 3, 3, 3, 4, 4, 4, 4, 5, 5]),\
							   array([6, 6, 6, 6,  7, 7, 7, 7, 8, 8]),\
							   array([9,9,9,9,10,10,10,10,11,11]),\
							   array([12,12,12,13,13,13,14,14,14,15,15,15,16,16,16,17,17,17,18,18,18,19,19,19,20,20,20]),\
							   array([21,21,21,21,21,21, 22,22,22,22,22,22, 23,23,23,23,23,23]),\
							   array([24,24,24,24]), array([25,25,25,25]),array([26,26,26,26]),\
							   array([27,27,27,27,27,27]),array([28,28,28,28,28,28]),\
							   array([29,29,29,29,29]),array([30,30,30,30,30]),array([31,31,31,31,31]),\
							   array([32,32,32,32]), array([33,33,33,33]),\
							   array([34,34,34,34]),array([35,35,35,35,35,35]),\
							   array([36,36,36,36,36,36]), array([37,37,37,37,37]),\
							   array([38,38,38,38,38]),array([39,39,39,39,39]),\
							   array([40,40,40,40]),array([41,41,41,41]), array([42,42,42,42]),\
							   array([43,43,43,43,43,43]), array([44,44,44,44,44,44]),\
							   array([45,45,45,45,45]), array([46,46,46,46,46]), array([47,47,47,47,47]),\
							   spRow,\
							   spRow1,\
							   ones(12)*156,\
							   ones(24)*157,ones(24)*158,ones(24)*159,ones(24)*160,ones(24)*161,ones(24)*162\
							   ])
SparseJacCol = np.concatenate([array([0, 1, 2, 3, 4, 5, 6, 7, 8]),\
							array([0, 1, 2, 9, 0, 1, 2, 10, 2, 11]),\
							array([3, 4 ,5, 12, 3, 4, 5, 13, 5, 14]),\
							array([6,7,8,15,6,7,8,16,8,17]),\
							array([18,19,20,21,22,23,24,25,26,27,18,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44]),\
							array([9,10,11,18,19,20,  12,13,14,27,28,29, 15,16,17,36,37,38]),\
							array([10,11,19,20]), array([9,11,18,20]),array([9,10,18,19]),\
							array([18,19,20,21,22,23]),array([18,19,20,24,25,26]),\
							array([18,22,23,25,26]),array([19,21,23,24,26]),array([20,21,22,24,25]),\
							array([13,14,28,29]),array([12,14,27,29]),array([12,13,27,28]),\
							array([27,28,29,30,31,32]),array([27,28,29,33,34,35]), array([27,31,32,34,35]),\
							array([28,30,32,33,35]), array([29,30,31,33,34]),\
							array([16,17,37,38]), array([15,17,36,38]),\
							array([15,16,36,37]), array([36,37,38,39,40,41]),\
							array([36,37,38,42,43,44]), array([36,40,41,43,44]),\
							array([37,39,41,42,44]), array([38,39,40,42,43]),\
							spCol,\
							spCol1,\
							array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)),\
							concatenate([array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(93,160,6))]),\
							concatenate([array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(94,161,6))]),\
							concatenate([array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(95,162,6))]),\
							concatenate([array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(96,163,6))]),\
							concatenate([array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(97,164,6))]),\
							concatenate([array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(98,165,6))])\
							])
print("sparse Jacobian structure, row index: " + repr(len(SparseJacRow)))
print("sparse Jacobian structure, col index: " + repr(len(SparseJacCol)))

def eval_g(x, user_data = None):	
	assert len(x) == nvar
	con_contact = zeros((nb_contacts), dtype = float_)  # points on object
	for i in range(nb_contacts):
		con_contact[i] = ((x[3*i]/s1)**(2.0/epsilon2) + \
			(x[3*i+1]/s2)**(2.0/epsilon2))**(epsilon2/epsilon1) + (x[3*i+2]/s3)**(2.0/epsilon1)
	"""
	sparsity
	[0, 0, 0, 1, 1, 1, 2, 2, 2]
    [0, 1, 2, 3, 4, 5, 6, 7, 8]
	"""
	con_normal = zeros((3*nb_contacts), dtype = float_)     # normal directions
	for i in range(nb_contacts):
		con_normal[3*i] = ((((x[3*i]/s1)**(2/epsilon2)) + \
				((x[3*i+1]/s2)**(2/epsilon2)))**(epsilon2/epsilon1 - 1))*((x[3*i+2]/s1)**(2/epsilon2-1)) -x[3*nb_contacts+3*i]
		
		con_normal[3*i+1] =((((x[3*i]/s1)**(2/epsilon2)) + \
			((x[3*i+1]/s2)**(2/epsilon2)))**(epsilon2/epsilon1 - 1))*((x[3*i+2]/s2)**(2/epsilon2-1))- x[3*nb_contacts+3*i+1]
		
		con_normal[3*i+2] = (x[3*i+2]/s3)**(2/epsilon1-1)- x[3*nb_contacts+3*i+2]        
	"""
	sparsity
	[3, 3, 3, 3, 4, 4, 4, 4, 5, 5]  [6, 6, 6, 6,  7, 7, 7, 7, 8, 8] [9,9,9,9,10,10,10,10,11,11]
	[0, 1, 2, 9, 0, 1, 2, 10, 2, 11] [3, 4 ,5 12, 3, 4, 5, 13, 5, 14] [6,7,8,15,6,7,8,16,8,17]
	""" 	
	# constraints for the contact frame:
	vframe = x[6*nb_contacts:15*nb_contacts].reshape(nb_contacts,3,-1)
	con_frame_norm = zeros((3*nb_contacts),dtype= float_)
	for i in range(nb_contacts):
		con_frame_norm[3*i] = vframe[i,0,0]**2+vframe[i,0,1]**2+vframe[i,0,2]**2
		con_frame_norm[3*i+1] = vframe[i,1,0]**2+vframe[i,1,1]**2+vframe[i,1,2]**2
		con_frame_norm[3*i+2] = vframe[i,2,0]**2+vframe[i,2,1]**2+vframe[i,2,2]**2
	"""
	sparsity
	[12,12,12,13,13,13,14,14,14,15,15,15,16,16,16,17,17,17,18,18,18,19,19,19,20,20,20]
	[18,19,20,21,22,23,24,25,26,27,18,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44]
	"""
	con_normal_alignment_inner = ones((nb_contacts),dtype = float_)  #inner product positive
	for i in range(nb_contacts):
		con_normal_alignment_inner[i] = vframe[i,0,0]*x[3*nb_contacts+3*i]+\
										vframe[i,0,1]*x[3*nb_contacts+3*i+1]+\
										vframe[i,0,2]*x[3*nb_contacts+3*i+2]
	"""
	sparsity
	[21,21,21,21,21,21, 22,22,22,22,22,22, 23,23,23,23,23,23]
	[9,10,11,18,19,20,  12,13,14,27,28,29, 15,16,17,36,37,38]
	"""
	con_normal_alignment_cross = ones((8*nb_contacts),dtype = float_)  #zero
	for i in range(nb_contacts):
		con_normal_alignment_cross[8*i] = vframe[i,0,1]*x[3*nb_contacts+3*i+2]-vframe[i,0,2]*x[3*nb_contacts+3*i+1]
		con_normal_alignment_cross[8*i+1] = vframe[i,0,2]*x[3*nb_contacts+3*i]-vframe[i,0,0]*x[3*nb_contacts+3*i+2]
		con_normal_alignment_cross[8*i+2] = vframe[i,0,0]*x[3*nb_contacts+3*i+1]-vframe[i,0,1]*x[3*nb_contacts+3*i]
		con_normal_alignment_cross[8*i+3] = vframe[i,0,0]*vframe[i,1,0]+vframe[i,0,1]*vframe[i,1,1]+vframe[i,0,2]*vframe[i,1,2]
		con_normal_alignment_cross[8*i+4] = vframe[i,0,0]*vframe[i,2,0]+vframe[i,0,1]*vframe[i,2,1]+vframe[i,0,2]*vframe[i,2,2]
		
		con_normal_alignment_cross[8*i+5] = vframe[i,1,1]*vframe[i,2,2]-vframe[i,1,2]*vframe[i,2,1]-vframe[i,0,0]
		con_normal_alignment_cross[8*i+6] = vframe[i,1,2]*vframe[i,2,0]-vframe[i,1,0]*vframe[i,2,2]-vframe[i,0,1]
		con_normal_alignment_cross[8*i+7] = vframe[i,1,0]*vframe[i,2,1]-vframe[i,1,1]*vframe[i,2,0]-vframe[i,0,2]
	"""
	sparsity
	[24,24,24,24] [25,25,25,25] [26,26,26,26] [27,27,27,27,27,27] [28,28,28,28,28,28] [29,29,29,29,29] [30,30,30,30,30] [31,31,31,31,31]
	[10,11,19,20] [9,11,18,20] [9,10,18,19]  [18,19,20,21,22,23]  [18,19,20,24,25,26] [18,22,23,25,26] [19,21,23,24,26] [20,21,22,24,25]

	[32,32,32,32] [33,33,33,33] [34,34,34,34] [35,35,35,35,35,35] [36,36,36,36,36,36] [37,37,37,37,37] [38,38,38,38,38] [39,39,39,39,39]
	[13,14,28,29] [12,14,27,29] [12,13,27,28]  [27,28,29,30,31,32]  [27,28,29,33,34,35] [27,31,32,34,35] [28,30,32,33,35] [29,30,31,33,34]
	
	[40,40,40,40] [41,41,41,41] [42,42,42,42] [43,43,43,43,43,43] [44,44,44,44,44,44] [45,45,45,45,45] [46,46,46,46,46] [47,47,47,47,47]
	[16,17,37,38] [15,17,36,38] [15,16,36,37]  [36,37,38,39,40,41]  [36,37,38,42,43,44] [36,40,41,43,44] [37,39,41,42,44] [38,39,40,42,43]
	"""
	# contact primitive
	con_contact_primitive = ones((3*nb_contacts*nb_segments),dtype = float_).reshape(nb_contacts,nb_segments, 3) 
	contact_primitive = x[15*nb_contacts+nb_segments*nb_contacts:15*nb_contacts+nb_segments*nb_contacts*4]\
						.reshape(nb_contacts,nb_segments,3)
	for i in range(nb_contacts):
		for j in range(nb_segments):
			for k in range(3):
				con_contact_primitive[i,j,k] = -contact_primitive[i,j,k] + vframe[i,1,k]*mu*cos(6.28*j/nb_segments)\
				+ vframe[i,2,k]*mu*sin(6.28*j/nb_segments) + vframe[i,0,k] 
	con_contact_primitive = con_contact_primitive.reshape(-1)
	"""
	sparsity
	[48,48,48,48] [49,49,49,49] [50,50,50,50]
	[57,18,21,24] [58,19,22,25] [59,20,23,26]

	spRow= ones((12*nb_segments*nb_contacts),dtype = float_).reshape(nb_contacts,nb_segments,12)
	spCol= ones((12*nb_segments*nb_contacts),dtype = float_).reshape(nb_contacts,nb_segments,12)
	ind = 0
	for i in range(nb_contacts):
		for j in range(nb_segments):
			spRow[i,j,:] = arrary([48+ind,48+ind,48+ind,48+ind,49+ind,49+ind,49+ind,49+ind,50+ind,50+ind,50+ind,50+ind])
			spCol[i,j,:] = array([57+ind,18+i*nb_contacts*3,21+i*nb_contacts*3,24+i*nb_contacts*3,\
				                  58+ind,19+i*nb_contacts*3,22+i*nb_contacts*3,25+i*nb_contacts*3,\
				                  59+ind,20+i*nb_contacts*3,23+i*nb_contacts*3,26+i*nb_contacts*3]) 
			ind= ind+3
 	spRow = spRow.reshape(-1)
 	spCol = spCol.reshape(-1)
	"""

	# contact wrench
	con_contact_wrench = zeros((6*nb_contacts*nb_segments),dtype = float_).reshape(nb_contacts,nb_segments, 6)
	contact_wrench = x[15*nb_contacts+nb_segments*nb_contacts*4:15*nb_contacts+nb_segments*nb_contacts*10]\
					 .reshape(nb_contacts,nb_segments,6)
	for i in range(nb_contacts):
		for j in range(nb_segments):
			con_contact_wrench[i,j,0] = -contact_wrench[i,j,0] + contact_primitive[i,j,0]
			con_contact_wrench[i,j,1] = -contact_wrench[i,j,1] + contact_primitive[i,j,1]
			con_contact_wrench[i,j,2] = -contact_wrench[i,j,2] + contact_primitive[i,j,2]
			con_contact_wrench[i,j,3] = -contact_wrench[i,j,3] + contact_primitive[i,j,2]*x[3*i+1]-\
																 contact_primitive[i,j,1]*x[3*i+2]
			con_contact_wrench[i,j,4] = -contact_wrench[i,j,4] - contact_primitive[i,j,2]*x[3*i+0]+\
																 contact_primitive[i,j,0]*x[3*i+2]
			con_contact_wrench[i,j,5] = -contact_wrench[i,j,5] + contact_primitive[i,j,1]*x[3*i+0]-\
																 contact_primitive[i,j,0]*x[3*i+1]
	con_contact_wrench = con_contact_wrench.reshape(-1)
	"""
	sparsity
	[84,84,85,85,86,86, 87,87,87,87,87, 88,88,88,88,88, 89,89,89,89,89]
	[57,93,58,94,59,95, 1,2,58,59,96, 0,2,57,59,97, 0,1,57,58,98]

	spRow1= ones((21*nb_segments*nb_contacts),dtype = float_).reshape(nb_contacts,nb_segments,21)
	spCol1= ones((21*nb_segments*nb_contacts),dtype = float_).reshape(nb_contacts,nb_segments,21)
	ind = 0
	for i in range(nb_contacts):
		for j in range(nb_segments):
			spRow1[i,j,:] = arrary([84+ind,84+ind,85+ind,85+ind,86+ind,86+ind,\
									87+ind,87+ind,87+ind,87+ind,87+ind,\
									88+ind,88+ind,88+ind,88+ind,88+ind,\
									89+ind,89+ind,89+ind,89+ind,89+ind])

			spCol1[i,j,:] = array([57+12*i+3*j,93+ind,\
								   58+12*i+3*j,94+ind,\
								   59+12*i+3*j,95+ind,\
								   1+3*i,2+3*i,58+12*i+3*j,59+12*i+3*j,96+ind,\
								   0+3*i,2+3*i,57+12*i+3*j,59+12*i+3*j,97+ind,\
								   0+3*i,1+3*i,57+12*i+3*j,58+12*i+3*j,98+ind,
								   ]) 
			ind= ind+6
 	spRow1 = spRow1.reshape(-1)
 	spCol1 = spCol1.reshape(-1)
	"""
	# force closure constraints
	con_coefficient = ones((1),dtype = float_)
	coefficient = x[15*nb_contacts:15*nb_contacts+nb_segments*nb_contacts]
	con_coefficient = array([sum(coefficient)])
	"""
	sparsity
	ones(12)*156
	array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts))
	"""
	coefficient = coefficient.reshape(nb_contacts,nb_segments)

	con_force_closure = zeros((6),dtype= float_)
	for k in range(6):
				con_force_closure[k] = sum(np.multiply(coefficient, contact_wrench[:,:,k]))
	"""
	ones(24)*157
	concatenate(array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(93,159,6)))

	ones(24)*158
	concatenate(array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(94,160,6)))

	ones(24)*159
	concatenate(array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(95,161,6)))

	ones(24)*160
	concatenate(array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(96,162,6)))

	ones(24)*161
	concatenate(array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(97,163,6)))

	ones(24)*162
	concatenate(array(range(15*nb_contacts,15*nb_contacts+nb_segments*nb_contacts)), array(range(98,164,6)))
	"""
	g_constraints = np.concatenate([con_contact,con_normal,con_frame_norm,\
		con_normal_alignment_inner,con_normal_alignment_cross,con_contact_primitive,\
		con_contact_wrench,con_coefficient,con_force_closure])	
	return g_constraints

def eval_jac_g(x,flag,user_data = None):
	if flag:		
		return(SparseJacRow,
			   SparseJacCol)
	else:
		assert len(x) == nvar
		jacg = nd.Jacobian(eval_g)
		return jacg(x).reshape(-1)

nnzj = len(SparseJacRow)
nnzh = 0

def apply_new(x):
	return True

nlp = pyipopt.create(nvar, x_L, x_U, ncon, g_L, g_U, nnzj, nnzh, eval_f, eval_grad_f, eval_g, eval_jac_g)

nlp.int_option('max_iter', 10)
nlp.num_option('tol', 10e-2)
nlp.num_option('acceptable_tol',10e-3)
nlp.num_option('bound_relax_factor',0.1)
nlp.num_option('dual_inf_tol',1.0)
nlp.str_option('mu_strategy','adaptive')

x0 = ones((nvar), dtype = float_)*(0.3)
x0[15*nb_contacts:15*nb_contacts+nb_segments*nb_contacts]=\
					ones((nb_segments*nb_contacts), dtype = float_)*(1.0/nb_segments*nb_contacts)
vtmp = x0[6*nb_contacts:15*nb_contacts].reshape(nb_contacts,3,-1)
for i in range(nb_contacts):
	vtmp[i,0,0]= 0
	vtmp[i,0,1]= 0
	vtmp[i,0,2]= 1
	vtmp[i,1,0]= 0.7071
	vtmp[i,1,1]= -0.7071
	vtmp[i,1,2]= 0
	vtmp[i,2,0]= 0.7071
	vtmp[i,2,1]= 0.7071
	vtmp[i,2,2]= 1
x0[6*nb_contacts:15*nb_contacts] = vtmp.reshape(-1)

x, zl, zu, constraint_multipliers, obj, status = nlp.solve(x0)

nlp.close()

def print_variable(variable_name, value):
  for i in xrange(len(value)):
	print variable_name + "["+str(i)+"] =", value[i]

print
print "Solution of the primal variables, x"
print_variable("x", x)
print
print "Objective value"
print "f(x*) =", obj

"""
print "Solution of the bound multipliers, z_L and z_U"
print_variable("z_L", zl)
print_variable("z_U", zu)
print
print "Solution of the constraint multipliers, lambda"
print_variable("lambda", constraint_multipliers)
print
print "Objective value"
print "f(x*) =", obj
"""