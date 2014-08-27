function   [solve_time,distance, Hand_pos, Finger_joint] = readResult(fname)

%  Hand_pos: 9x1 vector the cos and sin of 3 Euler angles and the postion
%  Finger_joint: the cos and sin of finger joint;
%  open file for input, include error handling
fid = fopen(fname,'r');
if fid < 0
   error(['Could not open ',fname,' for input']);
end

for i=1:4
    buffer = fgetl(fid);
end

tline = fgetl(fid);
 [a,b,c] = strread(tline,'%s %s %f');
solve_time = c;

for i=1:2
    buffer = fgetl(fid);
end

tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
distance = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnh1 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnh2 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnh3 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snh1 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snh2 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snh3 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
Hx = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
Hy = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
Hz = c;

Hand_pos=[cnh1;cnh2;cnh3;snh1;snh2;snh3;Hx;Hy;Hz;];

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnf11 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnf21 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnf31 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnf12 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnf22 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnf32 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnt1 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
cnt2 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snf11 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snf21 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snf31 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snf12 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snf22 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snf32 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snt1 = c;

tline = fgetl(fid);
tline = fgetl(fid);
[a,b,c] = strread(tline,'%s %s %f');
snt2 = c;

Finger_joint=[cnf11;cnf21;cnf31;cnf12;cnf22;cnf32;cnt1;cnt2;snf11;snf21;snf31;snf12;snf22;snf32;snt1;snt2];


