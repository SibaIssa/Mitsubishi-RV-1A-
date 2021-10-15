syms m1 m2 m3 m4 m5 m6;
syms q1 q2 q3 q4 q5 q6;
syms dq1 dq2 dq3 dq4 dq5 dq6;
syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6;
syms l1 l2 l3 l4 l5 l6;
syms u1 u2 u3 u4 u5 u6;
syms g
%equations
x1 = l1*cos(q1);
x2 = x1+l2*cos(q2);
x3 = x2+l3*cos(q2+q3);
x4 = x3+l4*cos(q1+q4);
x5 = x4;
x6 = x5+l6*cos(q1+q4+q6);

y1 = l1*sin(q1);
y2 = y1;
y3 = y2;
y4 = y3+l4*sin(q1+q4);
y5 = y4+l5*cos(q5);
y6 = y5+l6*sin(q1+q4+q6);

z1 = 0;
z2 = z1+l2*sin(q2);
z3 = z2+l3*cos(q2+q3);
z4 = z3;
z5 = z4+l5*sin(q5);
z6 = z5;

%translation jacobians
jv1=[diff(x1,q1) diff(x1,q2) diff(x1,q3) diff(x1,q4) diff(x1,q5) diff(x1,q6); ...
     diff(y1,q1) diff(y1,q2) diff(y1,q3) diff(y1,q4) diff(y1,q5) diff(y1,q6); ...
     diff(z1,q1) diff(z1,q2) diff(z1,q3) diff(z1,q4) diff(y1,q5) diff(z1,q6)];
 
jv2=[diff(x2,q1) diff(x2,q2) diff(x2,q3) diff(x2,q4) diff(x2,q5) diff(x2,q6); ...
     diff(y2,q1) diff(y2,q2) diff(y2,q3) diff(y2,q4) diff(y2,q5) diff(y2,q6); ...
     diff(z2,q1) diff(z2,q2) diff(z2,q3) diff(z2,q4) diff(y2,q5) diff(z2,q6)];

jv3=[diff(x3,q1) diff(x3,q2) diff(x3,q3) diff(x3,q4) diff(x3,q5) diff(x3,q6); ...
     diff(y3,q1) diff(y3,q2) diff(y3,q3) diff(y3,q4) diff(y3,q5) diff(y3,q6); ...
     diff(z3,q1) diff(z3,q2) diff(z3,q3) diff(z3,q4) diff(y3,q5) diff(z3,q6)];

jv4=[diff(x4,q1) diff(x4,q2) diff(x4,q3) diff(x4,q4) diff(x4,q5) diff(x4,q6); ...
     diff(y4,q1) diff(y4,q2) diff(y4,q3) diff(y4,q4) diff(y4,q5) diff(y4,q6); ...
     diff(z4,q1) diff(z4,q2) diff(z4,q3) diff(z4,q4) diff(y4,q5) diff(z4,q6)]; 
 
jv5=[diff(x5,q1) diff(x5,q2) diff(x5,q3) diff(x5,q4) diff(x5,q5) diff(x5,q6); ...
     diff(y5,q1) diff(y5,q2) diff(y5,q3) diff(y5,q4) diff(y5,q5) diff(y5,q6); ...
     diff(z5,q1) diff(z5,q2) diff(z5,q3) diff(z5,q4) diff(y5,q5) diff(z5,q6)];
 
jv6=[diff(x6,q1) diff(x6,q2) diff(x6,q3) diff(x6,q4) diff(x6,q5) diff(x6,q6); ...
     diff(y6,q1) diff(y6,q2) diff(y6,q3) diff(y6,q4) diff(y6,q5) diff(y6,q6); ...
     diff(z6,q1) diff(z6,q2) diff(z6,q3) diff(z6,q4) diff(y6,q5) diff(z6,q6)];

%rotation jacobians
jw1=[0 0 0 0 0 0;...
     0 0 0 0 0 0;...
     1 0 0 0 0 0];

jw2=[0 0 0 0 0 0;...
     0 1 0 0 0 0;...
     0 0 0 0 0 0];
 
jw3=[0 0 0 0 0 0;...
     0 0 1 0 0 0;...
     0 0 0 0 0 0];
 
jw4=[0 0 0 0 0 0;...
     0 0 0 0 0 0;...
     0 0 0 1 0 0];
 
jw5=[0 0 0 0 1 0;...
     0 0 0 0 0 0;...
     0 0 0 0 0 0];
 
jw6=[0 0 0 0 0 0;...
     0 0 0 0 0 0;...
     0 0 0 0 0 1];

%rotation matrices
R1= [cos(q1) -sin(q1)   0;...
     sin(q1)  cos(q1)   0;...
       0        0       1];
   
R2= [cos(q2)  0  sin(q2);...
         0    1       0 ;...
    -sin(q2)  0  cos(q2)];
 
R3= [cos(q3)  0  sin(q3);...
         0    1       0 ;...
    -sin(q3)  0  cos(q3)];

R4= [cos(q4) -sin(q4)   0;...
     sin(q4)  cos(q4)   0;...
       0        0       1];
 
R5= [1     0            0;...
     0   cos(q5) -sin(q5);...
     0   sin(q5)  cos(q5)];
   
R6= [cos(q6) -sin(q6)   0;...
     sin(q6)  cos(q6)   0;...
       0        0       1];
%Inertia matrix 
I= [1 0 0;...
    0 1 0;...
    0 0 1];

%Math Matrix M(q)
Mq= m1*jv1'*jv1 + + jw1'*R1*I*R1'*jw1+...
    m1*jv2'*jv2 + + jw2'*R2*I*R2'*jw2+...
    m1*jv3'*jv3 + + jw3'*R3*I*R3'*jw3+...
    m1*jv4'*jv4 + + jw4'*R4*I*R4'*jw4+...
    m1*jv5'*jv5 + + jw5'*R5*I*R5'*jw5+...
    m1*jv6'*jv6 + + jw5'*R6*I*R6'*jw6;

M=(simplify(Mq))

%Colriolis Matrix C(q,dq)
m11= Mq(1,1);  m12= Mq(1,2);  m13= Mq(1,3); m14= Mq(1,1);  m15= Mq(1,2);  m16= Mq(1,3);
m21= Mq(2,1);  m22= Mq(2,2);  m23= Mq(2,2); m24= Mq(2,1);  m25= Mq(2,2);  m26= Mq(2,2);
m31= Mq(3,1);  m32= Mq(3,1);  m33= Mq(3,3); m34= Mq(3,1);  m35= Mq(3,1);  m36= Mq(3,3);
m41= Mq(4,1);  m42= Mq(4,2);  m43= Mq(4,3); m44= Mq(4,1);  m45= Mq(4,2);  m46= Mq(4,3);
m51= Mq(5,1);  m52= Mq(5,2);  m53= Mq(5,2); m54= Mq(5,1);  m55= Mq(5,2);  m56= Mq(5,2);
m61= Mq(6,1);  m62= Mq(6,1);  m63= Mq(6,3); m64= Mq(6,1);  m65= Mq(6,1);  m66= Mq(6,3);

c111= 0.5*(diff(m11,q1)+diff(m11,q1)-diff(m11,q1));
c112= 0.5*(diff(m11,q2)+diff(m12,q1)-diff(m12,q1));
c113= 0.5*(diff(m11,q3)+diff(m13,q1)-diff(m13,q1));
c114= 0.5*(diff(m11,q4)+diff(m14,q1)-diff(m14,q1));
c115= 0.5*(diff(m11,q5)+diff(m15,q1)-diff(m15,q1));
c116= 0.5*(diff(m11,q6)+diff(m16,q1)-diff(m16,q1));

c121= 0.5*(diff(m12,q1)+diff(m11,q2)-diff(m21,q1));
c122= 0.5*(diff(m12,q2)+diff(m12,q2)-diff(m22,q1));
c123= 0.5*(diff(m12,q3)+diff(m13,q2)-diff(m23,q1));
c124= 0.5*(diff(m12,q4)+diff(m14,q2)-diff(m24,q1));
c125= 0.5*(diff(m12,q5)+diff(m15,q2)-diff(m25,q1));
c126= 0.5*(diff(m12,q6)+diff(m16,q2)-diff(m26,q1));

c131= 0.5*(diff(m13,q1)+diff(m11,q3)-diff(m31,q1));
c132= 0.5*(diff(m13,q2)+diff(m12,q3)-diff(m32,q1));
c133= 0.5*(diff(m13,q3)+diff(m13,q3)-diff(m33,q1));
c134= 0.5*(diff(m13,q4)+diff(m14,q3)-diff(m34,q1));
c135= 0.5*(diff(m13,q5)+diff(m15,q3)-diff(m35,q1));
c136= 0.5*(diff(m13,q6)+diff(m16,q3)-diff(m36,q1));

c141= 0.5*(diff(m14,q1)+diff(m11,q4)-diff(m41,q1));
c142= 0.5*(diff(m14,q2)+diff(m12,q4)-diff(m42,q1));
c143= 0.5*(diff(m14,q3)+diff(m13,q4)-diff(m43,q1));
c144= 0.5*(diff(m14,q4)+diff(m14,q4)-diff(m44,q1));
c145= 0.5*(diff(m14,q5)+diff(m15,q4)-diff(m45,q1));
c146= 0.5*(diff(m14,q6)+diff(m16,q4)-diff(m46,q1));

c151= 0.5*(diff(m15,q1)+diff(m11,q5)-diff(m51,q1));
c152= 0.5*(diff(m15,q2)+diff(m12,q5)-diff(m52,q1));
c153= 0.5*(diff(m15,q3)+diff(m13,q5)-diff(m53,q1));
c154= 0.5*(diff(m15,q4)+diff(m14,q5)-diff(m54,q1));
c155= 0.5*(diff(m15,q5)+diff(m15,q5)-diff(m55,q1));
c156= 0.5*(diff(m15,q6)+diff(m16,q5)-diff(m56,q1));

c161= 0.5*(diff(m16,q1)+diff(m11,q6)-diff(m61,q1));
c162= 0.5*(diff(m16,q2)+diff(m12,q6)-diff(m62,q1));
c163= 0.5*(diff(m16,q3)+diff(m13,q6)-diff(m63,q1));
c164= 0.5*(diff(m16,q4)+diff(m14,q6)-diff(m64,q1));
c165= 0.5*(diff(m16,q5)+diff(m15,q6)-diff(m65,q1));
c166= 0.5*(diff(m16,q6)+diff(m16,q6)-diff(m66,q1));

c211= 0.5*(diff(m21,q1)+diff(m21,q1)-diff(m11,q2));
c212= 0.5*(diff(m21,q2)+diff(m22,q1)-diff(m12,q2));
c213= 0.5*(diff(m21,q3)+diff(m23,q1)-diff(m13,q2));
c214= 0.5*(diff(m21,q4)+diff(m24,q1)-diff(m14,q2));
c215= 0.5*(diff(m21,q5)+diff(m25,q1)-diff(m15,q2));
c216= 0.5*(diff(m21,q6)+diff(m26,q1)-diff(m16,q2));

c221= 0.5*(diff(m22,q1)+diff(m21,q2)-diff(m21,q2));
c222= 0.5*(diff(m22,q2)+diff(m22,q2)-diff(m22,q2));
c223= 0.5*(diff(m22,q3)+diff(m23,q2)-diff(m23,q2));
c224= 0.5*(diff(m22,q4)+diff(m24,q2)-diff(m24,q2));
c225= 0.5*(diff(m22,q5)+diff(m25,q2)-diff(m25,q2));
c226= 0.5*(diff(m22,q6)+diff(m26,q2)-diff(m26,q2));

c231= 0.5*(diff(m23,q1)+diff(m21,q3)-diff(m31,q2));
c232= 0.5*(diff(m23,q2)+diff(m22,q3)-diff(m32,q2));
c233= 0.5*(diff(m23,q3)+diff(m23,q3)-diff(m33,q2));
c234= 0.5*(diff(m23,q4)+diff(m24,q3)-diff(m34,q2));
c235= 0.5*(diff(m23,q5)+diff(m25,q3)-diff(m35,q2));
c236= 0.5*(diff(m23,q6)+diff(m26,q3)-diff(m36,q2));

c241= 0.5*(diff(m24,q1)+diff(m21,q4)-diff(m41,q2));
c242= 0.5*(diff(m24,q2)+diff(m22,q4)-diff(m42,q2));
c243= 0.5*(diff(m24,q3)+diff(m23,q4)-diff(m43,q2));
c244= 0.5*(diff(m24,q4)+diff(m24,q4)-diff(m44,q2));
c245= 0.5*(diff(m24,q5)+diff(m25,q4)-diff(m45,q2));
c246= 0.5*(diff(m24,q6)+diff(m26,q4)-diff(m46,q2));

c251= 0.5*(diff(m25,q1)+diff(m21,q5)-diff(m51,q2));
c252= 0.5*(diff(m25,q2)+diff(m22,q5)-diff(m52,q2));
c253= 0.5*(diff(m25,q3)+diff(m23,q5)-diff(m53,q2));
c254= 0.5*(diff(m25,q4)+diff(m24,q5)-diff(m54,q2));
c255= 0.5*(diff(m25,q5)+diff(m25,q5)-diff(m55,q2));
c256= 0.5*(diff(m25,q6)+diff(m26,q5)-diff(m56,q2));

c261= 0.5*(diff(m26,q1)+diff(m21,q6)-diff(m61,q2));
c262= 0.5*(diff(m26,q2)+diff(m22,q6)-diff(m62,q2));
c263= 0.5*(diff(m26,q3)+diff(m23,q6)-diff(m63,q2));
c264= 0.5*(diff(m26,q4)+diff(m24,q6)-diff(m64,q2));
c265= 0.5*(diff(m26,q5)+diff(m25,q6)-diff(m65,q2));
c266= 0.5*(diff(m26,q6)+diff(m26,q6)-diff(m66,q2));

c311= 0.5*(diff(m31,q1)+diff(m31,q1)-diff(m11,q3));
c312= 0.5*(diff(m31,q1)+diff(m32,q1)-diff(m12,q3));
c313= 0.5*(diff(m31,q3)+diff(m33,q1)-diff(m13,q3));
c314= 0.5*(diff(m31,q4)+diff(m34,q1)-diff(m14,q3));
c315= 0.5*(diff(m31,q5)+diff(m35,q1)-diff(m15,q3));
c316= 0.5*(diff(m31,q6)+diff(m36,q1)-diff(m16,q3));

c321= 0.5*(diff(m32,q1)+diff(m31,q2)-diff(m21,q3));
c322= 0.5*(diff(m32,q1)+diff(m32,q2)-diff(m22,q3));
c323= 0.5*(diff(m32,q3)+diff(m33,q2)-diff(m23,q3));
c324= 0.5*(diff(m32,q4)+diff(m34,q2)-diff(m24,q3));
c325= 0.5*(diff(m32,q5)+diff(m35,q2)-diff(m25,q3));
c326= 0.5*(diff(m32,q6)+diff(m36,q2)-diff(m26,q3));

c331= 0.5*(diff(m33,q1)+diff(m31,q3)-diff(m31,q3));
c332= 0.5*(diff(m33,q1)+diff(m32,q3)-diff(m32,q3));
c333= 0.5*(diff(m33,q3)+diff(m33,q3)-diff(m33,q3));
c334= 0.5*(diff(m33,q4)+diff(m34,q3)-diff(m34,q3));
c335= 0.5*(diff(m33,q5)+diff(m35,q3)-diff(m35,q3));
c336= 0.5*(diff(m33,q6)+diff(m36,q3)-diff(m36,q3));

c341= 0.5*(diff(m34,q1)+diff(m31,q4)-diff(m41,q3));
c342= 0.5*(diff(m34,q1)+diff(m32,q4)-diff(m42,q3));
c343= 0.5*(diff(m34,q3)+diff(m33,q4)-diff(m43,q3));
c344= 0.5*(diff(m34,q4)+diff(m34,q4)-diff(m44,q3));
c345= 0.5*(diff(m34,q5)+diff(m35,q4)-diff(m45,q3));
c346= 0.5*(diff(m34,q6)+diff(m36,q4)-diff(m46,q3));

c351= 0.5*(diff(m35,q1)+diff(m31,q5)-diff(m51,q3));
c352= 0.5*(diff(m35,q1)+diff(m32,q5)-diff(m52,q3));
c353= 0.5*(diff(m35,q3)+diff(m33,q5)-diff(m53,q3));
c354= 0.5*(diff(m35,q4)+diff(m34,q5)-diff(m54,q3));
c355= 0.5*(diff(m35,q5)+diff(m35,q5)-diff(m55,q3));
c356= 0.5*(diff(m35,q6)+diff(m36,q5)-diff(m56,q3));

c361= 0.5*(diff(m36,q1)+diff(m31,q6)-diff(m61,q3));
c362= 0.5*(diff(m36,q1)+diff(m32,q6)-diff(m62,q3));
c363= 0.5*(diff(m36,q3)+diff(m33,q6)-diff(m63,q3));
c364= 0.5*(diff(m36,q4)+diff(m34,q6)-diff(m64,q3));
c365= 0.5*(diff(m36,q5)+diff(m35,q6)-diff(m65,q3));
c366= 0.5*(diff(m36,q6)+diff(m36,q6)-diff(m66,q3));

c411= 0.5*(diff(m41,q1)+diff(m41,q1)-diff(m11,q4));
c412= 0.5*(diff(m41,q2)+diff(m42,q1)-diff(m12,q4));
c413= 0.5*(diff(m41,q3)+diff(m43,q1)-diff(m13,q4));
c414= 0.5*(diff(m41,q4)+diff(m44,q1)-diff(m14,q4));
c415= 0.5*(diff(m41,q5)+diff(m45,q1)-diff(m15,q4));
c416= 0.5*(diff(m41,q6)+diff(m46,q1)-diff(m16,q4));

c421= 0.5*(diff(m42,q1)+diff(m41,q2)-diff(m21,q4));
c422= 0.5*(diff(m42,q2)+diff(m42,q2)-diff(m22,q4));
c423= 0.5*(diff(m42,q3)+diff(m43,q2)-diff(m23,q4));
c424= 0.5*(diff(m42,q4)+diff(m44,q2)-diff(m24,q4));
c425= 0.5*(diff(m42,q5)+diff(m45,q2)-diff(m25,q4));
c426= 0.5*(diff(m42,q6)+diff(m46,q2)-diff(m26,q4));

c431= 0.5*(diff(m43,q1)+diff(m41,q3)-diff(m31,q4));
c432= 0.5*(diff(m43,q2)+diff(m42,q3)-diff(m32,q4));
c433= 0.5*(diff(m43,q3)+diff(m43,q3)-diff(m33,q4));
c434= 0.5*(diff(m43,q4)+diff(m44,q3)-diff(m34,q4));
c435= 0.5*(diff(m43,q5)+diff(m45,q3)-diff(m35,q4));
c436= 0.5*(diff(m43,q6)+diff(m46,q3)-diff(m36,q4));

c441= 0.5*(diff(m44,q1)+diff(m41,q4)-diff(m41,q4));
c442= 0.5*(diff(m44,q2)+diff(m42,q4)-diff(m42,q4));
c443= 0.5*(diff(m44,q3)+diff(m43,q4)-diff(m43,q4));
c444= 0.5*(diff(m44,q4)+diff(m44,q4)-diff(m44,q4));
c445= 0.5*(diff(m44,q5)+diff(m45,q4)-diff(m45,q4));
c446= 0.5*(diff(m44,q6)+diff(m46,q4)-diff(m46,q4));

c451= 0.5*(diff(m45,q1)+diff(m41,q5)-diff(m51,q4));
c452= 0.5*(diff(m45,q2)+diff(m42,q5)-diff(m52,q4));
c453= 0.5*(diff(m45,q3)+diff(m43,q5)-diff(m53,q4));
c454= 0.5*(diff(m45,q4)+diff(m44,q5)-diff(m54,q4));
c455= 0.5*(diff(m45,q5)+diff(m45,q5)-diff(m55,q4));
c456= 0.5*(diff(m45,q6)+diff(m46,q5)-diff(m56,q4));

c461= 0.5*(diff(m46,q1)+diff(m41,q6)-diff(m61,q4));
c462= 0.5*(diff(m46,q2)+diff(m42,q6)-diff(m62,q4));
c463= 0.5*(diff(m46,q3)+diff(m43,q6)-diff(m63,q4));
c464= 0.5*(diff(m46,q4)+diff(m44,q6)-diff(m64,q4));
c465= 0.5*(diff(m46,q5)+diff(m45,q6)-diff(m65,q4));
c466= 0.5*(diff(m46,q6)+diff(m46,q6)-diff(m66,q4));

c511= 0.5*(diff(m51,q1)+diff(m51,q1)-diff(m11,q5));
c512= 0.5*(diff(m51,q2)+diff(m52,q1)-diff(m12,q5));
c513= 0.5*(diff(m51,q3)+diff(m53,q1)-diff(m13,q5));
c514= 0.5*(diff(m51,q4)+diff(m54,q1)-diff(m14,q5));
c515= 0.5*(diff(m51,q5)+diff(m55,q1)-diff(m15,q5));
c516= 0.5*(diff(m51,q6)+diff(m56,q1)-diff(m16,q5));

c521= 0.5*(diff(m52,q1)+diff(m51,q2)-diff(m21,q5));
c522= 0.5*(diff(m52,q2)+diff(m52,q2)-diff(m22,q5));
c523= 0.5*(diff(m52,q3)+diff(m53,q2)-diff(m23,q5));
c524= 0.5*(diff(m52,q4)+diff(m54,q2)-diff(m24,q5));
c525= 0.5*(diff(m52,q5)+diff(m55,q2)-diff(m25,q5));
c526= 0.5*(diff(m52,q6)+diff(m56,q2)-diff(m26,q5));

c531= 0.5*(diff(m53,q1)+diff(m51,q3)-diff(m31,q5));
c532= 0.5*(diff(m53,q2)+diff(m52,q3)-diff(m32,q5));
c533= 0.5*(diff(m53,q3)+diff(m53,q3)-diff(m33,q5));
c534= 0.5*(diff(m53,q4)+diff(m54,q3)-diff(m34,q5));
c535= 0.5*(diff(m53,q5)+diff(m55,q3)-diff(m35,q5));
c536= 0.5*(diff(m53,q6)+diff(m56,q3)-diff(m36,q5));

c541= 0.5*(diff(m54,q1)+diff(m51,q4)-diff(m41,q5));
c542= 0.5*(diff(m54,q2)+diff(m52,q4)-diff(m42,q5));
c543= 0.5*(diff(m54,q3)+diff(m53,q4)-diff(m43,q5));
c544= 0.5*(diff(m54,q4)+diff(m54,q4)-diff(m44,q5));
c545= 0.5*(diff(m54,q5)+diff(m55,q4)-diff(m45,q5));
c546= 0.5*(diff(m54,q6)+diff(m56,q4)-diff(m46,q5));

c551= 0.5*(diff(m55,q1)+diff(m51,q5)-diff(m51,q5));
c552= 0.5*(diff(m55,q2)+diff(m52,q5)-diff(m52,q5));
c553= 0.5*(diff(m55,q3)+diff(m53,q5)-diff(m53,q5));
c554= 0.5*(diff(m55,q4)+diff(m54,q5)-diff(m54,q5));
c555= 0.5*(diff(m55,q5)+diff(m55,q5)-diff(m55,q5));
c556= 0.5*(diff(m55,q6)+diff(m56,q5)-diff(m56,q5));

c561= 0.5*(diff(m56,q1)+diff(m51,q6)-diff(m61,q5));
c562= 0.5*(diff(m56,q2)+diff(m52,q6)-diff(m62,q5));
c563= 0.5*(diff(m56,q3)+diff(m53,q6)-diff(m63,q5));
c564= 0.5*(diff(m56,q4)+diff(m54,q6)-diff(m64,q5));
c565= 0.5*(diff(m56,q5)+diff(m55,q6)-diff(m65,q5));
c566= 0.5*(diff(m56,q6)+diff(m56,q6)-diff(m66,q5));

c611= 0.5*(diff(m61,q1)+diff(m61,q1)-diff(m11,q6));
c612= 0.5*(diff(m61,q1)+diff(m62,q1)-diff(m12,q6));
c613= 0.5*(diff(m61,q3)+diff(m63,q1)-diff(m13,q6));
c614= 0.5*(diff(m61,q4)+diff(m64,q1)-diff(m14,q6));
c615= 0.5*(diff(m61,q5)+diff(m65,q1)-diff(m15,q6));
c616= 0.5*(diff(m61,q6)+diff(m66,q1)-diff(m16,q6));

c621= 0.5*(diff(m62,q1)+diff(m61,q2)-diff(m21,q6));
c622= 0.5*(diff(m62,q1)+diff(m62,q2)-diff(m22,q6));
c623= 0.5*(diff(m62,q3)+diff(m63,q2)-diff(m23,q6));
c624= 0.5*(diff(m62,q4)+diff(m64,q2)-diff(m24,q6));
c625= 0.5*(diff(m62,q5)+diff(m65,q2)-diff(m25,q6));
c626= 0.5*(diff(m62,q6)+diff(m66,q2)-diff(m26,q6));

c631= 0.5*(diff(m63,q1)+diff(m61,q3)-diff(m31,q6));
c632= 0.5*(diff(m63,q1)+diff(m62,q3)-diff(m32,q6));
c633= 0.5*(diff(m63,q3)+diff(m63,q3)-diff(m33,q6));
c634= 0.5*(diff(m63,q4)+diff(m64,q3)-diff(m34,q6));
c635= 0.5*(diff(m63,q5)+diff(m65,q3)-diff(m35,q6));
c636= 0.5*(diff(m63,q6)+diff(m66,q3)-diff(m36,q6));

c641= 0.5*(diff(m64,q1)+diff(m61,q4)-diff(m41,q6));
c642= 0.5*(diff(m64,q1)+diff(m62,q4)-diff(m42,q6));
c643= 0.5*(diff(m64,q3)+diff(m63,q4)-diff(m43,q6));
c644= 0.5*(diff(m64,q4)+diff(m64,q4)-diff(m44,q6));
c645= 0.5*(diff(m64,q5)+diff(m65,q4)-diff(m45,q6));
c646= 0.5*(diff(m64,q6)+diff(m66,q4)-diff(m46,q6));

c651= 0.5*(diff(m65,q1)+diff(m61,q5)-diff(m51,q6));
c652= 0.5*(diff(m65,q1)+diff(m62,q5)-diff(m52,q6));
c653= 0.5*(diff(m65,q3)+diff(m63,q5)-diff(m53,q6));
c654= 0.5*(diff(m65,q4)+diff(m64,q5)-diff(m54,q6));
c655= 0.5*(diff(m65,q5)+diff(m65,q5)-diff(m55,q6));
c656= 0.5*(diff(m65,q6)+diff(m66,q5)-diff(m56,q6));

c661= 0.5*(diff(m66,q1)+diff(m61,q6)-diff(m61,q6));
c662= 0.5*(diff(m66,q1)+diff(m62,q6)-diff(m62,q6));
c663= 0.5*(diff(m66,q3)+diff(m63,q6)-diff(m63,q6));
c664= 0.5*(diff(m66,q4)+diff(m64,q6)-diff(m64,q6));
c665= 0.5*(diff(m66,q5)+diff(m65,q6)-diff(m65,q6));
c666= 0.5*(diff(m66,q6)+diff(m66,q6)-diff(m66,q6));

c11= c111*diff(q1)+c112*diff(q2)+c113*diff(q3)+c114*diff(q4)+c115*diff(q5)+c116*diff(q6);
c12= c121*diff(q1)+c122*diff(q2)+c123*diff(q3)+c124*diff(q4)+c125*diff(q5)+c126*diff(q6);
c13= c131*diff(q1)+c132*diff(q2)+c133*diff(q3)+c134*diff(q4)+c135*diff(q5)+c136*diff(q6);
c14= c141*diff(q1)+c142*diff(q2)+c143*diff(q3)+c144*diff(q4)+c145*diff(q5)+c146*diff(q6);
c15= c151*diff(q1)+c152*diff(q2)+c153*diff(q3)+c154*diff(q4)+c155*diff(q5)+c156*diff(q6);
c16= c161*diff(q1)+c162*diff(q2)+c163*diff(q3)+c164*diff(q4)+c165*diff(q5)+c166*diff(q6);
c21= c211*diff(q1)+c212*diff(q2)+c213*diff(q3)+c214*diff(q4)+c215*diff(q5)+c216*diff(q6);
c22= c221*diff(q1)+c222*diff(q2)+c223*diff(q3)+c224*diff(q4)+c225*diff(q5)+c226*diff(q6);
c23= c231*diff(q1)+c232*diff(q2)+c233*diff(q3)+c234*diff(q4)+c235*diff(q5)+c236*diff(q6);
c24= c241*diff(q1)+c242*diff(q2)+c243*diff(q3)+c244*diff(q4)+c245*diff(q5)+c246*diff(q6);
c25= c251*diff(q1)+c252*diff(q2)+c253*diff(q3)+c254*diff(q4)+c255*diff(q5)+c256*diff(q6);
c26= c261*diff(q1)+c262*diff(q2)+c263*diff(q3)+c264*diff(q4)+c265*diff(q5)+c266*diff(q6);
c31= c311*diff(q1)+c312*diff(q2)+c313*diff(q3)+c314*diff(q4)+c315*diff(q5)+c316*diff(q6);
c32= c321*diff(q1)+c322*diff(q2)+c323*diff(q3)+c324*diff(q4)+c325*diff(q5)+c326*diff(q6);
c33= c331*diff(q1)+c332*diff(q2)+c333*diff(q3)+c334*diff(q4)+c335*diff(q5)+c336*diff(q6);
c34= c341*diff(q1)+c342*diff(q2)+c343*diff(q3)+c344*diff(q4)+c345*diff(q5)+c346*diff(q6);
c35= c351*diff(q1)+c352*diff(q2)+c353*diff(q3)+c354*diff(q4)+c355*diff(q5)+c356*diff(q6);
c36= c361*diff(q1)+c362*diff(q2)+c363*diff(q3)+c364*diff(q4)+c365*diff(q5)+c366*diff(q6);
c41= c411*diff(q1)+c412*diff(q2)+c413*diff(q3)+c414*diff(q4)+c415*diff(q5)+c416*diff(q6);
c42= c421*diff(q1)+c422*diff(q2)+c423*diff(q3)+c424*diff(q4)+c425*diff(q5)+c426*diff(q6);
c43= c431*diff(q1)+c432*diff(q2)+c433*diff(q3)+c434*diff(q4)+c435*diff(q5)+c436*diff(q6);
c44= c441*diff(q1)+c442*diff(q2)+c443*diff(q3)+c444*diff(q4)+c445*diff(q5)+c446*diff(q6);
c45= c451*diff(q1)+c452*diff(q2)+c453*diff(q3)+c454*diff(q4)+c455*diff(q5)+c456*diff(q6);
c46= c461*diff(q1)+c462*diff(q2)+c463*diff(q3)+c464*diff(q4)+c465*diff(q5)+c466*diff(q6);
c51= c511*diff(q1)+c512*diff(q2)+c513*diff(q3)+c514*diff(q4)+c515*diff(q5)+c516*diff(q6);
c52= c521*diff(q1)+c522*diff(q2)+c523*diff(q3)+c524*diff(q4)+c525*diff(q5)+c526*diff(q6);
c53= c531*diff(q1)+c532*diff(q2)+c533*diff(q3)+c534*diff(q4)+c535*diff(q5)+c536*diff(q6);
c54= c541*diff(q1)+c542*diff(q2)+c543*diff(q3)+c544*diff(q4)+c545*diff(q5)+c546*diff(q6);
c55= c551*diff(q1)+c552*diff(q2)+c553*diff(q3)+c554*diff(q4)+c555*diff(q5)+c556*diff(q6);
c56= c561*diff(q1)+c562*diff(q2)+c563*diff(q3)+c564*diff(q4)+c565*diff(q5)+c566*diff(q6);
c61= c611*diff(q1)+c612*diff(q2)+c613*diff(q3)+c614*diff(q4)+c615*diff(q5)+c616*diff(q6);
c62= c621*diff(q1)+c622*diff(q2)+c623*diff(q3)+c624*diff(q4)+c625*diff(q5)+c626*diff(q6);
c63= c631*diff(q1)+c632*diff(q2)+c633*diff(q3)+c634*diff(q4)+c635*diff(q5)+c636*diff(q6);
c64= c641*diff(q1)+c642*diff(q2)+c643*diff(q3)+c644*diff(q4)+c645*diff(q5)+c646*diff(q6);
c65= c651*diff(q1)+c652*diff(q2)+c653*diff(q3)+c654*diff(q4)+c655*diff(q5)+c656*diff(q6);
c66= c661*diff(q1)+c662*diff(q2)+c663*diff(q3)+c664*diff(q4)+c665*diff(q5)+c666*diff(q6);

Cq=[c11 c12 c13 c14 c15 c26;...
    c21 c22 c23 c24 c25 c26;...
    c31 c32 c33 c34 c35 c36;...
    c41 c42 c43 c44 c45 c46;...
    c51 c52 c53 c54 c55 c56;...
    c61 c62 c63 c64 c65 c66];

C=simplify(Cq)

%Gravity 
g0=[0;0;-g];

g1= m1*g0'*jv1(:,1)+m2*g0'*jv1(:,2)+m3*g0'*jv1(:,3)+m4*g0'*jv1(:,4)+m5*g0'*jv1(:,5)+m6*g0'*jv1(:,6);
g2= m1*g0'*jv2(:,1)+m2*g0'*jv2(:,2)+m3*g0'*jv2(:,3)+m4*g0'*jv2(:,4)+m5*g0'*jv2(:,5)+m6*g0'*jv2(:,6);
g3= m1*g0'*jv3(:,1)+m2*g0'*jv3(:,2)+m3*g0'*jv3(:,3)+m4*g0'*jv3(:,4)+m5*g0'*jv3(:,5)+m6*g0'*jv3(:,6);
g4= m1*g0'*jv4(:,1)+m2*g0'*jv4(:,2)+m3*g0'*jv4(:,3)+m4*g0'*jv4(:,4)+m5*g0'*jv4(:,5)+m6*g0'*jv4(:,6);
g5= m1*g0'*jv5(:,1)+m2*g0'*jv5(:,2)+m3*g0'*jv5(:,3)+m4*g0'*jv5(:,4)+m5*g0'*jv5(:,5)+m6*g0'*jv5(:,6);
g6= m1*g0'*jv6(:,1)+m2*g0'*jv6(:,2)+m3*g0'*jv6(:,3)+m4*g0'*jv6(:,4)+m5*g0'*jv6(:,5)+m6*g0'*jv6(:,6);

gq=[g1; g2; g3; g4; g5; g6];

g=simplify(gq)

ddq= [diff(diff(q1)); diff(diff(q2)); diff(diff(q3)); diff(diff(q4)); diff(diff(q5)); diff(diff(q6))];
dq = [diff(q1); diff(q2); diff(q3); diff(q4); diff(q5); diff(q6)];

tau= simplify(M*ddq + C*dq + g)
