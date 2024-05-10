clc;
clear;
Para;

%���߱���
Pg = sdpvar(1,Time);
Pgrid = sdpvar(1,Time);
Pw = sdpvar(1,Time);
Ppv = sdpvar(1,Time);
Pbat = sdpvar(1,Time);
SOC = sdpvar(1,Time+1);

%Լ��
cst = [];
%����ƽ��Լ��
cst = [cst;Pg+Pgrid+Pw+Ppv+Pbat==Load];
%������������Լ��
cst = [cst;Pgmin<=Pg<=Pgmax];
%�������Լ��
cst = [cst;-Rup<=Pg(:,2:end)-Pg(:,1:end-1)<=Rup];
%���������������Լ��
cst = [cst;-Pgrid_max<=Pgrid<=Pgrid_max];
%������������Լ��
cst = [cst;0<=Pw<=WT];
%�������������Լ��
cst = [cst;0<=Ppv<=PV];
%��س���������Լ��
cst = [cst;-Pbat_max<=Pbat<=Pbat_max];
%SOC������Լ��
cst = [cst;SOC(:,1)==E0];
cst = [cst;SOC(:,2:end)==SOC(:,1:end-1)-Pbat];
cst = [cst;Emin<=SOC<=Emax];
cst = [cst;SOC(:,end)==E0];

%Ŀ�꺯��
obj = sum(g_a*Pg.^2+g_b*Pg+g_c+Price.*Pgrid);

%���
ops = sdpsettings('solver','gurobi','verbose',0);
x=solvesdp(cst,obj,ops);
Pg_IS = double(Pg);
Pgrid_IS = double(Pgrid);
Pw_IS = double(Pw);
Ppv_IS = double(Ppv);
Pbat_IS = double(Pbat);
SOC_IS = double(SOC);
Cost_IS = double(obj);


