clc;
clear;
Para;

%决策变量
Pg = sdpvar(1,Time);
Pgrid = sdpvar(1,Time);
Pw = sdpvar(1,Time);
Ppv = sdpvar(1,Time);
Pbat = sdpvar(1,Time);
SOC = sdpvar(1,Time+1);

%约束
cst = [];
%功率平衡约束
cst = [cst;Pg+Pgrid+Pw+Ppv+Pbat==Load];
%火电出力上下限约束
cst = [cst;Pgmin<=Pg<=Pgmax];
%火电爬坡约束
cst = [cst;-Rup<=Pg(:,2:end)-Pg(:,1:end-1)<=Rup];
%外电网出力上下限约束
cst = [cst;-Pgrid_max<=Pgrid<=Pgrid_max];
%风电出力上下限约束
cst = [cst;0<=Pw<=WT];
%光伏出力上下限约束
cst = [cst;0<=Ppv<=PV];
%电池出力上下限约束
cst = [cst;-Pbat_max<=Pbat<=Pbat_max];
%SOC上下限约束
cst = [cst;SOC(:,1)==E0];
cst = [cst;SOC(:,2:end)==SOC(:,1:end-1)-Pbat];
cst = [cst;Emin<=SOC<=Emax];
cst = [cst;SOC(:,end)==E0];

%目标函数
obj = sum(g_a*Pg.^2+g_b*Pg+g_c+Price.*Pgrid);

%求解
ops = sdpsettings('solver','gurobi','verbose',0);
x=solvesdp(cst,obj,ops);
Pg_IS = double(Pg);
Pgrid_IS = double(Pgrid);
Pw_IS = double(Pw);
Ppv_IS = double(Ppv);
Pbat_IS = double(Pbat);
SOC_IS = double(SOC);
Cost_IS = double(obj);


