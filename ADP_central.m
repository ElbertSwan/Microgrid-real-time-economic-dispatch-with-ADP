clc;
clear;
Para;
%状态变量
Pg_formal = sdpvar(1,1);
Price_opt = sdpvar(1,1);
Pw_opt = sdpvar(1,1);
Ppv_opt = sdpvar(1,1);
Load_opt = sdpvar(1,1);
SOC_opt = sdpvar(1,1);
Emin_opt = sdpvar(1,1);
Emax_opt = sdpvar(1,1);
slope_opt = sdpvar(1,1);%以分一段为例

%决策变量
Pg = sdpvar(1,1);
Pgrid = sdpvar(1,1);
Pw = sdpvar(1,1);
Ppv = sdpvar(1,1);
Pbat = sdpvar(1,1);
SOC = sdpvar(1,1);

%约束
cst = [];
%功率平衡约束
cst = [cst;Pg+Pgrid+Pw+Ppv+Pbat==Load_opt];
%火电出力上下限约束
cst = [cst;max(Pgmin,Pg_formal-Rup)<=Pg<=min(Pgmax,Pg_formal+Rup)];
%外电网出力上下限约束
cst = [cst;-Pgrid_max<=Pgrid<=Pgrid_max];
%风电出力上下限约束
cst = [cst;0<=Pw<=Pw_opt];
%光伏出力上下限约束
cst = [cst;0<=Ppv<=Ppv_opt];
%电池出力上下限约束
cst = [cst;-Pbat_max<=Pbat<=Pbat_max];
%SOC上下限约束
cst = [cst;SOC==SOC_opt-Pbat];
cst = [cst;Emin_opt<=SOC<=Emax_opt];

%目标函数
obj = g_a*Pg^2+g_b*Pg+g_c+Price_opt*Pgrid-slope_opt*SOC;

%求解
ops = sdpsettings('solver','gurobi','verbose',0);
ADP_controller = optimizer(cst,obj,ops,[Pg_formal,Price_opt,Pw_opt,Ppv_opt,Load_opt,SOC_opt,Emin_opt,Emax_opt,slope_opt],[Pg,Pgrid,Pw,Ppv,Pbat,SOC]);


%% ADP参数
N = 200;
stepsize = 0.1;
piece_num = 1;%离散的分段数
v_slope = zeros(Time+1,N);
v_slope_sample = zeros(Time,N);
cost_ADP = zeros(N,Time);
Cost_ADP = zeros(N,1);
Pg_ADP = zeros(N,Time);
Pgrid_ADP = zeros(N,Time);
Pw_ADP = zeros(N,Time);
Ppv_ADP = zeros(N,Time);
Pbat_ADP = zeros(N,Time);
SOC_ADP = zeros(N,Time+1);
SOC_ADP(:,1) = E0;
C_ADP = zeros(N,Time);
C_ADP2 = zeros(N,Time);
%% ADP优化求解
for n=2:N
    for t=1:Time
if t==1
    S_real = [     60        Price(t) WT(t)  PV(t)  Load(t)      E0        Emin_Myopic(t)    Emax_Myopic(t)   v_slope(t+1,n-1)];
else
    S_real = [Pg_ADP(n,t-1)  Price(t) WT(t)  PV(t)  Load(t)  SOC_ADP(n,t)  Emin_Myopic(t)    Emax_Myopic(t)   v_slope(t+1,n-1)];
end
[Solutions,Diagnostics] = ADP_controller(S_real);
Pg_ADP(n,t) =  Solutions(1);   
Pgrid_ADP(n,t) =  Solutions(2);       
Pw_ADP(n,t) =  Solutions(3);       
Ppv_ADP(n,t) =  Solutions(4);       
Pbat_ADP(n,t) =  Solutions(5);       
SOC_ADP(n,t+1) =  Solutions(6);       
C_ADP(n,t) = g_a*Pg_ADP(n,t)^2+g_b*Pg_ADP(n,t)+g_c+Price(t)*Pgrid_ADP(n,t)+v_slope(t+1,n-1)*SOC_ADP(n,t+1);
cost_ADP(n,t) = g_a*Pg_ADP(n,t)^2+g_b*Pg_ADP(n,t)+g_c+Price(t)*Pgrid_ADP(n,t);
%% 计算采样估计值
SOC_formal_posi = SOC_ADP(n,t)+0.001;
SOC_formal_nega = SOC_ADP(n,t)-0.001;
if SOC_formal_posi>Emax_Myopic(t)
    SOC_formal = SOC_formal_nega;
else
    SOC_formal = SOC_formal_posi;
end
%% 重新优化
if t==1
    S_real2 = [     60        Price(t) WT(t)  PV(t)  Load(t)    SOC_formal  Emin_Myopic(t)    Emax_Myopic(t)   v_slope(t+1,n-1)];
else
    S_real2 = [Pg_ADP(n,t-1)  Price(t) WT(t)  PV(t)  Load(t)    SOC_formal  Emin_Myopic(t)    Emax_Myopic(t)   v_slope(t+1,n-1)];
end
[Solutions2,Diagnostics2] = ADP_controller(S_real2);    
Pg_ADP2(n,t) = Solutions2(1);
Pgrid_ADP2(n,t) = Solutions2(2); 
Pbat_ADP2(n,t) =  Solutions2(5);
C_ADP2(n,t) = g_a*Solutions2(1)^2+g_b*Solutions2(1)+g_c+Price(t)*Solutions2(2)+v_slope(t+1,n-1)*Solutions2(6);
%% 斜率更新
v_slope_sample(t,n) = abs(C_ADP(n,t)-C_ADP2(n,t))/0.001;%计算斜率采样值
v_sample = abs(C_ADP(n,t)-C_ADP2(n,t))/0.001;
if stepsize~=0
    v_slope(t,n) = (1-stepsize)*v_slope(t,n-1)+stepsize*v_slope_sample(t,n);%这个记得，采样观测值更新的是上一个时刻的斜率
else
    v_slope(t,n) = (1-a/(a+n-1))*v_slope(t,n-1)+a/(a+n-1)*v_slope_sample(t,n);%这个记得，采样观测值更新的是上一个时刻的斜率
end
    end
Cost_ADP(n) = sum(cost_ADP(n,:));
n
end

Cum_reward = -Cost_ADP;
Cost_IS = 55.9129;

save ADP


