Para;
%% 火电优化子问题建模
%状态变量
lanbuta_g_opt = sdpvar(1,1);
Pgmax_opt = sdpvar(1,1);
Pgmin_opt = sdpvar(1,1);
%决策变量
Pg_opt = sdpvar(1,1);
%约束
cst_g = [];
cst_g = [cst_g;Pgmin_opt<=Pg_opt<=Pgmax_opt];
%目标函数
obj_g = g_a*Pg_opt^2+g_b*Pg_opt+g_c-lanbuta_g_opt*Pg_opt;
%求解
ops_g = sdpsettings('solver','gurobi','verbose',0);
g_controller = optimizer(cst_g,obj_g,ops_g,[lanbuta_g_opt,Pgmax_opt,Pgmin_opt],[Pg_opt]);

%% 外电网优化子问题建模
%状态变量
lanbuta_grid_opt = sdpvar(1,1);
Price_opt = sdpvar(1,1);
Pgrid_max_opt = sdpvar(1,1);
%决策变量
Pgrid_opt = sdpvar(1,1);
%约束
cst_grid = [];
cst_grid = [cst_grid;-Pgrid_max_opt<=Pgrid_opt<=Pgrid_max_opt];
%目标函数
obj_grid = (Price_opt-lanbuta_grid_opt)*Pgrid_opt;
%求解
ops_grid = sdpsettings('solver','gurobi','verbose',0);
grid_controller = optimizer(cst_grid,obj_grid,ops_grid,[lanbuta_grid_opt,Price_opt,Pgrid_max_opt],[Pgrid_opt]);

%% 储能优化子问题建模
%状态变量
lanbuta_bat_opt = sdpvar(1,1);
Pbat_min_opt = sdpvar(1,1);
Pbat_max_opt = sdpvar(1,1);
slope_opt = sdpvar(1,1);
%决策变量
Pbat_opt = sdpvar(1,1);
%约束
cst_bat = [];
cst_bat = [cst_bat;Pbat_min_opt<=Pbat_opt<=Pbat_max_opt];
%目标函数
obj_bat = (slope_opt-lanbuta_bat_opt)*Pbat_opt;
%求解
ops_bat = sdpsettings('solver','gurobi','verbose',0);
bat_controller = optimizer(cst_bat,obj_bat,ops_bat,[lanbuta_bat_opt,Pbat_min_opt,Pbat_max_opt,slope_opt],[Pbat_opt]);

