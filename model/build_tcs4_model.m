function build_tcs4_model(outdir)
%BUILD_TCS4_MODEL  自动生成“四轮独立”版 TCS-PID 仿真模型 TCS4_Top.slx
%
%   build_tcs4_model()          % 生成到本文件所在目录
%   build_tcs4_model(outdir)    % 生成到指定目录
%
%   和旧版 TCS_PID_Top 的区别（答辩时可以讲的三点升级）：
%     1. 被控对象从“两根轴”细化成“四个车轮”，可以看每个轮子的滑移率；
%     2. 控制器的车手扭矩来源改成“油门开度 x 转速”二维查表，前/后轴各一条曲线
%        （因为前后减速比不同，同一路面下可用扭矩曲线天然错开，不是人为造两条表）；
%     3. 参考车速由“四轮取小 + 低通滤波”估计得到（无 GPS 的真实方案），
%        同时保留“仿真真值”通道，只用来验证估计得准不准。
%
%   模型分层：
%     DRV_油门踏板曲线     车手踏板开度随时间变化（1-D 查表）
%     ROAD_路面附着曲线    路面附着系数随时间变化（1-D 查表，可做阶跃路面）
%     TCS_控制器           滑移率估计 -> 介入判断 -> 平滑 -> PID -> 扭矩仲裁分配
%     VEH_被控对象         四轮各自的“电机外特性 -> 轮胎 -> 车轮” + 车身纵向动力学
%
%   所有参数都来自 tcs4_params.m 的结构体 p，模块里写的是 p.xxx。

if nargin < 1 || isempty(outdir)
    outdir = fileparts(mfilename('fullpath'));
end
pd = tcs4_params();   % 只为取默认参数值写进注释文字，模型里仍然用 p.xxx
mdl = 'TCS4_Top';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(fullfile(outdir,[mdl '.slx']),'file'), delete(fullfile(outdir,[mdl '.slx'])); end

new_system(mdl);
load_system('simulink');
set_param(mdl,'Description', sprintf([ ...
    'FSEC 四电机赛车 TCS-PID 仿真（四轮版）：\n' ...
    '输入 = 油门开度 + 四轮转速（轮速来自电机编码器/减速比）\n' ...
    '参考车速 = 四轮取小估计（无 GPS 方案），滑移率闭环，PID 只减不加扭矩']));

%% 1) 建容器 ---------------------------------------------------------------
mk(mdl,'CLK_Time','simulink/Sources/Clock',[30 500 60 520]);
mkSub(mdl,'DRV_油门踏板曲线',[100 60 280 180]);
mkSub(mdl,'ROAD_路面附着曲线',[100 320 280 440]);
mkSub(mdl,'TCS_控制器',[400 40 640 560]);
mkSub(mdl,'VEH_被控对象',[760 40 980 560]);

c = [mdl '/TCS_控制器'];
mkSub(c,'1_参考车速估计',[60 30 260 150]);
mkSub(c,'2_滑移率计算',[60 200 260 420]);
mkSub(c,'3_介入退出判断',[320 30 500 160]);
mkSub(c,'4_介入平滑',[320 210 470 290]);
mkSub(c,'5a_油门扭矩查表_前轴',[320 340 500 430]);
mkSub(c,'5b_油门扭矩查表_后轴',[320 470 500 560]);
mkSub(c,'6a_PID扭矩补偿_前轴',[560 30 760 190]);
mkSub(c,'6b_PID扭矩补偿_后轴',[560 230 760 390]);
mkSub(c,'7_扭矩仲裁与四轮分配',[820 30 1020 300]);

v = [mdl '/VEH_被控对象'];
mkSub(v,'车轮_左前',[60 30 260 190]);
mkSub(v,'车轮_右前',[60 230 260 390]);
mkSub(v,'车轮_左后',[320 30 520 190]);
mkSub(v,'车轮_右后',[320 230 520 390]);
mkSub(v,'车身纵向动力学',[580 30 800 320]);

%% 2) 填叶子子系统 ---------------------------------------------------------
fill_pedal(mdl);
fill_road(mdl);
fill_vref([c '/1_参考车速估计']);
fill_slip([c '/2_滑移率计算']);
fill_enable([c '/3_介入退出判断']);
fill_smooth([c '/4_介入平滑']);
fill_pedal_map([c '/5a_油门扭矩查表_前轴'],'f');
fill_pedal_map([c '/5b_油门扭矩查表_后轴'],'r');
fill_pid([c '/6a_PID扭矩补偿_前轴']);
fill_pid([c '/6b_PID扭矩补偿_后轴']);
fill_arbitration([c '/7_扭矩仲裁与四轮分配']);
for w = {'fl','fr','rl','rr'}
    fill_wheel([v '/车轮_' wheel_cn(w{1})], w{1});
end
fill_body([v '/车身纵向动力学']);

%% 3) 连线 -----------------------------------------------------------------
wire_controller(c);
wire_plant(v);
wire_top(mdl);

%% 4) 仿真设置、排版、保存 -------------------------------------------------
set_param(mdl,'SolverType','Fixed-step','Solver','ode4','FixedStep','p.Ts_plant', ...
    'StartTime','0','StopTime','p.t_end','ReturnWorkspaceOutputs','on');
add_note(mdl,[100 620], sprintf([ ...
    'TCS 四轮版仿真模型  |  参数唯一来源：tcs4_params.m 的结构体 p\n' ...
    '运行：p = tcs4_params(); assignin(''base'',''p'',p); sim(''TCS4_Top'')  或直接跑 run_tcs4_sim\n' ...
    '控制器 50 Hz（Ts=20ms，与实车 VCU 一致）；被控对象 0.5 ms 固定步长 ode4\n' ...
    '信号颜色：控制器输出 4 台电机扭矩，被控对象返回 4 个轮速 + 车身真实车速(仅对照用)']));
% 排版说明：本脚本不做自动排版（大模型上 arrangeSystem 很慢）。
% 需要整洁布局时，在 Simulink 里打开子系统 -> 全选(Ctrl+A) -> 工具条“自动排列”即可。
% 每个叶子子系统内部的模块位置在下面 mk(...,'Position',...) 里已手工排好。
save_system(mdl, fullfile(outdir,[mdl '.slx']));
fprintf('模型已生成: %s\n', fullfile(outdir,[mdl '.slx']));
end

%% ======================================================================
%%  叶子子系统：输入侧
%% ======================================================================
function fill_pedal(mdl)
% 车手油门开度曲线：时间 -> 踏板开度 0~1（模拟车手踩油门的过程）
s = [mdl '/DRV_油门踏板曲线'];
in(s,1,'时间',[20 60 50 80]);
lookup(s,'开度随时间','p.drv_t','p.drv_pedal','Linear point-slope',[100 50 190 90]);
out(s,1,'油门开度',[310 60 340 80]);
ln(s,'时间/1','开度随时间/1');
ln(s,'开度随时间/1','油门开度/1');
add_note(s,[20 130],'车手踏板 0~1，1.1 秒内从 0 踩到 100%，之后保持全开');
end

function fill_road(mdl)
% 路面附着曲线：时间 -> 路面附着系数 mu（Flat = 零阶保持，可做“干地->湿地”阶跃）
s = [mdl '/ROAD_路面附着曲线'];
in(s,1,'时间',[20 60 50 80]);
lookup(s,'附着随时间','p.mu_t','p.mu_v','Flat',[100 50 190 90]);
out(s,1,'路面附着',[230 60 260 80]);
ln(s,'时间/1','附着随时间/1');
ln(s,'附着随时间/1','路面附着/1');
add_note(s,[20 130],'mu=1.4 干沥青 / 0.9 湿沥青粗糙 / 0.5 湿滑；Flat 插值 = 阶跃路面');
end

%% ======================================================================
%%  TCS 控制器各模块
%% ======================================================================
function fill_vref(s)
% 1_参考车速估计：四轮轮速 x 半径 -> 取最小 -> 低通滤波
% 为什么取最小：加速时每个驱动轮的轮速都 >= 真实车速（打滑只会让轮速偏大），
% 所以“最慢的那个轮子”最接近真实车速；再低通滤波抑制信号噪声。
% 模式开关：1=四轮取小估计(实车方案)  2=仿真真值(仅用于验证估计误差)
in(s,1,'轮速_左前',[15 40 45 60]);
in(s,2,'轮速_右前',[15 90 45 110]);
in(s,3,'轮速_左后',[15 140 45 160]);
in(s,4,'轮速_右后',[15 190 45 210]);
in(s,5,'真值车速',[15 240 45 260]);

wheels = {'轮速_左前','轮速_右前','轮速_左后','轮速_右后'};
for k = 1:4
    mk(s,['增益半径_' num2str(k)],'simulink/Math Operations/Gain',[80 30+(k-1)*50 110 60+(k-1)*50], ...
        'Gain','p.r_w');
    ln(s,[wheels{k} '/1'],['增益半径_' num2str(k) '/1']);
end
% 取小_1 = min(左前线速度, 右前线速度)；取小_2 = min(左后, 右后)；取小_3 = min(前轴, 后轴)
mk(s,'取小_1','simulink/Math Operations/MinMax',[140 40 175 90],'Function','min','Inputs','2');
mk(s,'取小_2','simulink/Math Operations/MinMax',[140 140 175 190],'Function','min','Inputs','2');
mk(s,'取小_3','simulink/Math Operations/MinMax',[200 90 235 140],'Function','min','Inputs','2');
ln(s,'增益半径_1/1','取小_1/1');
ln(s,'增益半径_2/1','取小_1/2');
ln(s,'增益半径_3/1','取小_2/1');
ln(s,'增益半径_4/1','取小_2/2');
ln(s,'取小_1/1','取小_3/1');
ln(s,'取小_2/1','取小_3/2');
mk(s,'模式开关','simulink/Signal Routing/Multiport Switch',[260 60 300 200],'Inputs','2');
mk(s,'模式常数','simulink/Sources/Constant',[260 220 310 240],'Value','p.speed_ref_mode');
mk(s,'估计低通','simulink/Discrete/Discrete Filter',[330 100 410 140], ...
    'Numerator','[p.alpha_ref 0]','Denominator','[1 -(1-p.alpha_ref)]', ...
    'SampleTime','p.Ts','InitialStates','0');
out(s,1,'参考车速',[440 110 470 130]);
ln(s,'取小_3/1','模式开关/2');
ln(s,'真值车速/1','模式开关/3');
ln(s,'模式常数/1','模式开关/1');
ln(s,'模式开关/1','估计低通/1');
ln(s,'估计低通/1','参考车速/1');
add_note(s,[15 300], sprintf([ ...
    'v_ref = min(四轮轮速) x r_w，再经一阶低通(tau_ref=0.05s)\n' ...
    '加速时轮速只会偏大，最慢轮最接近真值；滤波去噪声\n' ...
    '模式 2 接的是仿真真值，仅用于验证估计精度，实车代码只有模式 1']));
end

function fill_slip(s)
% 2_滑移率计算：每个轮子 lam = (轮速x半径 - 参考车速) / max(参考车速, 下限)
% 分母加下限是防止低速时除以接近 0 的数把滑移率算爆；算完低通滤波去噪声。
names = {'左前','右前','左后','右后'};
for k = 1:4
    y0 = 20 + (k-1)*90;
    in(s,k,['轮速_' names{k}],[15 y0 45 y0+20]);
    mk(s,['半径_' names{k}],'simulink/Math Operations/Gain',[80 y0 110 y0+30],'Gain','p.r_w');
    mk(s,['相减_' names{k}],'simulink/Math Operations/Sum',[140 y0 165 y0+30],'Inputs','+-');
    mk(s,['车速下限_' names{k}],'simulink/Sources/Constant',[80 y0+50 135 y0+70],'Value','p.v_floor');
    mk(s,['保底_' names{k}],'simulink/Math Operations/MinMax',[140 y0+45 180 y0+75], ...
        'Function','max','Inputs','2');
    mk(s,['相除_' names{k}],'simulink/Math Operations/Divide',[210 y0 240 y0+30]);
    mk(s,['滤波_' names{k}],'simulink/Discrete/Discrete Filter',[265 y0-5 340 y0+35], ...
        'Numerator','[p.alpha_slip 0]','Denominator','[1 -(1-p.alpha_slip)]', ...
        'SampleTime','p.Ts','InitialStates','0');
    out(s,k,['滑移率_' names{k}],[380 y0 410 y0+20]);
    ln(s,['轮速_' names{k} '/1'],['半径_' names{k} '/1']);
    ln(s,['半径_' names{k} '/1'],['相减_' names{k} '/1']);
    ln(s,['相减_' names{k} '/1'],['相除_' names{k} '/1']);
    ln(s,['车速下限_' names{k} '/1'],['保底_' names{k} '/1']);
    ln(s,['保底_' names{k} '/1'],['相除_' names{k} '/2']);
    ln(s,['相除_' names{k} '/1'],['滤波_' names{k} '/1']);
    ln(s,['滤波_' names{k} '/1'],['滑移率_' names{k} '/1']);
end
% 参考车速接到每个“相减/保底”的第二输入
in(s,5,'参考车速',[15 400 45 420]);
for k = 1:4
    ln(s,'参考车速/1',['相减_' names{k} '/2']);
    ln(s,'参考车速/1',['保底_' names{k} '/2']);
end
% 控制用滑移率：前轴取大、后轴取大、整车取大
mk(s,'前轴取大','simulink/Math Operations/MinMax',[380 60 420 110],'Function','max','Inputs','2');
mk(s,'后轴取大','simulink/Math Operations/MinMax',[380 240 420 290],'Function','max','Inputs','2');
mk(s,'整车取大','simulink/Math Operations/MinMax',[450 150 490 200],'Function','max','Inputs','2');
out(s,5,'滑移率_前轴',[540 75 570 95]);
out(s,6,'滑移率_后轴',[540 255 570 275]);
out(s,7,'滑移率_控制用',[540 165 580 185]);
ln(s,'滤波_左前/1','前轴取大/1');
ln(s,'滤波_右前/1','前轴取大/2');
ln(s,'滤波_左后/1','后轴取大/1');
ln(s,'滤波_右后/1','后轴取大/2');
ln(s,'前轴取大/1','整车取大/1');
ln(s,'后轴取大/1','整车取大/2');
ln(s,'前轴取大/1','滑移率_前轴/1');
ln(s,'后轴取大/1','滑移率_后轴/1');
ln(s,'整车取大/1','滑移率_控制用/1');
add_note(s,[15 480], sprintf([ ...
    'lam_i = (w_i*r - v_ref) / max(v_ref, v_floor)，一阶低通 tau_slip=0.03s\n' ...
    '控制用滑移率 = max(四轮)：只要有一个轮子打滑就要管\n' ...
    'PID 按轴控制：前轴用 max(左前,右前)，后轴用 max(左后,右后)']));
end

function fill_enable(s)
pd = tcs4_params();
% 3_介入退出判断：带滞环的状态机，防止在阈值附近反复开关
%   介入条件：滑移率 >= 介入阈值 且 车速 >= 车速门限
%   退出条件：滑移率 <  退出阈值 或  车速 <  车速门限
%   保持：用上一拍的使能信号维持（RS 锁存思想）
in(s,1,'滑移率_控制用',[15 40 45 60]);
in(s,2,'参考车速',[15 120 45 140]);
mk(s,'常数_介入阈值','simulink/Sources/Constant',[70 20 130 40],'Value','p.lam_on');
mk(s,'比较_介入','simulink/Logic and Bit Operations/Relational Operator',[160 15 190 45],'Operator','>=');
mk(s,'常数_车速门限1','simulink/Sources/Constant',[70 70 130 90],'Value','p.v_min');
mk(s,'比较_车速够','simulink/Logic and Bit Operations/Relational Operator',[160 65 190 95],'Operator','>=');
mk(s,'介入与','simulink/Logic and Bit Operations/Logical Operator',[220 25 255 70],'Operator','AND','Inputs','2');
mk(s,'常数_退出阈值','simulink/Sources/Constant',[70 150 130 170],'Value','p.lam_off');
mk(s,'比较_退出','simulink/Logic and Bit Operations/Relational Operator',[160 145 190 175],'Operator','<');
mk(s,'常数_车速门限2','simulink/Sources/Constant',[70 210 130 230],'Value','p.v_min');
mk(s,'比较_车速低','simulink/Logic and Bit Operations/Relational Operator',[160 205 190 235],'Operator','<');
mk(s,'退出或','simulink/Logic and Bit Operations/Logical Operator',[220 155 255 200],'Operator','OR','Inputs','2');
mk(s,'退出取反','simulink/Logic and Bit Operations/Logical Operator',[275 160 305 190],'Operator','NOT');
mk(s,'保持延时','simulink/Discrete/Unit Delay',[275 60 305 90],'SampleTime','p.Ts','InitialCondition','0');
mk(s,'保持与','simulink/Logic and Bit Operations/Logical Operator',[330 60 365 110],'Operator','AND','Inputs','2');
mk(s,'使能或','simulink/Logic and Bit Operations/Logical Operator',[390 30 425 110],'Operator','OR','Inputs','2');
out(s,1,'使能',[460 60 490 80]);
ln(s,'滑移率_控制用/1','比较_介入/1');
ln(s,'常数_介入阈值/1','比较_介入/2');
ln(s,'参考车速/1','比较_车速够/1');
ln(s,'常数_车速门限1/1','比较_车速够/2');
ln(s,'比较_介入/1','介入与/1');
ln(s,'比较_车速够/1','介入与/2');
ln(s,'滑移率_控制用/1','比较_退出/1');
ln(s,'常数_退出阈值/1','比较_退出/2');
ln(s,'参考车速/1','比较_车速低/1');
ln(s,'常数_车速门限2/1','比较_车速低/2');
ln(s,'比较_退出/1','退出或/1');
ln(s,'比较_车速低/1','退出或/2');
ln(s,'退出或/1','退出取反/1');
ln(s,'退出取反/1','保持与/2');
ln(s,'保持延时/1','保持与/1');
ln(s,'介入与/1','使能或/1');
ln(s,'保持与/1','使能或/2');
ln(s,'使能或/1','使能/1');
ln(s,'使能或/1','保持延时/1');
add_note(s,[20 270], sprintf([ ...
    '介入：lam >= %.2f 且 v >= %.2f m/s\n退出：lam < %.2f 或 v < %.2f m/s\n' ...
    '滞环（介入阈值 > 退出阈值）防止在阈值附近反复开关'], ...
    pd.lam_on, pd.v_min, pd.lam_off, pd.v_min));
end

function fill_smooth(s)
% 4_介入平滑：使能信号 0/1 跳变 -> 0~1 连续过渡，避免补偿扭矩一上来就是阶跃
in(s,1,'使能',[20 40 50 60]);
mk(s,'转双精度','simulink/Signal Attributes/Data Type Conversion',[80 35 110 65], ...
    'OutDataTypeStr','double');
mk(s,'平滑低通','simulink/Discrete/Discrete Filter',[140 30 220 70], ...
    'Numerator','[p.alpha_en 0]','Denominator','[1 -(1-p.alpha_en)]', ...
    'SampleTime','p.Ts','InitialStates','0');
mk(s,'限幅01','simulink/Discontinuities/Saturation',[250 35 290 65], ...
    'UpperLimit','1','LowerLimit','0');
out(s,1,'使能平滑',[320 40 350 60]);
ln(s,'使能/1','转双精度/1');
ln(s,'转双精度/1','平滑低通/1');
ln(s,'平滑低通/1','限幅01/1');
ln(s,'限幅01/1','使能平滑/1');
add_note(s,[20 100],'一阶惯性 tau_en=0.04s：0/1 跳变 -> 0~1 平滑过渡');
end

function fill_pedal_map(s,ax)
pd = tcs4_params();
% 5_油门扭矩查表（前/后轴）：油门开度 x 车轮转速 -> 单电机车手需求扭矩
% 二维查表：行 = 油门开度 [0 0.2 0.5 0.8 1.0]，列 = 车轮转速 [rpm]
% 表内容 = 油门开度 x 电机外特性(基速前恒扭矩 21N*m，基速后恒功率段下降)
% 前/后轴减速比不同 -> 车轮转速断点不同 -> 两条曲线天然错开：
%   同一车速下，后轴(i=13.2)电机转速更高，更早进入恒功率降扭区。
if ax == 'f', tag = '前轴'; i_g = 'p.i_f'; i_val = pd.i_f; else, tag = '后轴'; i_g = 'p.i_r'; i_val = pd.i_r; end
in(s,1,'油门开度',[15 40 45 60]);
in(s,2,['轮速_左' tag],[15 100 45 120]);
in(s,3,['轮速_右' tag],[15 160 45 180]);
mk(s,['转速求和_' tag],'simulink/Math Operations/Sum',[80 110 105 150],'Inputs','++');
mk(s,['转速减半_' tag],'simulink/Math Operations/Gain',[130 110 165 150],'Gain','0.5');
mk(s,['转每分_' tag],'simulink/Math Operations/Gain',[190 110 240 150],'Gain','60/(2*pi)');
mk(s,['扭矩查表_' tag],'simulink/Lookup Tables/2-D Lookup Table',[230 40 330 120], ...
    'BreakpointsForDimension1','p.pedal_bp', ...
    'BreakpointsForDimension2',sprintf('p.w_bp_%s',ax), ...
    'Table','p.T_map','InterpMethod','Linear point-slope','ExtrapMethod','Clip');
out(s,1,['车手扭矩_' tag],[360 70 400 90]);
ln(s,'油门开度/1',['扭矩查表_' tag '/1']);
ln(s,['轮速_左' tag '/1'],['转速求和_' tag '/1']);
ln(s,['轮速_右' tag '/1'],['转速求和_' tag '/2']);
ln(s,['转速求和_' tag '/1'],['转速减半_' tag '/1']);
ln(s,['转速减半_' tag '/1'],['转每分_' tag '/1']);
ln(s,['转每分_' tag '/1'],['扭矩查表_' tag '/2']);
ln(s,['扭矩查表_' tag '/1'],['车手扭矩_' tag '/1']);
add_note(s,[15 220], sprintf([ ...
    'T_drv = 查表(油门开度, 车轮转速)，查表内容 = 油门 x 电机外特性\n' ...
    '电机外特性：0~%d rpm 恒 %.1f N*m，之后恒功率段降到 %.1f N*m @ %d rpm\n' ...
    '本表是 %s（减速比 %.1f），断点已换算到车轮转速'], ...
    pd.n_base, pd.T_mot_max, pd.T_at_maxn, pd.n_max, tag, i_val));
end

function fill_pid(s)
pd = tcs4_params();
% 6_PID 扭矩补偿：滑移率误差 -> PID -> 补偿扭矩（要被减掉的那部分）
%   e = lam_axle - lam_ref（超过目标为正）-> 输出正补偿 -> 从车手扭矩里扣
%   P 快速响应；I 消除稳态误差（带条件积分抗饱和）；D 默认 0（轮速噪声敏感）
in(s,1,'滑移率_轴',[15 20 45 40]);
in(s,2,'车手扭矩_轴',[15 250 45 270]);
in(s,3,'复位',[15 330 45 350]);
mk(s,'常数_目标滑移率','simulink/Sources/Constant',[70 120 140 140],'Value','p.lam_ref');
mk(s,'误差','simulink/Math Operations/Sum',[170 20 195 60],'Inputs','+-');
mk(s,'Kp_比例','simulink/Math Operations/Gain',[290 15 330 55],'Gain','p.Kp');
mk(s,'积分门控','simulink/Signal Routing/Switch',[250 90 285 130], ...
    'Criteria','u2 > Threshold','Threshold','0.5');
mk(s,'常数_零1','simulink/Sources/Constant',[180 85 215 105],'Value','0');
mk(s,'Ki_积分','simulink/Math Operations/Gain',[310 90 350 130],'Gain','p.Ki');
mk(s,'积分器','simulink/Discrete/Discrete-Time Integrator',[380 85 420 135], ...
    'IntegratorMethod','Integration: Forward Euler','gainval','1', ...
    'SampleTime','p.Ts','ExternalReset','rising','InitialCondition','0', ...
    'LimitOutput','on','UpperSaturationLimit','p.comp_abs','LowerSaturationLimit','-p.comp_abs');
mk(s,'误差延时','simulink/Discrete/Unit Delay',[250 180 280 210],'SampleTime','p.Ts','InitialCondition','0');
mk(s,'差分','simulink/Math Operations/Sum',[310 170 335 210],'Inputs','+-');
mk(s,'Kd除Ts','simulink/Math Operations/Gain',[360 175 410 205],'Gain','p.Kd/p.Ts');
mk(s,'微分滤波','simulink/Discrete/Discrete Filter',[440 170 510 210], ...
    'Numerator','[p.alpha_d 0]','Denominator','[1 -(1-p.alpha_d)]', ...
    'SampleTime','p.Ts','InitialStates','0');
mk(s,'PID求和','simulink/Math Operations/Sum',[540 60 565 140],'Inputs','+++');
mk(s,'动态上限','simulink/Math Operations/Gain',[440 300 480 330],'Gain','p.comp_ratio');
mk(s,'动态下限','simulink/Math Operations/Gain',[440 250 480 280],'Gain','-p.comp_ratio');
mk(s,'动态限幅','simulink/Discontinuities/Saturation Dynamic',[560 245 600 315]);
mk(s,'取绝对值','simulink/Math Operations/Abs',[600 60 630 90]);
mk(s,'是否饱和','simulink/Logic and Bit Operations/Relational Operator',[650 60 680 90],'Operator','>=');
mk(s,'常数_零2','simulink/Sources/Constant',[600 160 640 180],'Value','0');
mk(s,'符号乘','simulink/Math Operations/Product',[600 200 630 240],'Inputs','2');
mk(s,'同号为正','simulink/Logic and Bit Operations/Relational Operator',[650 205 680 235],'Operator','>');
mk(s,'饱和且同向','simulink/Logic and Bit Operations/Logical Operator',[700 110 730 160],'Operator','AND','Inputs','2');
out(s,1,'补偿扭矩',[760 150 790 170]);
ln(s,'常数_目标滑移率/1','误差/2');
ln(s,'滑移率_轴/1','误差/1');
ln(s,'误差/1','Kp_比例/1');
ln(s,'误差/1','积分门控/3');
ln(s,'常数_零1/1','积分门控/1');
ln(s,'积分门控/1','Ki_积分/1');
ln(s,'Ki_积分/1','积分器/1');
ln(s,'复位/1','积分器/2');
ln(s,'误差/1','误差延时/1');
ln(s,'误差延时/1','差分/2');
ln(s,'误差/1','差分/1');
ln(s,'差分/1','Kd除Ts/1');
ln(s,'Kd除Ts/1','微分滤波/1');
ln(s,'Kp_比例/1','PID求和/1');
ln(s,'积分器/1','PID求和/2');
ln(s,'微分滤波/1','PID求和/3');
ln(s,'车手扭矩_轴/1','动态上限/1');
ln(s,'车手扭矩_轴/1','动态下限/1');
ln(s,'PID求和/1','动态限幅/1');
ln(s,'动态上限/1','动态限幅/2');
ln(s,'动态下限/1','动态限幅/3');
ln(s,'动态限幅/1','补偿扭矩/1');
ln(s,'PID求和/1','取绝对值/1');
ln(s,'取绝对值/1','是否饱和/1');
ln(s,'动态上限/1','是否饱和/2');
% 抗饱和：只有“已经饱和 且 误差还在往饱和方向推”才冻结积分；误差反号时允许退饱和
ln(s,'PID求和/1','符号乘/1');
ln(s,'误差/1','符号乘/2');
ln(s,'符号乘/1','同号为正/1');
ln(s,'常数_零2/1','同号为正/2');
ln(s,'是否饱和/1','饱和且同向/1');
ln(s,'同号为正/1','饱和且同向/2');
ln(s,'饱和且同向/1','积分门控/2');
add_note(s,[70 390], sprintf([ ...
    'e = lam_axle - %.2f（过滑移为正）-> 输出要减掉的扭矩\n' ...
    'P=%.0f：误差 0.1 就减 %.0f N*m，快\nI=%.0f：把稳态误差磨到 0\n' ...
    'D=%.0f：不用——轮速信号有噪声，微分会放大噪声\n' ...
    '抗饱和(Clamping)：输出已饱和且误差还同向时，冻结积分器'], ...
    pd.lam_ref, pd.Kp, pd.Kp*0.1, pd.Ki, pd.Kd));
end

function fill_arbitration(s)
pd = tcs4_params();
% 7_扭矩仲裁与四轮分配：T_cmd = 车手扭矩 - 使能x补偿，限幅[0, 车手扭矩]，
%   再变化率限制 + 一阶平滑（模拟电机扭矩响应），最后左右轮各分一半。
in(s,1,'车手扭矩_前',[15 30 45 50]);
in(s,2,'车手扭矩_后',[15 90 45 110]);
in(s,3,'补偿扭矩_前',[15 150 45 170]);
in(s,4,'补偿扭矩_后',[15 210 45 230]);
in(s,5,'使能平滑',[15 270 45 290]);

axles = {'前','后'};
comp_in = {'补偿扭矩_前','补偿扭矩_后'};
drv_in  = {'车手扭矩_前','车手扭矩_后'};
out_name = {'Tcmd_前','Tcmd_后'};
for k = 1:2
    y0 = 30 + (k-1)*130;
    mk(s,['乘使能_' axles{k}],'simulink/Math Operations/Product',[90 y0 120 y0+40],'Inputs','2');
    mk(s,['总开关_' axles{k}],'simulink/Math Operations/Gain',[140 y0 180 y0+40],'Gain','p.TCS_enable');
    mk(s,['相减_' axles{k}],'simulink/Math Operations/Sum',[220 y0 245 y0+40],'Inputs','+-');
    mk(s,['常数_零_' axles{k}],'simulink/Sources/Constant',[220 y0+60 260 y0+80],'Value','0');
    mk(s,['下限0_' axles{k}],'simulink/Math Operations/MinMax',[280 y0 315 y0+40], ...
        'Function','max','Inputs','2');
    mk(s,['上限_车手_' axles{k}],'simulink/Math Operations/MinMax',[340 y0 375 y0+40], ...
        'Function','min','Inputs','2');
    mk(s,['变化率限制_' axles{k}],'simulink/Discontinuities/Rate Limiter',[400 y0 445 y0+40], ...
        'RisingSlewLimit','p.tq_rate','FallingSlewLimit','-p.tq_rate', ...
        'SampleTimeMode','inherited','InitialCondition','0');
    mk(s,['扭矩平滑_' axles{k}],'simulink/Discrete/Discrete Filter',[470 y0 545 y0+40], ...
        'Numerator','[p.alpha_tq 0]','Denominator','[1 -(1-p.alpha_tq)]', ...
        'SampleTime','p.Ts','InitialStates','0');
    out(s,k,out_name{k},[580 y0 620 y0+20]);
    ln(s,[comp_in{k} '/1'],['乘使能_' axles{k} '/1']);
    ln(s,'使能平滑/1',['乘使能_' axles{k} '/2']);
    ln(s,['乘使能_' axles{k} '/1'],['总开关_' axles{k} '/1']);
    ln(s,[drv_in{k} '/1'],['相减_' axles{k} '/1']);
    ln(s,['总开关_' axles{k} '/1'],['相减_' axles{k} '/2']);
    ln(s,['相减_' axles{k} '/1'],['下限0_' axles{k} '/1']);
    ln(s,['常数_零_' axles{k} '/1'],['下限0_' axles{k} '/2']);
    ln(s,['下限0_' axles{k} '/1'],['上限_车手_' axles{k} '/1']);
    ln(s,[drv_in{k} '/1'],['上限_车手_' axles{k} '/2']);
    ln(s,['上限_车手_' axles{k} '/1'],['变化率限制_' axles{k} '/1']);
    ln(s,['变化率限制_' axles{k} '/1'],['扭矩平滑_' axles{k} '/1']);
    ln(s,['扭矩平滑_' axles{k} '/1'],[out_name{k} '/1']);
end
% 同轴左右轮使用相同指令（查表值已是“单电机”扭矩，无需再分）；左右独立差扭是下一赛季扩展
mk(s,'同指令_前','simulink/Math Operations/Gain',[640 30 680 60],'Gain','1');
mk(s,'同指令_后','simulink/Math Operations/Gain',[640 160 680 190],'Gain','1');
out(s,3,'电机扭矩_左前',[720 30 760 50]);
out(s,4,'电机扭矩_右前',[720 80 760 100]);
out(s,5,'电机扭矩_左后',[720 160 760 180]);
out(s,6,'电机扭矩_右后',[720 210 760 230]);
ln(s,'扭矩平滑_前/1','同指令_前/1');
ln(s,'扭矩平滑_后/1','同指令_后/1');
ln(s,'同指令_前/1','电机扭矩_左前/1');
ln(s,'同指令_前/1','电机扭矩_右前/1');
ln(s,'同指令_后/1','电机扭矩_左后/1');
ln(s,'同指令_后/1','电机扭矩_右后/1');
add_note(s,[15 380], sprintf([ ...
    'T_cmd = T_drv - en_s * comp * TCS_enable，限制在 [0, T_drv]\n' ...
    '上限 = 车手扭矩：TCS 只能减扭矩，永远不会超过车手请求（人机对抗保护）\n' ...
    '变化率限制 %.0f N*m/s + 一阶平滑 tau=%.2fs（模拟电机扭矩响应）\n' ...
    '同轴左右轮各分一半；左右独立差扭控制留作下一赛季扩展'], pd.tq_rate, pd.tau_tq));
end

%% ======================================================================
%%  被控对象各模块
%% ======================================================================
function cn = wheel_cn(code)
switch code
    case 'fl', cn = '左前'; case 'fr', cn = '右前';
    case 'rl', cn = '左后'; otherwise, cn = '右后';
end
end

function fill_wheel(s,corner)
pd = tcs4_params();
% 车轮角落模型（每个轮子一个，四轮互相独立）：
%   1) 电机能力限幅：min(指令扭矩, 该转速下电机外特性可用扭矩)
%   2) 减速器放大：T_wheel = T_motor x 减速比 x 传动效率
%   3) 轮胎：lam = (w*r - v)/max(v,v_floor) -> F = mu(lam) x Fz
%   4) 车轮转动：J*dw/dt = T_wheel - F*r
switch corner
    case 'fl', J = 'p.J_fl'; i_g = 'p.i_f'; cn = '左前';
    case 'fr', J = 'p.J_fr'; i_g = 'p.i_f'; cn = '右前';
    case 'rl', J = 'p.J_rl'; i_g = 'p.i_r'; cn = '左后';
    otherwise, J = 'p.J_rr'; i_g = 'p.i_r'; cn = '右后';
end
in(s,1,'电机扭矩',[15 30 45 50]);
in(s,2,'车速',[15 120 45 140]);
in(s,3,'轮上载荷',[15 300 45 320]);
in(s,4,'路面附着',[15 380 45 400]);
% 电机外特性限幅（以电机转速为横轴；转速延时一拍，断开“限幅<->转速”代数环）
mk(s,'电机转速','simulink/Math Operations/Gain',[80 20 130 50],'Gain',sprintf('%s*60/(2*pi)',i_g));
mk(s,'转速延时','simulink/Discrete/Unit Delay',[140 20 170 50], ...
    'SampleTime','p.Ts_plant','InitialCondition','0');
mk(s,'可用扭矩','simulink/Lookup Tables/1-D Lookup Table',[200 15 290 55], ...
    'BreakpointsForDimension1','p.n_bp','Table','p.T_curve', ...
    'InterpMethod','Linear point-slope','ExtrapMethod','Clip');
mk(s,'取小限幅','simulink/Math Operations/MinMax',[320 20 355 50],'Function','min','Inputs','2');
mk(s,'减速放大','simulink/Math Operations/Gain',[380 20 435 50],'Gain',sprintf('%s*p.eta_drv',i_g));
% 滑移率（轮胎松弛一阶滞后）
mk(s,'轮端线速度','simulink/Math Operations/Gain',[80 110 110 140],'Gain','p.r_w');
mk(s,'速度差','simulink/Math Operations/Sum',[140 110 165 140],'Inputs','+-');
mk(s,'低速保底','simulink/Sources/Constant',[80 175 125 195],'Value','p.v_floor');
mk(s,'分母保底','simulink/Math Operations/MinMax',[140 170 180 200],'Function','max','Inputs','2');
mk(s,'滑移率相除','simulink/Math Operations/Divide',[210 105 240 145]);
mk(s,'松弛滤波','simulink/Discrete/Discrete Filter',[265 100 340 150], ...
    'Numerator','[p.alpha_relax 0]','Denominator','[1 -(1-p.alpha_relax)]', ...
    'SampleTime','p.Ts_plant','InitialStates','0');
% 轮胎力（魔术公式简化版）
mk(s,'附着曲线','simulink/User-Defined Functions/Fcn',[360 105 430 145], ...
    'Expr','sin(p.C_pac*atan(p.B_pac*u))');
mk(s,'轮胎力计算','simulink/Math Operations/Product',[390 100 425 180],'Inputs','3');
mk(s,'力转扭矩','simulink/Math Operations/Gain',[455 120 495 150],'Gain','p.r_w');
% 车轮转动方程
mk(s,'扭矩求和','simulink/Math Operations/Sum',[520 40 545 140],'Inputs','+-');
mk(s,'惯量倒数','simulink/Math Operations/Gain',[580 40 630 80],'Gain',sprintf('1/%s',J));
mk(s,'转速积分','simulink/Continuous/Integrator',[670 40 710 90],'InitialCondition','p.v0/p.r_w');
out(s,1,'轮速',[760 50 790 70]);
out(s,2,'滑移率',[760 190 790 210]);
out(s,3,'轮胎力',[760 320 790 340]);
ln(s,'转速积分/1','电机转速/1');
ln(s,'电机转速/1','转速延时/1');
ln(s,'转速延时/1','可用扭矩/1');
ln(s,'电机扭矩/1','取小限幅/1');
ln(s,'可用扭矩/1','取小限幅/2');
ln(s,'取小限幅/1','减速放大/1');
ln(s,'转速积分/1','轮端线速度/1');
ln(s,'轮端线速度/1','速度差/1');
ln(s,'车速/1','速度差/2');
ln(s,'车速/1','分母保底/1');
ln(s,'低速保底/1','分母保底/2');
ln(s,'速度差/1','滑移率相除/1');
ln(s,'分母保底/1','滑移率相除/2');
ln(s,'滑移率相除/1','松弛滤波/1');
ln(s,'松弛滤波/1','附着曲线/1');
ln(s,'附着曲线/1','轮胎力计算/1');
ln(s,'路面附着/1','轮胎力计算/2');
ln(s,'轮上载荷/1','轮胎力计算/3');
ln(s,'轮胎力计算/1','力转扭矩/1');
ln(s,'减速放大/1','扭矩求和/1');
ln(s,'力转扭矩/1','扭矩求和/2');
ln(s,'扭矩求和/1','惯量倒数/1');
ln(s,'惯量倒数/1','转速积分/1');
ln(s,'转速积分/1','轮速/1');
ln(s,'松弛滤波/1','滑移率/1');
ln(s,'轮胎力计算/1','轮胎力/1');
add_note(s,[15 440], sprintf([ ...
    '%s车轮：J*dw/dt = T_电机x减速比x效率 - mu(lam)*Fz*r\n' ...
    'lam = (w*r - v)/max(v, %.1f m/s)，经轮胎松弛滤波(tau=%.2fs)\n' ...
    'mu(lam) = sin(C*atan(B*lam))，峰值滑移率 %.2f（曲线见参数说明文档）\n' ...
    '电机外特性限幅保证：即使 TCS 失效，扭矩也不会超过电机能力'], ...
    cn, pd.v_floor, pd.tau_relax, pd.lam_p));
end

function fill_body(s)
% 车身纵向动力学：m*dv/dt = 四轮轮胎力之和 - 风阻 - 滚动阻力
% 轴荷转移：Fz_前 = m*g*b/L - m*h/L*ax，Fz_后 = m*g*a/L + m*h/L*ax（每轮再分一半）
in(s,1,'力_左前',[15 170 45 190]);
in(s,2,'力_右前',[15 240 45 260]);
in(s,3,'力_左后',[15 310 45 330]);
in(s,4,'力_右后',[15 380 45 400]);
mk(s,'合力','simulink/Math Operations/Sum',[80 180 105 260],'Inputs','++++');
mk(s,'速度平方','simulink/Math Operations/Product',[80 70 110 100],'Inputs','2');
mk(s,'风阻系数','simulink/Math Operations/Gain',[140 70 190 100],'Gain','0.5*p.rho_air*p.CdA');
mk(s,'方向','simulink/Math Operations/Sign',[80 20 110 50]);
mk(s,'滚阻','simulink/Math Operations/Gain',[140 20 200 50],'Gain','p.f_rr*p.m*p.g');
mk(s,'净外力','simulink/Math Operations/Sum',[230 90 255 200],'Inputs','+--');
mk(s,'质量倒数','simulink/Math Operations/Gain',[290 130 340 160],'Gain','1/p.m');
mk(s,'车速积分','simulink/Continuous/Integrator',[380 125 420 165],'InitialCondition','p.v0');
mk(s,'加速度延时','simulink/Discrete/Unit Delay',[380 230 420 260],'SampleTime','p.Ts_plant','InitialCondition','0');
% 轴荷（动态转移），左右轮各分一半
mk(s,'前轴静载','simulink/Sources/Constant',[225 300 292 330],'Value','p.m*p.g*p.b_cg/p.L');
mk(s,'前轴转移','simulink/Math Operations/Gain',[230 350 290 380],'Gain','-p.m*p.h_cg/p.L');
mk(s,'前轴载荷','simulink/Math Operations/Sum',[330 300 355 380],'Inputs','++');
mk(s,'后轴静载','simulink/Sources/Constant',[225 420 292 450],'Value','p.m*p.g*p.a_cg/p.L');
mk(s,'后轴转移','simulink/Math Operations/Gain',[230 470 290 500],'Gain','p.m*p.h_cg/p.L');
mk(s,'后轴载荷','simulink/Math Operations/Sum',[330 420 355 500],'Inputs','++');
mk(s,'前轴减半','simulink/Math Operations/Gain',[400 310 440 340],'Gain','0.5');
mk(s,'后轴减半','simulink/Math Operations/Gain',[400 430 440 460],'Gain','0.5');
out(s,1,'车速',[460 135 490 155]);
out(s,2,'纵向加速度',[430 230 470 250]);
out(s,3,'载荷_左前',[500 305 540 325]);
out(s,4,'载荷_右前',[500 345 540 365]);
out(s,5,'载荷_左后',[500 425 540 445]);
out(s,6,'载荷_右后',[500 465 540 485]);
ln(s,'力_左前/1','合力/1');
ln(s,'力_右前/1','合力/2');
ln(s,'力_左后/1','合力/3');
ln(s,'力_右后/1','合力/4');
ln(s,'车速积分/1','速度平方/1');
ln(s,'车速积分/1','速度平方/2');
ln(s,'速度平方/1','风阻系数/1');
ln(s,'车速积分/1','方向/1');
ln(s,'方向/1','滚阻/1');
ln(s,'合力/1','净外力/1');
ln(s,'风阻系数/1','净外力/2');
ln(s,'滚阻/1','净外力/3');
ln(s,'净外力/1','质量倒数/1');
ln(s,'质量倒数/1','车速积分/1');
ln(s,'质量倒数/1','加速度延时/1');
ln(s,'质量倒数/1','纵向加速度/1');
ln(s,'车速积分/1','车速/1');
ln(s,'前轴静载/1','前轴载荷/1');
ln(s,'加速度延时/1','前轴转移/1');
ln(s,'前轴转移/1','前轴载荷/2');
ln(s,'后轴静载/1','后轴载荷/1');
ln(s,'加速度延时/1','后轴转移/1');
ln(s,'后轴转移/1','后轴载荷/2');
ln(s,'前轴载荷/1','前轴减半/1');
ln(s,'后轴载荷/1','后轴减半/1');
ln(s,'前轴减半/1','载荷_左前/1');
ln(s,'前轴减半/1','载荷_右前/1');
ln(s,'后轴减半/1','载荷_左后/1');
ln(s,'后轴减半/1','载荷_右后/1');
add_note(s,[15 540], sprintf([ ...
    'm*dv/dt = sum(F_i) - 0.5*rho*CdA*v^2 - f_rr*m*g*sign(v)\n' ...
    'Fz_前轴 = m*g*b/L - m*h/L*ax；Fz_后轴 = m*g*a/L + m*h/L*ax\n' ...
    '轴荷转移用上一拍加速度(Unit Delay)，断开“轮胎力<->轴荷”代数环']));
end

%% ======================================================================
%%  父级连线
%% ======================================================================
function wire_controller(s)
in(s,1,'油门开度',[20 40 50 60]);
in(s,2,'轮速_左前',[20 100 50 120]);
in(s,3,'轮速_右前',[20 160 50 180]);
in(s,4,'轮速_左后',[20 220 50 240]);
in(s,5,'轮速_右后',[20 280 50 300]);
in(s,6,'真值车速',[20 340 50 360]);
out(s,1,'电机扭矩_左前',[1200 40 1230 60]);
out(s,2,'电机扭矩_右前',[1200 100 1230 120]);
out(s,3,'电机扭矩_左后',[1200 160 1230 180]);
out(s,4,'电机扭矩_右后',[1200 220 1230 240]);
out(s,5,'诊断_参考车速',[1200 300 1230 320]);
out(s,6,'诊断_滑移率',[1200 360 1230 380]);
out(s,7,'诊断_使能',[1200 420 1230 440]);
out(s,8,'诊断_补偿前',[1200 480 1230 500]);
out(s,9,'诊断_补偿后',[1200 540 1230 560]);
out(s,10,'诊断_车手扭矩前',[1200 600 1230 620]);
out(s,11,'诊断_车手扭矩后',[1200 660 1230 680]);

ln(s,'轮速_左前/1','1_参考车速估计/1');
ln(s,'轮速_右前/1','1_参考车速估计/2');
ln(s,'轮速_左后/1','1_参考车速估计/3');
ln(s,'轮速_右后/1','1_参考车速估计/4');
ln(s,'真值车速/1','1_参考车速估计/5');
ln(s,'1_参考车速估计/1','2_滑移率计算/5');
ln(s,'轮速_左前/1','2_滑移率计算/1');
ln(s,'轮速_右前/1','2_滑移率计算/2');
ln(s,'轮速_左后/1','2_滑移率计算/3');
ln(s,'轮速_右后/1','2_滑移率计算/4');
ln(s,'2_滑移率计算/7','3_介入退出判断/1');
ln(s,'1_参考车速估计/1','3_介入退出判断/2');
ln(s,'3_介入退出判断/1','4_介入平滑/1');
% 查表：油门 + 该轴两个轮速
ln(s,'油门开度/1','5a_油门扭矩查表_前轴/1');
ln(s,'轮速_左前/1','5a_油门扭矩查表_前轴/2');
ln(s,'轮速_右前/1','5a_油门扭矩查表_前轴/3');
ln(s,'油门开度/1','5b_油门扭矩查表_后轴/1');
ln(s,'轮速_左后/1','5b_油门扭矩查表_后轴/2');
ln(s,'轮速_右后/1','5b_油门扭矩查表_后轴/3');
% PID：每轴用“该轴滑移率最大值”控制
ln(s,'2_滑移率计算/5','6a_PID扭矩补偿_前轴/1');
ln(s,'2_滑移率计算/6','6b_PID扭矩补偿_后轴/1');
ln(s,'5a_油门扭矩查表_前轴/1','6a_PID扭矩补偿_前轴/2');
ln(s,'5b_油门扭矩查表_后轴/1','6b_PID扭矩补偿_后轴/2');
ln(s,'3_介入退出判断/1','6a_PID扭矩补偿_前轴/3');
ln(s,'3_介入退出判断/1','6b_PID扭矩补偿_后轴/3');
% 仲裁分配
ln(s,'5a_油门扭矩查表_前轴/1','7_扭矩仲裁与四轮分配/1');
ln(s,'5b_油门扭矩查表_后轴/1','7_扭矩仲裁与四轮分配/2');
ln(s,'6a_PID扭矩补偿_前轴/1','7_扭矩仲裁与四轮分配/3');
ln(s,'6b_PID扭矩补偿_后轴/1','7_扭矩仲裁与四轮分配/4');
ln(s,'4_介入平滑/1','7_扭矩仲裁与四轮分配/5');
% 诊断输出
ln(s,'1_参考车速估计/1','诊断_参考车速/1');
ln(s,'2_滑移率计算/7','诊断_滑移率/1');
ln(s,'4_介入平滑/1','诊断_使能/1');
ln(s,'6a_PID扭矩补偿_前轴/1','诊断_补偿前/1');
ln(s,'6b_PID扭矩补偿_后轴/1','诊断_补偿后/1');
ln(s,'5a_油门扭矩查表_前轴/1','诊断_车手扭矩前/1');
ln(s,'5b_油门扭矩查表_后轴/1','诊断_车手扭矩后/1');
% 仲裁子系统输出端口：1/2 = 前后轴总指令(诊断)，3/4/5/6 = 四轮扭矩
ln(s,'7_扭矩仲裁与四轮分配/3','电机扭矩_左前/1');
ln(s,'7_扭矩仲裁与四轮分配/4','电机扭矩_右前/1');
ln(s,'7_扭矩仲裁与四轮分配/5','电机扭矩_左后/1');
ln(s,'7_扭矩仲裁与四轮分配/6','电机扭矩_右后/1');
add_note(s,[100 760], sprintf([ ...
    '控制器输入：油门开度 + 四轮轮速（实车全部可从 CAN 拿到：轮速=电机编码器/减速比）\n' ...
    '“真值车速”输入只存在于仿真里（对照验证参考车速估计精度用），实车代码没有这一路\n' ...
    '控制链路：估车速 -> 算滑移率 -> 判断介入 -> PID 算补偿 -> 从车手扭矩里扣 -> 分给四台电机']));
end

function wire_plant(s)
in(s,1,'扭矩_左前',[20 40 50 60]);
in(s,2,'扭矩_右前',[20 100 50 120]);
in(s,3,'扭矩_左后',[20 160 50 180]);
in(s,4,'扭矩_右后',[20 220 50 240]);
in(s,5,'路面附着',[20 280 50 300]);
out(s,1,'车速',[900 40 930 60]);
out(s,2,'轮速_左前',[900 90 930 110]);
out(s,3,'轮速_右前',[900 140 930 160]);
out(s,4,'轮速_左后',[900 190 930 210]);
out(s,5,'轮速_右后',[900 240 930 260]);
out(s,6,'纵向加速度',[900 290 930 310]);

wheels = {'左前','右前','左后','右后'};
tins   = {'扭矩_左前','扭矩_右前','扭矩_左后','扭矩_右后'};
% 扭矩进各轮
for k = 1:4
    ln(s,[tins{k} '/1'],['车轮_' wheels{k} '/1']);
end
% 车速进各轮（第 2 输入）
for k = 1:4
    ln(s,'车身纵向动力学/1',['车轮_' wheels{k} '/2']);
end
% 路面附着进各轮（第 4 输入）
for k = 1:4
    ln(s,'路面附着/1',['车轮_' wheels{k} '/4']);
end
% 各轮轮胎力进车身（第 1~4 输入）
for k = 1:4
    ln(s,['车轮_' wheels{k} '/3'],['车身纵向动力学/' num2str(k)]);
end
% 车身角载荷进各轮（第 3 输入）：左前<-1 右前<-2 左后<-5 右后<-6
ln(s,'车身纵向动力学/3','车轮_左前/3');
ln(s,'车身纵向动力学/4','车轮_右前/3');
ln(s,'车身纵向动力学/5','车轮_左后/3');
ln(s,'车身纵向动力学/6','车轮_右后/3');
% 输出
ln(s,'车身纵向动力学/1','车速/1');
ln(s,'车轮_左前/1','轮速_左前/1');
ln(s,'车轮_右前/1','轮速_右前/1');
ln(s,'车轮_左后/1','轮速_左后/1');
ln(s,'车轮_右后/1','轮速_右后/1');
ln(s,'车身纵向动力学/2','纵向加速度/1');
add_note(s,[560 380], sprintf([ ...
    '被控对象：四台电机扭矩 -> 四个车轮(各自的外特性限幅/轮胎/转动方程) -> 车身\n' ...
    '四个轮子互相独立：想扩展成“左右附着不同(对开路面)”或“单轮打滑”工况很容易']));
end

function wire_top(mdl)
ln(mdl,'CLK_Time/1','DRV_油门踏板曲线/1');
ln(mdl,'CLK_Time/1','ROAD_路面附着曲线/1');
ln(mdl,'DRV_油门踏板曲线/1','TCS_控制器/1');
ln(mdl,'ROAD_路面附着曲线/1','VEH_被控对象/5');
ln(mdl,'TCS_控制器/1','VEH_被控对象/1');
ln(mdl,'TCS_控制器/2','VEH_被控对象/2');
ln(mdl,'TCS_控制器/3','VEH_被控对象/3');
ln(mdl,'TCS_控制器/4','VEH_被控对象/4');
ln(mdl,'VEH_被控对象/1','TCS_控制器/6');
ln(mdl,'VEH_被控对象/2','TCS_控制器/2');
ln(mdl,'VEH_被控对象/3','TCS_控制器/3');
ln(mdl,'VEH_被控对象/4','TCS_控制器/4');
ln(mdl,'VEH_被控对象/5','TCS_控制器/5');
% 数据记录（To Workspace，跑完 run_tcs4_sim 画图用）
% 注意：VariableName 必须是合法 MATLAB 变量名（ASCII），模块名仍用中文方便截图
logs = {
    'VEH_被控对象/1','LOG_v',        [1240  60]
    'VEH_被控对象/2','LOG_w_fl',     [1240 110]
    'VEH_被控对象/3','LOG_w_fr',     [1240 160]
    'VEH_被控对象/4','LOG_w_rl',     [1240 210]
    'VEH_被控对象/5','LOG_w_rr',     [1240 260]
    'VEH_被控对象/6','LOG_ax',       [1240 310]
    'TCS_控制器/5','LOG_v_ref',      [1400  60]
    'TCS_控制器/6','LOG_lam',        [1400 110]
    'TCS_控制器/7','LOG_en',         [1400 160]
    'TCS_控制器/8','LOG_comp_f',     [1400 210]
    'TCS_控制器/9','LOG_comp_r',     [1400 260]
    'TCS_控制器/10','LOG_drv_f',     [1400 310]
    'TCS_控制器/11','LOG_drv_r',     [1400 360]
    'DRV_油门踏板曲线/1','LOG_pedal',  [1400 410]
    'ROAD_路面附着曲线/1','LOG_mu',    [1400 460]
    };
for k = 1:size(logs,1)
    nm = logs{k,2}; xy = logs{k,3};
    mk(mdl,nm,'simulink/Sinks/To Workspace',[xy(1) xy(2) xy(1)+90 xy(2)+30], ...
        'VariableName',nm,'SaveFormat','Timeseries');
    ln(mdl,logs{k,1},[nm '/1']);
end
end

%% ======================================================================
%%  工具函数
%% ======================================================================
function mk(sys,name,lib,pos,varargin)
h = [sys '/' name];
add_block(lib,h,'Position',pos);
for k = 1:2:numel(varargin)
    set_param(h,varargin{k},varargin{k+1});
end
end

function s = mkSub(sys,name,pos)
s = [sys '/' name];
add_block('simulink/Ports & Subsystems/Subsystem',s,'Position',pos);
delete_block([s '/In1']);
delete_block([s '/Out1']);
end

function in(sys,port,name,pos)
mk(sys,name,'simulink/Ports & Subsystems/In1',pos,'Port',num2str(port));
end

function out(sys,port,name,pos)
mk(sys,name,'simulink/Ports & Subsystems/Out1',pos,'Port',num2str(port));
end

function ln(sys,src,dst)
add_line(sys,src,dst,'autorouting','on');
end

function lookup(sys,name,bp,tbl,interp,pos)
mk(sys,name,'simulink/Lookup Tables/1-D Lookup Table',pos, ...
    'BreakpointsForDimension1',bp,'Table',tbl, ...
    'InterpMethod',interp,'ExtrapMethod','Clip');
end

function add_note(sys,pos,txt)
try
    a = Simulink.Annotation([sys '/' sprintf('note_%d_%d',pos(1),pos(2))]);
    a.Text = txt;
    a.Position = [pos(1) pos(2) pos(1)+440 pos(2)+70];
catch
end
end
