function metrics = run_tcs4_sim(outdir)
%RUN_TCS4_SIM  批量跑 TCS 四轮版仿真，生成对比图和指标表
%
%   metrics = run_tcs4_sim()          % 输出到本文件所在目录的 sim_out
%   metrics = run_tcs4_sim(outdir)    % 指定输出目录
%
%   工况清单（v0.2，6 工况）：
%     1. 干地 TCS ON / OFF        -> TCS 到底有没有用（硬指标）
%     2. 湿地 TCS ON / OFF        -> 低附着路面表现
%     3. 路面阶跃 mu 1.0->0.45    -> 鲁棒性：路面突变时会不会失稳
%     4. P 控制（Ki=0）           -> 为增益整定做对照
%
%   跑完在 sim_out 里生成 fig1~fig2 PNG + metrics.txt + results.mat

if nargin < 1 || isempty(outdir)
    outdir = fileparts(mfilename('fullpath'));
end
outdir = fullfile(outdir, 'sim_out');
if ~exist(outdir,'dir'), mkdir(outdir); end
mdl = 'TCS4_Top';
if ~bdIsLoaded(mdl)
    load_system(fullfile(fileparts(mfilename('fullpath')),[mdl '.slx']));
end

R = {};   % 每行: {名称, logs, p}
cases = { ...
    '干地 mu=1.40 ON',    tcs4_params(),                                          true ; ...
    '干地 mu=1.40 OFF',   tcs4_params('TCS_enable',0),                            true ; ...
    '湿地 mu=0.50 ON',    tcs4_params('mu_road',0.50,'mu_v',[0.50 0.50]),         true ; ...
    '湿地 mu=0.50 OFF',   tcs4_params('mu_road',0.50,'mu_v',[0.50 0.50],'TCS_enable',0), true ; ...
    'mu阶跃 1.0->0.45',   tcs4_params('mu_t',[0 2.5 1e9],'mu_v',[1.00 0.45 0.45],'mu_road',1.0), true ; ...
    'P 控制',             tcs4_params('Ki',0),                                    true ; ...
    };

for k = 1:size(cases,1)
    fprintf('(%d/%d) %s ...\n', k, size(cases,1), cases{k,1});
    pk = cases{k,2};
    assignin('base','p',pk);
    out = sim(mdl);
    R{k,1} = cases{k,1};
    R{k,2} = out;
    R{k,3} = pk;
end

%% 指标计算 ---------------------------------------------------------------
metrics = cell(size(cases,1),6);
metrics(1,:) = {'工况','lam峰值','lam稳态(末2s均值)','滑移RMS','TCS介入占比','末速 km/h'};
for k = 1:size(cases,1)
    lam = getlog(R{k,2},'LOG_lam');
    en  = getlog(R{k,2},'LOG_en');
    v   = getlog(R{k,2},'LOG_v');
    tail = lam.t >= (R{k,3}.t_end - 2);
    metrics(k+1,:) = { R{k,1}, ...
        sprintf('%.3f', max(lam.data)), ...
        sprintf('%.3f', mean(lam.data(tail))), ...
        sprintf('%.3f', rms(lam.data)), ...
        sprintf('%.1f%%', 100*mean(en.data >= 0.5)), ...
        sprintf('%.2f', v.data(end)*3.6) };
end
% 打印 + 存文件
fid = fopen(fullfile(outdir,'metrics.txt'),'w','n','UTF-8');
for r = 1:size(metrics,1)
    line = sprintf('%-24s %10s %12s %10s %10s %10s\n', metrics{r,:});
    fprintf('%s', line);
    fprintf(fid, '%s', line);
end
fclose(fid);

%% 画图 -------------------------------------------------------------------
set(groot,'defaultAxesFontSize',11);
% 图1：车速对比（干地/湿地 ON vs OFF）
F1 = figure('Name','图1 车速对比','Position',[60 60 860 420]);
hold on;
v1 = getlog(R{1,2},'LOG_v'); v2 = getlog(R{2,2},'LOG_v');
v3 = getlog(R{3,2},'LOG_v'); v4 = getlog(R{4,2},'LOG_v');
plot(v1.t, v1.data*3.6,'LineWidth',1.6);
plot(v2.t, v2.data*3.6,'--','LineWidth',1.6);
plot(v3.t, v3.data*3.6,'LineWidth',1.6);
plot(v4.t, v4.data*3.6,'--','LineWidth',1.6);
grid on; xlabel('时间 (s)'); ylabel('车速 (km/h)');
legend('干地 TCS ON','干地 TCS OFF','湿地 TCS ON','湿地 TCS OFF','Location','southeast');
title('直线全油门加速：车速对比（虚线 = TCS 关闭）');
saveas(F1, fullfile(outdir,'fig1_车速对比.png'));

% 图2：滑移率对比（干地 ON vs OFF）
F2 = figure('Name','图2 滑移率对比','Position',[100 80 860 420]);
l1 = getlog(R{1,2},'LOG_lam'); l2 = getlog(R{2,2},'LOG_lam');
hold on;
plot(l1.t, l1.data,'LineWidth',1.6);
plot(l2.t, l2.data,'--','LineWidth',1.6);
yline(tcs4_params().lam_ref,'g-','目标滑移率','LineWidth',1.2);
yline(tcs4_params().lam_on,'r-','介入阈值','LineWidth',1.0);
grid on; xlabel('时间 (s)'); ylabel('控制用滑移率');
legend('TCS ON','TCS OFF','Location','northeast');
title('干地全油门：控制用滑移率（四轮最大值）');
saveas(F2, fullfile(outdir,'fig2_滑移率对比.png'));

% 存全部数据
save(fullfile(outdir,'results.mat'), 'R', 'metrics');
fprintf('全部完成，输出目录: %s\n', outdir);
end

%% ----------------------------------------------------------------------
function g = getlog(out, name)
% 从 SimulationOutput（或 base 工作区）取 To Workspace 的时间序列
if exist('out','var') && isa(out,'Simulink.SimulationOutput')
    ts = out.get(name);
else
    ts = evalin('base', name);
end
g.t = ts.Time;
g.data = squeeze(ts.Data);
if iscolumn(g.data), g.data = g.data(:)'; end
end
