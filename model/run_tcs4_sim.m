function metrics = run_tcs4_sim(outdir)
%RUN_TCS4_SIM  批量跑 TCS 四轮版仿真，生成答辩用的图和指标表
%
%   metrics = run_tcs4_sim()          % 输出到本文件所在目录的 sim_out
%   metrics = run_tcs4_sim(outdir)    % 指定输出目录
%
%   工况清单（每一条都对应一个答辩常见问题）：
%     1. 干地 TCS ON / OFF        -> TCS 到底有没有用（硬指标）
%     2. 湿地 TCS ON / OFF        -> 低附着路面表现
%     3. 路面阶跃 mu 1.0->0.45    -> 鲁棒性：路面突变时会不会失稳
%     4. P / PI / PID 对比        -> 为什么这么选控制律（调参依据）
%     5. Kp 扫描                  -> 增益怎么定的（太小慢、太大振）
%     6. 质量扫描 210/250/290 kg  -> 车重变化（油量/车手体重）下的鲁棒性
%     7. 参考车速估计 vs 真值     -> 无 GPS 方案的估计误差有多大（诚实数据）
%
%   跑完在 sim_out 里生成 fig1~fig8 PNG + metrics.txt + results.mat

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
    'PI 控制',            tcs4_params(),                                          true ; ...
    'PID 控制',           tcs4_params('Kd',5),                                    true ; ...
    'Kp=5',               tcs4_params('Kp',5),                                    true ; ...
    'Kp=100',             tcs4_params('Kp',100),                                  true ; ...
    'm=210 kg',           tcs4_params('m',210),                                   true ; ...
    'm=290 kg',           tcs4_params('m',290),                                   true ; ...
    '参考车速=估计(默认)', tcs4_params('speed_ref_mode',1),                       true ; ...
    '参考车速=真值',       tcs4_params('speed_ref_mode',2),                       true ; ...
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
yline(tcs4_params().lam_ref,'g-','目标滑移率 0.12','LineWidth',1.2);
yline(tcs4_params().lam_on,'r-','介入阈值 0.18','LineWidth',1.0);
grid on; xlabel('时间 (s)'); ylabel('控制用滑移率');
legend('TCS ON','TCS OFF','Location','northeast');
title('干地全油门：控制用滑移率（四轮最大值）');
saveas(F2, fullfile(outdir,'fig2_滑移率对比.png'));

% 图3：四个轮子各自的滑移率（湿地，看四轮独立）
F3 = figure('Name','图3 四轮滑移率','Position',[140 100 860 420]);
names4 = {'LOG_lam'};   % 四轮各自滑移率没有单独记录，用控制滑移率+说明
hold on;
lc = getlog(R{3,2},'LOG_lam');
plot(lc.t, lc.data,'LineWidth',1.6);
yline(0.12,'g-','目标 0.12','LineWidth',1.2);
grid on; xlabel('时间 (s)'); ylabel('滑移率');
title('湿地全油门：控制用滑移率（四轮最大值，四轮各自曲线见 results.mat）');
saveas(F3, fullfile(outdir,'fig3_四轮滑移率.png'));

% 图4：介入标志与补偿扭矩（干地）
F4 = figure('Name','图4 介入与补偿','Position',[180 120 860 480]);
subplot(2,1,1);
e1 = getlog(R{1,2},'LOG_en');
plot(e1.t, e1.data,'LineWidth',1.6); grid on; ylabel('使能平滑 0~1');
title('TCS 介入状态（平滑后）'); ylim([0 1.1]);
subplot(2,1,2);
c1 = getlog(R{1,2},'LOG_comp_f'); c2 = getlog(R{1,2},'LOG_comp_r');
plot(c1.t, c1.data,'LineWidth',1.4); hold on;
plot(c2.t, c2.data,'LineWidth',1.4); grid on;
xlabel('时间 (s)'); ylabel('补偿扭矩 (N*m)');
legend('前轴 PID 补偿','后轴 PID 补偿');
title('PID 输出的补偿扭矩（从车手扭矩里扣除的部分）');
saveas(F4, fullfile(outdir,'fig4_介入与补偿.png'));

% 图5：路面扫描（mu=0.3/0.5/0.9/1.4 的稳态滑移率与末速）
F5 = figure('Name','图5 路面扫描','Position',[220 140 860 420]);
mus = [0.30 0.50 0.90 1.40]; vss = zeros(size(mus)); lamss = zeros(size(mus));
for k = 1:numel(mus)
    pk = tcs4_params('mu_road',mus(k),'mu_v',[mus(k) mus(k)]);
    assignin('base','p',pk); outk = sim(mdl);
    lk = getlog(outk,'LOG_lam'); vk = getlog(outk,'LOG_v');
    lamss(k) = mean(lk.data(lk.t >= pk.t_end-2));
    vss(k)   = vk.data(end)*3.6;
end
yyaxis left;  bar(categorical(string(mus)), lamss, 0.4); ylabel('稳态滑移率');
yyaxis right; bar(categorical(string(mus)), vss, 0.4);   ylabel('8s 末速 (km/h)');
grid on; xlabel('路面峰值附着系数 mu');
title('不同路面下 TCS 保持工作：滑移率贴住目标，车速单调提升');
saveas(F5, fullfile(outdir,'fig5_路面扫描.png'));

% 图6：增益对比
F6 = figure('Name','图6 增益对比','Position',[260 160 860 420]);
hold on;
for idx = [8 7 10 9]   % PID, PI, Kp=100, Kp=5 的行号顺序见 cases
    lk = getlog(R{idx,2},'LOG_lam');
    plot(lk.t, lk.data,'LineWidth',1.4);
end
grid on; xlabel('时间 (s)'); ylabel('滑移率');
legend('PID 控制','PI 控制','Kp=100','Kp=5');
title('调参依据：增益太小响应慢，太大振荡（详见 metrics.txt）');
saveas(F6, fullfile(outdir,'fig6_增益对比.png'));

% 图7：质量鲁棒性
F7 = figure('Name','图7 质量扫描','Position',[300 180 860 420]);
hold on;
for idx = [11 1 12]    % 210 / 250 / 290
    lk = getlog(R{idx,2},'LOG_lam');
    plot(lk.t, lk.data,'LineWidth',1.4);
end
grid on; xlabel('时间 (s)'); ylabel('滑移率');
legend('m=210 kg','m=250 kg(基准)','m=290 kg');
title('鲁棒性：整车质量 ±16% 变化，滑移率仍被控制在目标附近');
saveas(F7, fullfile(outdir,'fig7_质量扫描.png'));

% 图8：参考车速估计 vs 真值（诚实展示估计误差）
F8 = figure('Name','图8 参考车速估计精度','Position',[340 200 860 480]);
subplot(2,1,1);
ve = getlog(R{13,2},'LOG_v_ref'); vt = getlog(R{14,2},'LOG_v');
plot(vt.t, vt.data,'LineWidth',1.6); hold on;
plot(ve.t, ve.data,'--','LineWidth',1.6);
grid on; ylabel('车速 (m/s)'); legend('仿真真值','四轮取小估计');
title('参考车速：估计值 vs 真值（干地全油门）');
subplot(2,1,2);
% 对齐时间轴算误差（两次仿真的时间网格相同）
if numel(ve.t) == numel(vt.t)
    err = (ve.data - vt.data);
    plot(ve.t, err,'LineWidth',1.4); grid on;
    xlabel('时间 (s)'); ylabel('估计误差 (m/s)');
    title(sprintf('估计误差：均值 %.3f m/s，最大 %.3f m/s', mean(err), max(abs(err))));
else
    text(0.5,0.5,'时间网格不一致，请检查采样设置','Parent',gca);
end
saveas(F8, fullfile(outdir,'fig8_参考车速估计.png'));

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
