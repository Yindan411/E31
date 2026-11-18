%TCS4_DEMO  一键脚本：建模型（如没有）+ 跑全部工况 + 出图
%   直接在 MATLAB 里运行本文件即可；生成的模型 TCS4_Top.slx 可以双击打开截图。
%
%   已有模型只想重跑工况：直接运行 run_tcs4_sim
%   改了模型结构想重建：  先删掉 TCS4_Top.slx 再运行本文件

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);

if ~exist(fullfile(thisDir,'TCS4_Top.slx'),'file')
    build_tcs4_model(thisDir);
end

% 打开模型窗口（可视化：答辩截图直接在 Simulink 里截）
open_system('TCS4_Top');

% 跑全部工况
metrics = run_tcs4_sim(thisDir);
assignin('base','metrics',metrics);

fprintf('\n==== 全部完成 ====\n');
fprintf('模型: %s\n', fullfile(thisDir,'TCS4_Top.slx'));
fprintf('图表: %s\n', fullfile(thisDir,'sim_out'));
fprintf('指标: %s\n', fullfile(thisDir,'sim_out','metrics.txt'));
