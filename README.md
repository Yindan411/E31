# TCS 四轮版仿真（FSEC · 四电机分布式驱动）

四电机 AMK DD5 轮边驱动的牵引力控制系统（TCS）仿真，控制律：滑移率闭环 PID。
用于 FSEC 设计答辩的模型与文档资料包（本仓库持续迭代中）。

## 目录

- model/  Simulink 模型与脚本
- docs/   答辩文档（待整理）

## 当前进展

- [x] 四轮被控对象建模（独立车轮 + 魔术公式轮胎 + 载荷转移）
- [x] TCS 控制器骨架：介入/退出滞环 + PID 扭矩补偿（前后轴各一路）
- [ ] 批量工况脚本（干地/湿地 ON/OFF 对比自动出图）
- [ ] PID 增益整定（初值 Kp=12 / Ki=8，湿地收敛偏慢）
- [ ] 滑移率阈值标定（初值：目标 0.15 / 介入 0.20）
- [ ] 答辩文档六件套

## 怎么跑

```matlab
cd model
build_tcs4_model       % 生成 TCS4_Top.slx（已有则跳过）
p = tcs4_params();
assignin('base','p',p);
sim('TCS4_Top');
```
