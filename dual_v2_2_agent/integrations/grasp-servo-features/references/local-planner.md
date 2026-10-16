# 本机独立控制入口

项目 `/mnt/ssd/user/V/dual_v2_2_agent`；Python `/mnt/ssd/robotarm/env/robotram_wx/bin/python`。
入口为 `servor.py`，使用为 V3.3 准备的专用规划器，**不调用或改写 V3.3 抓取流程**。
物体不限书本；根据用户物体名称、颜色/位置等描述和抓取部位选择刚性平面纹理区域。

先读项目 `README_servor.md` 的命令与接口限制。默认仅感知；用户要求实机对准时用 `--execute`。
用户已选定本阶段目标：只沿 Y 和手腕工具轴旋转纠偏，X/Z 固定，不前伸、不闭合夹爪。
不要将“已对准”报告成“已抓取”。不改旧规划器，不直接发 RT 关节命令。

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python servor.py --object '包装盒' --description '中间蓝色盒子的正面标签，中心和纵向轴'
```

模型首次语义定位会将当前图像及描述发到 OpenAI；后续跟踪在本机进行。
不显示密钥文件内容，不复制密钥到技能。不能外发的图像用本地 `--task` 像素区域，明确这是人工审查结果。
如果自动审批拒绝图片外发，不换通道绕过，说明原因并请求该图片及目的地的具体授权。

日志区分 `features`（前端仍 `control_ready:false`）和经过位姿/时效/可观测性检查的 `control`。
保留现有失效和运动界限，不为追求成功绕过跟踪校验。
