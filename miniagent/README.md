# miniagent

`dual_v2_2_agent_pcl.py` 的最小运行目录。代码、标定、场景描述和机械臂 UDP 协议已放在这里。模型权重没有放进来，按下面的路径自行放上后再启动。

在 `miniagent` 目录下运行：

```bash
python dual_v2_2_agent_pcl.py --assume-head-camera-calibrated
```

环境仍使用原来的 `robotram_wx`。GroundingDINO 第一次运行会从 Hugging Face 读取 `bert-base-uncased`，本机缓存里已有即可，不必放进这个目录。

## 需要自行放入的权重

来源都在原来的 `dual_v2_2_agent` 里。放到下表右侧路径，文件名保持不变。

| 作用 | 放到 |
| --- | --- |
| RAM 位姿网络 | `RobotRAM/visions/ram/ckpts/model.pth` |
| SAM2 分割 | `RobotRAM/visions/grounding/Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt` |
| GroundingDINO 检测 | `RobotRAM/visions/grounding/Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth` |

当前场景 `scene_57` 会用到 bowl、plate、cup、bread、fork。每个类别还要一个模板特征文件：

| 类别 | 放到 |
| --- | --- |
| bowl | `RobotRAM/visions/ram/template/objects/bowl/000561/000561.pkl` |
| plate | `RobotRAM/visions/ram/template/objects/plate/000563/000563.pkl` |
| cup | `RobotRAM/visions/ram/template/objects/cup/000562/000562.pkl` |
| bread | `RobotRAM/visions/ram/template/objects/bread/000571/000571.pkl` |
| fork | `RobotRAM/visions/ram/template/objects/fork/000570/000570.pkl` |

对应的网格 `obj_XXXXXX.ply` 和描述 `XXXXXX.json` 已经在这些目录里。没有 `000561.pkl` 时，程序会改去读同目录的 `000561_source.pkl` 并现场生成 `000561.pkl`。酒杯任务还要再加 `template/objects/wineglass/000602/000602.pkl` 和同名的 ply、json；当前场景文件里没有酒杯。

原来的文件位置：

- `dual_v2_2_agent/RobotRAM/visions/ram/ckpts/model.pth`
- `dual_v2_2_agent/RobotRAM/visions/grounding/Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt`
- `dual_v2_2_agent/RobotRAM/visions/grounding/Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth`
- `dual_v2_2_agent/RobotRAM/visions/ram/template/objects/<类别>/<编号>/<编号>.pkl`

DINOv2 的骨干权重在 `model.pth` 里面，不需要另外的 `dinov2_vits14_pretrain_torch1.pth`。
