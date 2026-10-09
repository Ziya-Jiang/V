# robot_agent —— Agent + SKILL + Tool 抓取系统

把机器人操作分解为 **元操作（Tool）** → 由 **SKILL 组合** → 由 **Agent 规划**，
实现「用户一句话 → 规划 SKILL 调用顺序与约束 → 执行」的完整闭环。

```
用户需求（自然语言）
      │  Agent (planner.py)
      ▼
Plan ── 有序 + 带约束的 SKILL 调用序列
      │  Executor (executor.py)
      ▼
SKILL（perceive_scene / pick_and_place / tidy_table）
      │  组合
      ▼
Tool（approach / grasp / place / transport / reset / ground / estimate / localize）
      │  复用
      ▼
RobotRAM（step1~step5、controllor、visions —— 原有底层能力，不改动）
```

## 目录结构

```
robot_agent/
├── base.py           # Tool / Skill / SkillCall / Plan / ToolResult 数据结构
├── context.py        # RobotContext：机器人、相机、模型、标定的惰性装配
├── registry.py       # Tool + Skill 注册表，生成 LLM 可见的 manifest
├── planner.py        # Agent：自然语言 → Plan（规则 + 可选 LLM 两种后端）
├── executor.py       # 执行 Plan：解析语义物体、校验依赖、调用 SKILL
├── llm.py            # 可选 LLM 后端（OpenAI 兼容，读环境变量）
├── run.py            # 对话式入口（python -m robot_agent.run）
├── tools/            # 元操作 + 感知原语
│   ├── manipulation.py   # approach / grasp / place / transport / reset
│   └── perception.py     # ground / estimate_grasp_poses / localize
└── skills/           # SKILL 组合
    ├── perceive.py       # perceive_scene
    ├── pick_and_place.py # pick_and_place
    └── tidy_table.py     # tidy_table
```

## 三层设计

### Tool —— 原子操作（元操作 / 感知）

每个 Tool 都是自描述的：`name` + `description` + `parameters`（JSON Schema），
Agent 可以像 LLM function-calling 一样读取 `registry.tool_manifest()` 来规划。

| Tool | 类别 | 语义 | 复用的原有动作 / 步骤 |
| --- | --- | --- | --- |
| `approach` | manipulation | 接近（`above`/`descend`） | `move_above_grasp` / `move_down_to_grasp` |
| `grasp` | manipulation | 抓取 | `close_gripper` |
| `place` | manipulation | 放置 | `open_gripper` |
| `transport` | manipulation | 搬运（`lift`/`move`/`lower`/`retreat`） | `lift_object` / `move_to_target` / `move_down_to_place` / `move_up_after_release` |
| `reset` | manipulation | 复位（`safe`/`first`/`pos`/`fork`） | `reset_all` / `first_reset` / `reset_pos` / `reset_for_fork` |
| `ground` | perception | 语义检测（Grounding-DINO + SAM2） | Step1 `grounding_main` |
| `estimate_grasp_poses` | perception | 物体 6-DoF 位姿 + 抓取位姿（VGGT + RAM） | Step2 `ram_main` |
| `localize` | perception | 相机系 → 机器人基座系 | Step3 `read_object_main` |

### SKILL —— 组合

| SKILL | 说明 |
| --- | --- |
| `perceive_scene` | `ground → estimate_grasp_poses → localize`，产出基座系物体元数据 |
| `pick_and_place` | 一次「把 X 放到 Y」：复用 Step4 `planning_main` 生成抓取/放置位姿，再派发到 `approach→grasp→transport→place→reset` 元操作 |
| `tidy_table` | 「整理桌面」：`perceive_scene` + 多次 `pick_and_place` |

### Agent —— 规划

`Agent.plan(instruction)` 输出 `Plan`（有序 `SkillCall` + 约束）：

```json
{
  "instruction": "帮我整理桌面",
  "constraints": { "task": "clean_table", "ordering": "sequential" },
  "steps": [
    { "skill": "perceive_scene", "args": {"instruction": "帮我整理桌面"}, "ref": "scene" },
    { "skill": "pick_and_place",
      "args": { "object": {"category":"bowl","color":"green"}, "target": {"category":"plate","color":"yellow"}, "placement_type":"on_center" },
      "constraints": { "depends_on": ["scene"] } }
  ]
}
```

物体用 **语义描述（category/color）** 引用，而非硬编码 `object_0`：`perceive_scene`
跑完后由 Executor 解析成语义 id，因此同一份计划可跨场景复用。

## 运行

```bash
# 只规划、不碰硬件（离线可用）
python -m robot_agent.run --plan-only --instruction "帮我整理桌面"

# 对话式循环（需要真实机器人 / 相机 / 模型时再加 --interactive）
python -m robot_agent.run --interactive

# 编程方式
from robot_agent.context import RobotContext
from robot_agent.registry import build_default_registry
from robot_agent.planner import Agent
from robot_agent.executor import Executor

ctx = RobotContext(output_dir="RobotRAM/output/scene_57", instruction="帮我整理桌面")
registry = build_default_registry(ctx)
agent = Agent(registry)
plan = agent.plan("帮我整理桌面")     # NL -> SKILL 组合
ctx.bootstrap()                      # 装配机械臂/相机/模型
Executor(registry).run(plan, ctx)    # 执行
```

## 旧 → 新 映射

| 旧（单体 `dual_v2_2_agent.py`） | 新 |
| --- | --- |
| `prompt = "clean the table"` 硬编码 | `Agent.plan(instruction)`，自然语言输入 |
| `while True: input(task_type)` 手动选任务 | Agent 从语言推断 SKILL 序列 |
| `predefined_tasks = [("object_0","object_1",...)]` | `CLEAN_TABLE_PLACEMENTS`（语义化 category/color） |
| Step1~Step5 线性管道 | `perceive_scene` + `pick_and_place` SKILL |
| `execute_single_task` 的 8 步动作 | `approach/grasp/place/transport/reset` 元操作 |

原有 `RobotRAM/` 与 `dual_v2_2_agent.py` 保持原样，本包以 **适配层** 复用它们，
不破坏既有可运行的演示。

## 可选：接入 DSH（DeepSeek Harness）

`registry.tool_manifest()` / `registry.skill_manifest()` 已经是标准 JSON Schema，
可直接映射为 DSH 的 model Tools（或动态 Cordis Plugin 的 Tool），让本 GUI 中的
Agent 直接驱动这套抓取系统；`robot_agent` 本身不依赖 DSH，可独立运行。
