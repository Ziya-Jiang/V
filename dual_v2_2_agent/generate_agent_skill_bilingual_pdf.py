from __future__ import annotations

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (
    KeepTogether,
    ListFlowable,
    ListItem,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


OUTPUT = Path(__file__).with_name("Agent_SKILL_架构梳理_中英双语.pdf")
PAGE_WIDTH, PAGE_HEIGHT = landscape(A4)
ACCENT = colors.HexColor("#E94B0C")
INK = colors.HexColor("#202124")
MUTED = colors.HexColor("#5F6873")
DARK = colors.HexColor("#30363D")
LINE = colors.HexColor("#D9DDE1")
PALE = colors.HexColor("#F5F6F7")
PALE_ACCENT = colors.HexColor("#FFF3EE")

pdfmetrics.registerFont(UnicodeCIDFont("STSong-Light"))
pdfmetrics.registerFontFamily(
    "STSong",
    normal="STSong-Light",
    bold="STSong-Light",
    italic="STSong-Light",
    boldItalic="STSong-Light",
)

base = getSampleStyleSheet()
styles = {
    "cover_title": ParagraphStyle(
        "cover_title",
        parent=base["Title"],
        fontName="STSong-Light",
        fontSize=27,
        leading=35,
        textColor=INK,
        alignment=TA_CENTER,
        spaceAfter=10 * mm,
    ),
    "cover_subtitle": ParagraphStyle(
        "cover_subtitle",
        parent=base["Normal"],
        fontName="STSong-Light",
        fontSize=15,
        leading=23,
        textColor=MUTED,
        alignment=TA_CENTER,
        spaceAfter=5 * mm,
    ),
    "h1": ParagraphStyle(
        "h1",
        parent=base["Heading1"],
        fontName="STSong-Light",
        fontSize=19,
        leading=24,
        textColor=INK,
        spaceAfter=5 * mm,
    ),
    "h2": ParagraphStyle(
        "h2",
        parent=base["Heading2"],
        fontName="STSong-Light",
        fontSize=12,
        leading=16,
        textColor=INK,
        spaceBefore=2 * mm,
        spaceAfter=2 * mm,
    ),
    "body": ParagraphStyle(
        "body",
        parent=base["BodyText"],
        fontName="STSong-Light",
        fontSize=9.2,
        leading=14,
        textColor=INK,
        spaceAfter=2.2 * mm,
    ),
    "small": ParagraphStyle(
        "small",
        parent=base["BodyText"],
        fontName="STSong-Light",
        fontSize=7.8,
        leading=11,
        textColor=MUTED,
        spaceAfter=1.2 * mm,
    ),
    "label": ParagraphStyle(
        "label",
        parent=base["BodyText"],
        fontName="STSong-Light",
        fontSize=7.5,
        leading=10,
        textColor=ACCENT,
        spaceAfter=2 * mm,
    ),
    "callout": ParagraphStyle(
        "callout",
        parent=base["BodyText"],
        fontName="STSong-Light",
        fontSize=10,
        leading=15,
        textColor=INK,
    ),
    "table_head": ParagraphStyle(
        "table_head",
        parent=base["BodyText"],
        fontName="STSong-Light",
        fontSize=8,
        leading=10,
        textColor=colors.white,
    ),
    "table": ParagraphStyle(
        "table",
        parent=base["BodyText"],
        fontName="STSong-Light",
        fontSize=7.5,
        leading=10.5,
        textColor=INK,
    ),
}


def p(text: str, style: str = "body") -> Paragraph:
    return Paragraph(text, styles[style])


def bullets(items: list[str], level: int = 0) -> ListFlowable:
    return ListFlowable(
        [ListItem(p(item, "body"), leftIndent=2 * mm) for item in items],
        bulletType="bullet",
        start="circle",
        leftIndent=(5 + level * 3) * mm,
        bulletFontName="STSong-Light",
        bulletFontSize=6,
        spaceAfter=2 * mm,
    )


def numbered(items: list[str]) -> ListFlowable:
    return ListFlowable(
        [ListItem(p(item, "body"), leftIndent=2 * mm) for item in items],
        bulletType="1",
        leftIndent=6 * mm,
        bulletFontName="STSong-Light",
        bulletFontSize=8,
    )


def section_title(kicker: str, title: str) -> list:
    return [p(kicker.upper(), "label"), p(title, "h1")]


def bilingual(left: list, right: list) -> Table:
    table = Table(
        [[left, right]],
        colWidths=[126 * mm, 126 * mm],
        hAlign="LEFT",
    )
    table.setStyle(
        TableStyle(
            [
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (0, 0), 0),
                ("RIGHTPADDING", (0, 0), (0, 0), 5 * mm),
                ("LEFTPADDING", (1, 0), (1, 0), 5 * mm),
                ("RIGHTPADDING", (1, 0), (1, 0), 0),
                ("LINEBEFORE", (1, 0), (1, 0), 0.5, LINE),
            ]
        )
    )
    return table


def callout(zh: str, en: str) -> Table:
    table = Table(
        [[[p(zh, "callout"), p(en, "callout")]]],
        colWidths=[252 * mm],
        hAlign="LEFT",
    )
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), PALE_ACCENT),
                ("LINEBEFORE", (0, 0), (0, 0), 4, ACCENT),
                ("BOX", (0, 0), (-1, -1), 0.5, LINE),
                ("LEFTPADDING", (0, 0), (-1, -1), 4 * mm),
                ("RIGHTPADDING", (0, 0), (-1, -1), 4 * mm),
                ("TOPPADDING", (0, 0), (-1, -1), 3 * mm),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 3 * mm),
            ]
        )
    )
    return table


def data_table(headers: list[str], rows: list[list[str]], widths: list[float]) -> Table:
    data = [[p(value, "table_head") for value in headers]]
    data.extend([[p(value, "table") for value in row] for row in rows])
    table = Table(data, colWidths=[width * mm for width in widths], repeatRows=1)
    commands = [
        ("BACKGROUND", (0, 0), (-1, 0), DARK),
        ("GRID", (0, 0), (-1, -1), 0.4, LINE),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 2.2 * mm),
        ("RIGHTPADDING", (0, 0), (-1, -1), 2.2 * mm),
        ("TOPPADDING", (0, 0), (-1, -1), 1.7 * mm),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1.7 * mm),
    ]
    for row_index in range(2, len(data), 2):
        commands.append(("BACKGROUND", (0, row_index), (-1, row_index), PALE))
    table.setStyle(TableStyle(commands))
    return table


def on_page(canvas, doc) -> None:
    canvas.saveState()
    canvas.setStrokeColor(LINE)
    canvas.setLineWidth(0.4)
    canvas.line(16 * mm, 9 * mm, PAGE_WIDTH - 16 * mm, 9 * mm)
    canvas.setFont("STSong-Light", 7)
    canvas.setFillColor(MUTED)
    canvas.drawString(16 * mm, 5.5 * mm, "Agent + Skill Architecture Review")
    canvas.drawRightString(
        PAGE_WIDTH - 16 * mm,
        5.5 * mm,
        f"{doc.page:02d}",
    )
    canvas.restoreState()


story: list = []

# Cover
story.extend(
    [
        Spacer(1, 36 * mm),
        p("DUAL V2.2 / ROBOT SOUL / ARCHITECTURE REVIEW", "label"),
        p("Agent + Skill 架构梳理", "cover_title"),
        p("Agent + Skill Architecture Review", "cover_subtitle"),
        Spacer(1, 5 * mm),
        callout(
            "从“VLM 直接调用函数”到“Plan → Approval → Task → Executor”的受监督机器人运行时。",
            "From direct VLM function calling to a supervised robot runtime built around Plan → Approval → Task → Executor.",
        ),
        Spacer(1, 16 * mm),
        p("当前实现 / Current implementation: dual_v2_2_agent", "cover_subtitle"),
        p("审查日期 / Review date: 2026-08-27", "cover_subtitle"),
        PageBreak(),
    ]
)

# Executive summary
story.extend(section_title("Executive Summary", "核心判断 / Executive Assessment"))
story.append(
    bilingual(
        [
            p("中文", "label"),
            p("<b>新架构的本质不是“换了一个模型”。</b>它重新定义了模型、网页、Agent、Skill、Tool 与底层控制之间的权限边界。"),
            p("传统 VLM + Functions 模式让模型选择并调用函数，规划和执行容易发生在同一链路中。当前实现让模型或规则 Planner 只产生高层语义 Plan；Plan 本身不运动机器人，必须经过白名单校验、现场检查和逐 Plan 批准，才会创建 Hardware Task。"),
            p("Python Agent 是机械臂、相机与感知模型的唯一软件所有者。Skill 负责完整任务策略，Tool 负责原子能力，RobotRAM 继续承担已验证的感知、规划与执行。"),
            p("<b>最大收益：</b>安全权威分离、任务可观测、能力可复用、故障可恢复。"),
            p("<b>主要代价：</b>进程和状态更多、策略存在双份校验、Planner 泛化能力仍有限。"),
        ],
        [
            p("ENGLISH", "label"),
            p("<b>The new architecture is not merely a model replacement.</b> It reallocates authority across the model, UI, Agent, Skills, Tools, and control stack."),
            p("In a conventional VLM + Functions design, the model chooses functions and planning may flow directly into dispatch. The current implementation allows a rule-based or LLM Planner to propose only a high-level semantic Plan. A Plan is inert until allowlist validation, on-site checks, and per-Plan approval create a Hardware Task."),
            p("The Python Agent is the sole software owner of arms, cameras, and perception models. Skills own task strategy, Tools expose atomic capabilities, and RobotRAM continues to provide the validated perception, planning, and execution stack."),
            p("<b>Primary gains:</b> separated safety authority, observability, reuse, and recovery."),
            p("<b>Main costs:</b> more processes and state, duplicated policy checks, and limited planner generality."),
        ],
    )
)
story.extend(
    [
        Spacer(1, 4 * mm),
        data_table(
            ["指标 / Metric", "当前实现 / Current implementation"],
            [
                ["高层能力", "5 个内建 Skills；其中 4 个进入网页/Harness 真机执行白名单。"],
                ["原子能力", "3 个感知 Tools + 5 个操作 Tools。"],
                ["硬件所有权", "1 个 Python Agent 持有机械臂、相机与感知模型。"],
                ["启动模式", "start：直接真机；console：网页在线、硬件 offline，现场一键启动。"],
            ],
            [55, 197],
        ),
        PageBreak(),
    ]
)

# Layers
story.extend(section_title("Layered Architecture", "层次结构 / System Layers"))
story.append(
    data_table(
        ["层 / Layer", "实现 / Implementation", "职责 / Responsibility"],
        [
            ["01 操作入口", "Robot Soul GUI + Harness", "目标输入、Plan 审查、现场审批、Task Timeline。 Interfaces for goals, review, approval, and status."],
            ["02 策略与审批", "TypeScript + Python gates", "Skill 白名单、参数、动作预算、Plan TTL、checklist 与确认口令。"],
            ["03 Agent Server", "robot_agent/server.py", "Plan/Task、硬件锁、异步执行、事件、取消与审批请求。"],
            ["04 Planner", "robot_agent/planner.py", "规则优先、可选 LLM；自然语言 → SkillCall Plan。"],
            ["05 Executor", "robot_agent/executor.py", "依赖校验、执行时语义解析、逐 Skill 执行与事件发送。"],
            ["06 Skill", "robot_agent/skills/", "完整任务策略：感知、抓放、整理、恢复和启动。"],
            ["07 Tool", "robot_agent/tools/", "3 个感知 + 5 个操作元动作；统一返回 ToolResult。"],
            ["08 身体执行层", "RobotRAM + CuRobo/Upper/RT", "Step1–5、轨迹规划、UDP 控制与物理硬件。"],
        ],
        [38, 68, 146],
    )
)
story.append(Spacer(1, 4 * mm))
story.append(
    bilingual(
        [
            p("中文职责边界", "label"),
            bullets(
                [
                    "<b>Plan：</b>纯数据，包含 instruction、SkillCall 与 constraints。",
                    "<b>Skill：</b>任务级组合，封装顺序、前置条件、失败恢复与业务语义。",
                    "<b>Tool：</b>确定性原子能力，是进入 RobotRAM 的受控适配层。",
                    "<b>RobotContext：</b>持有相机、机械臂、模型、标定和当前场景状态。",
                    "<b>RobotRAM：</b>底层能力被复用，而非被重新实现。",
                ]
            ),
        ],
        [
            p("ENGLISH RESPONSIBILITIES", "label"),
            bullets(
                [
                    "<b>Plan:</b> inert data containing instruction, SkillCalls, and constraints.",
                    "<b>Skill:</b> task composition owning order, preconditions, recovery, and semantics.",
                    "<b>Tool:</b> deterministic atomic capability and controlled RobotRAM adapter.",
                    "<b>RobotContext:</b> owns arms, cameras, models, calibration, and scene state.",
                    "<b>RobotRAM:</b> existing low-level capabilities are reused, not rewritten.",
                ]
            ),
        ],
    )
)
story.extend(
    [
        callout(
            "关键不变量：模型不能提交关节角、CAN、shell、任意坐标或任意原子调用。",
            "Key invariant: the model cannot submit joints, CAN, shell, arbitrary poses, or arbitrary atomic calls.",
        ),
        PageBreak(),
    ]
)

# Flow
story.extend(section_title("Execution Flow", "真实执行链路 / End-to-End Execution"))
story.append(
    data_table(
        ["阶段 / Stage", "中文说明", "English"],
        [
            ["1 Goal", "用户输入“整理桌面”。", "The operator submits “Clean the table”."],
            ["2 Plan", "Planner 生成 tidy_table SkillCall；不初始化硬件。", "The Planner emits a tidy_table SkillCall without hardware bootstrap."],
            ["3 Approval", "校验白名单、TTL、现场 checklist 与确认口令。", "Validate allowlist, TTL, on-site checklist, and confirmation phrase."],
            ["4 Task", "Server 创建唯一活动 Task，并获取 hardware lock。", "The Server creates the sole active Task and acquires the hardware lock."],
            ["5 Perception", "Ground → Estimate → Localize，建立当前场景。", "Ground → Estimate → Localize builds the current scene."],
            ["6 Skill", "tidy_table 过滤目标，逐项调用 pick_and_place。", "tidy_table filters candidates and invokes pick_and_place."],
            ["7 Tools", "Approach、Grasp、Transport、Place、Reset。", "Atomic approach, grasp, transport, place, and reset operations."],
            ["8 Hardware", "Step5 → RobotArm UDP → CuRobo / RT Control。", "Step5 → RobotArm UDP → CuRobo / RT Control."],
        ],
        [30, 111, 111],
    )
)
story.append(Spacer(1, 4 * mm))
story.append(
    bilingual(
        [
            p("中文", "label"),
            p("<b>执行时解析物体：</b>Plan 使用 {category, color} 语义引用。感知后，Executor 才解析为 object_2 等临时 ID，因此 Plan 不绑定某一检测帧。"),
            p("<b>整理桌面展开：</b>Planner 只提交一个 tidy_table Skill。Skill 感知场景、过滤缺失物体，再对碗、面包和杯子逐项调用 pick_and_place。"),
            p("<b>错误流：</b>ToolResult → Skill → Executor → Server Task → 网页 Timeline，可指出物体、目标、动作、手臂、位置与底层反馈。"),
        ],
        [
            p("ENGLISH", "label"),
            p("<b>Execution-time object resolution:</b> Plans carry {category, color}; after perception, the Executor resolves transient object IDs, keeping Plans frame-independent."),
            p("<b>Tidying expansion:</b> the Planner emits one tidy_table Skill. The Skill perceives, filters unavailable objects, and invokes pick_and_place for bowl, bread, and cup."),
            p("<b>Error flow:</b> ToolResult → Skill → Executor → Server Task → Web Timeline, preserving object, target, action, arm, pose, and low-level feedback."),
        ],
    )
)
story.extend(
    [
        callout(
            "Plan 不是 Task；Task 也不是直接的原子动作。每一层都进一步收窄权限并增加验证。",
            "A Plan is not a Task, and a Task is not a raw atomic command. Each layer narrows authority and adds validation.",
        ),
        PageBreak(),
    ]
)

# Comparison
story.extend(section_title("Architecture Comparison", "VLM + Functions 与 Agent + Skill 对比"))
story.append(
    data_table(
        ["维度 / Dimension", "传统 VLM + Functions", "当前 Agent + Skill", "价值 / Impact"],
        [
            ["模型输出", "直接选择 approach / grasp / place 等函数。", "只提交高层 Skill Plan。", "意图与确定性运动分离。 Intent is separated from deterministic motion."],
            ["执行触发", "推理后可直接 dispatch。", "Plan → 审查 → 批准 → Task。", "Plan 可拒绝、过期、审计，并且本身不运动。"],
            ["硬件所有权", "多个脚本或 bridge 可能进入硬件。", "Python Agent 是唯一硬件所有者。", "避免相机、机械臂、GPU 和控制状态冲突。"],
            ["能力抽象", "函数薄包装，流程散落在 prompt 或脚本。", "Skill 封装策略；Tool 封装原子动作。", "能力可测试、组合、版本化和授权。"],
            ["物体引用", "较早绑定 object_0 / object_1。", "语义引用在感知后解析。", "Plan 与临时场景编号解耦。"],
            ["安全边界", "依赖 prompt 与操作纪律。", "白名单、参数、锁、预算和现场批准。", "安全由代码实施，不依赖模型自觉。"],
            ["可观测性", "以日志为主，上下文易丢失。", "Plan、Task、step、placement、operation 事件链。", "能定位具体物体与失败环节。"],
            ["迁移成本", "直接运行 RobotRAM。", "RobotRAM 被 Tool/Skill 包装后复用。", "保留已验证能力，避免推倒重写。"],
        ],
        [36, 67, 76, 73],
    )
)
story.append(Spacer(1, 5 * mm))
story.append(
    callout(
        "<b>架构意图：</b>从“VLM/LLM 作为遥控器”转向“模型负责意图，Agent 负责状态与执行，人类负责逐 Plan 批准”。",
        "<b>Architectural intent:</b> move from “the model as remote control” to “model for intent, Agent for state/execution, human for per-Plan approval.”",
    )
)
story.append(PageBreak())

# Launcher
story.extend(section_title("Lifecycle & Supervision", "launch_robot_soul.sh 增加的生命周期能力"))
story.append(
    data_table(
        ["能力 / Capability", "中文", "English"],
        [
            ["统一装配", "预检、构建 Harness Web、准备配置、共享 token，并按顺序启动硬件面、Harness、GUI 与 Supervisor。", "Preflight, web build, configuration, shared token, and ordered startup."],
            ["双模式", "start 直接启动真机；console 只启动网页/Harness，硬件保持 offline。", "start boots hardware; console keeps hardware offline until on-site startup."],
            ["显式状态机", "offline → starting → healthy / faulted → recovering。", "UI availability and hardware readiness become separate states."],
            ["故障域分离", "GUI/Harness 失败触发完整停机；硬件失败则锁定硬件并保留网页。", "UI failure stops all; hardware failure preserves diagnostics while locking motion."],
            ["现场恢复", "网页启动与恢复要求确认口令、生命周期锁和 sudo 授权。", "Startup and recovery require confirmation, lifecycle locking, and sudo authorization."],
            ["统一退出", "按顺序清理 Agent、Planner、Upper 与 RT，减少残留进程。", "Coordinated shutdown reduces stale processes and ownership conflicts."],
        ],
        [42, 105, 105],
    )
)
story.append(Spacer(1, 5 * mm))
story.append(
    bilingual(
        [
            p("中文判断", "label"),
            p("<b>launch_robot_soul.sh 不只是启动脚本。</b>它把认知面、操作面与硬件面装配为具有健康检查、故障锁定和人工恢复能力的产品运行时。"),
            bullets(
                [
                    "Tier 1：Harness 或统一网页连续失败，执行完整安全停机。",
                    "Tier 2：硬件面连续失败，停止并锁定硬件，保留 Harness 与网页。",
                    "Console 模式允许先启动操作台，再由现场人员一键初始化真机栈。",
                ]
            ),
        ],
        [
            p("ENGLISH ASSESSMENT", "label"),
            p("<b>launch_robot_soul.sh is more than a startup script.</b> It assembles the cognitive, operator, and hardware planes into a supervised product runtime."),
            bullets(
                [
                    "Tier 1: repeated Harness or unified UI failure causes a complete safe shutdown.",
                    "Tier 2: repeated hardware failure locks and stops hardware while preserving diagnostics.",
                    "Console mode allows the operator surface to start before on-site hardware initialization.",
                ]
            ),
        ],
    )
)
story.append(PageBreak())

# Strengths and limits
story.extend(section_title("Strengths & Constraints", "优势与当前限制 / Strengths and Limits"))
story.append(
    bilingual(
        [
            p("优势 / STRENGTHS", "label"),
            bullets(
                [
                    "<b>安全权威分离：</b>模型提议、操作员批准、Agent 执行、控制栈驱动。",
                    "<b>可观测：</b>Plan ID、Task 状态、阶段事件与详细失败上下文。",
                    "<b>可测试：</b>Planner、Skill、策略、审批、Origin 与状态机可独立验证。",
                    "<b>可复用：</b>pick_and_place 可被单任务与 tidy_table 组合。",
                    "<b>渐进迁移：</b>保留 RobotRAM Step1–5 与控制栈。",
                    "<b>故障可恢复：</b>硬件 fault 不必杀死网页和 Harness。",
                ]
            ),
            p("English: separated authority, observability, testability, reusable Skills, incremental migration, and isolated recovery.", "small"),
        ],
        [
            p("限制 / LIMITS", "label"),
            bullets(
                [
                    "<b>Planner 较窄：</b>当前主要覆盖整理与单次抓放。",
                    "<b>整理仍有配方：</b>按感知过滤，但不是完全动态关系推导。",
                    "<b>感知误检：</b>有效 mask/位姿仍可能对应错误语义。",
                    "<b>场景可能过期：</b>移动物体后需要更新 world state 或重新感知。",
                    "<b>状态主要在内存：</b>重启会丢失短期 Plan、审批与历史。",
                    "<b>策略双份维护：</b>TS 与 Python 白名单、预算可能漂移。",
                    "<b>单 Task 串行：</b>暂无队列、优先级和调度。",
                    "<b>取消不是急停：</b>cancel_event 不能替代实体急停。",
                ]
            ),
            p("English: narrow planning, recipe-driven tidying, false positives, stale state, in-memory lifecycle, policy drift, and no queue.", "small"),
        ],
    )
)
story.append(Spacer(1, 4 * mm))
story.extend(
    [
        callout(
            "<b>直接结论：</b>当前系统还不是“通用自主机器人 Agent”；它是一个安全、可审查、可扩展的任务运行时，Planner 与世界模型仍处于早期阶段。",
            "<b>Direct conclusion:</b> this is not yet a general autonomous robot. It is a safe and extensible task runtime with an early-stage planner and world model.",
        ),
        PageBreak(),
    ]
)

# Roadmap
story.extend(section_title("Roadmap", "演进潜力 / Development Potential"))
roadmap_cells = [
    [
        p("短期 / SHORT TERM", "label"),
        numbered(
            [
                "使用单一 Skill Schema 生成 TS/Python 校验与动作预算。<br/>Generate both policy layers from one Skill Schema.",
                "每次放置后更新世界模型；被移动目标强制重新感知。<br/>Refresh world state after each placement.",
                "完整保留 operation、arm、action、pose 等结构化事件。<br/>Preserve structured operation metadata.",
                "持久化 Plan、Task、审批、事件和执行结果。<br/>Persist Plans, Tasks, approvals, events, and results.",
            ]
        ),
    ],
    [
        p("中期 / MID TERM", "label"),
        numbered(
            [
                "为 Skill 增加前置条件、后置条件、风险等级、版本与重试。<br/>Add pre/postconditions, risk, versions, and retries.",
                "将整理任务升级为场景驱动的动态 Skill DAG。<br/>Generate scene-driven Skill DAGs.",
                "加入仿真回放、碰撞预检和 Plan Diff。<br/>Add simulation replay, collision preflight, and Plan Diff.",
                "建立有限 Task 队列，同时保持硬件 Executor 串行。<br/>Queue Tasks while preserving serial hardware execution.",
            ]
        ),
    ],
    [
        p("长期 / LONG TERM", "label"),
        numbered(
            [
                "使用受 Skill Manifest 约束的 LLM Planner。<br/>Use manifest-constrained LLM planning.",
                "多个只规划 Agent 并行提出方案，由单一执行 Agent 仲裁。<br/>Allow many planners but one execution authority.",
                "建立跨机器人 Capability Profile 与版本兼容。<br/>Build cross-robot capability profiles.",
                "用执行结果与视觉复核形成闭环学习和主动恢复。<br/>Learn from outcomes and visual verification.",
            ]
        ),
    ],
]
roadmap = Table([roadmap_cells], colWidths=[84 * mm] * 3)
roadmap.setStyle(
    TableStyle(
        [
            ("VALIGN", (0, 0), (-1, -1), "TOP"),
            ("BOX", (0, 0), (-1, -1), 0.5, LINE),
            ("INNERGRID", (0, 0), (-1, -1), 0.5, LINE),
            ("TOPPADDING", (0, 0), (-1, -1), 4 * mm),
            ("BOTTOMPADDING", (0, 0), (-1, -1), 4 * mm),
            ("LEFTPADDING", (0, 0), (-1, -1), 4 * mm),
            ("RIGHTPADDING", (0, 0), (-1, -1), 4 * mm),
            ("BACKGROUND", (0, 0), (-1, -1), PALE),
            ("LINEABOVE", (0, 0), (-1, 0), 3, ACCENT),
        ]
    )
)
story.extend([roadmap, Spacer(1, 5 * mm)])
story.append(
    callout(
        "下一阶段不应继续增加更多“模型可直接调用的函数”，而应增加带契约、版本、风险和恢复策略的 Skill。",
        "The next phase should add contract-driven, versioned, risk-rated, recoverable Skills—not more functions directly exposed to the model.",
    )
)
story.append(PageBreak())

# Conclusion and source map
story.extend(section_title("Conclusion & Source Map", "结论与关键入口 / Conclusion and Source Map"))
story.append(
    bilingual(
        [
            p("必须保持的架构不变量", "label"),
            numbered(
                [
                    "Python Agent 始终是唯一硬件所有者。",
                    "模型只能提交语义 Plan，不能提交关节、CAN、shell 或任意原子调用。",
                    "Plan 与 Task 始终分离；真机运动始终经过独立现场批准。",
                    "软件取消与 Supervisor 不能替代独立实体急停。",
                ]
            ),
        ],
        [
            p("ARCHITECTURAL INVARIANTS", "label"),
            numbered(
                [
                    "The Python Agent remains the sole hardware owner.",
                    "The model submits semantic Plans, never joints, CAN, shell, or arbitrary calls.",
                    "Plan and Task remain separate; motion always requires on-site approval.",
                    "Software cancellation and supervisors never replace the physical emergency stop.",
                ]
            ),
        ],
    )
)
story.append(Spacer(1, 4 * mm))
story.append(
    data_table(
        ["路径 / Path", "职责 / Responsibility"],
        [
            ["launch_robot_soul.sh", "统一装配、start/console、Supervisor、硬件启动与恢复。 Runtime assembly and supervision."],
            ["robot_agent/server.py", "Plan、Task、审批、硬件锁、事件与唯一硬件所有权。"],
            ["robot_agent/planner.py", "规则优先、可选 LLM 的自然语言到 Skill Plan。"],
            ["robot_agent/executor.py", "依赖校验、语义解析与逐 Skill 执行。"],
            ["robot_agent/skills/", "感知、抓放、整理、复位和启动等任务级能力。"],
            ["robot_agent/tools/", "感知与操作原子能力，适配 RobotRAM。"],
            ["RobotRAM/scripts/step1…step5", "现有感知、位姿估计、抓取规划和执行。"],
        ],
        [78, 174],
    )
)
story.append(Spacer(1, 5 * mm))
story.append(
    callout(
        "<b>最终判断：</b>Agent + Skill 架构最大的长期价值，是在不放弃现有机器人能力的前提下，把自然语言智能、确定性执行和物理安全变成三个可以独立演进的系统。",
        "<b>Final assessment:</b> the long-term value is the ability to evolve language intelligence, deterministic execution, and physical safety independently while preserving existing robot capabilities.",
    )
)

doc = SimpleDocTemplate(
    str(OUTPUT),
    pagesize=landscape(A4),
    leftMargin=16 * mm,
    rightMargin=16 * mm,
    topMargin=13 * mm,
    bottomMargin=13 * mm,
    title="Agent + Skill 架构梳理 / Agent + Skill Architecture Review",
    author="Cursor",
    subject="dual_v2_2_agent architecture review",
)
doc.build(story, onFirstPage=on_page, onLaterPages=on_page)
print(OUTPUT)
