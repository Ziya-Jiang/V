/** Self-contained interface for preview-gated Python Agent and legacy dry-run modes. */
export const ROBOT_ASSISTANT_HTML = `<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Assistant</title>
  <style>
    :root { color-scheme: dark; font-family: Inter, ui-sans-serif, system-ui, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; color: #e3edf5; background: #071018; }
    body::before { content: ""; position: fixed; inset: 0; pointer-events: none; opacity: .16;
      background-image: linear-gradient(#4e8fac24 1px, transparent 1px), linear-gradient(90deg, #4e8fac24 1px, transparent 1px);
      background-size: 32px 32px; }
    main { position: relative; width: min(1240px, calc(100% - 32px)); margin: 0 auto; padding: 28px 0 52px; }
    header { display: flex; align-items: end; justify-content: space-between; gap: 20px; padding-bottom: 22px; border-bottom: 1px solid #294154; }
    h1, h2, p { margin-top: 0; }
    h1 { margin-bottom: 0; font-size: 24px; letter-spacing: .08em; font-weight: 680; }
    h2 { margin-bottom: 8px; font-size: 18px; }
    .eyebrow, .label { color: #78b9d7; text-transform: uppercase; letter-spacing: .16em; font-size: 11px; }
    .agent-status { display: flex; align-items: center; gap: 9px; color: #aac1d0; font-size: 13px; }
    .dot { width: 9px; height: 9px; border-radius: 50%; background: #e2b65e; box-shadow: 0 0 12px #e2b65e80; }
    .dot.ready { background: #45d39c; box-shadow: 0 0 12px #45d39c80; }
    .dot.error { background: #ef6b72; box-shadow: 0 0 12px #ef6b7280; }
    .layout { display: grid; grid-template-columns: minmax(0, 1.55fr) minmax(300px, .75fr); gap: 18px; margin-top: 18px; }
    .stack { display: grid; gap: 18px; align-content: start; }
    .panel { border: 1px solid #294154; background: #0b1721ed; }
    .panel-head { display: flex; align-items: center; justify-content: space-between; gap: 12px; padding: 14px 16px; border-bottom: 1px solid #294154; }
    .panel-body { padding: 16px; }
    .muted { color: #86a0b1; font-size: 12px; line-height: 1.55; }
    form { display: grid; gap: 12px; }
    textarea, input { width: 100%; border: 1px solid #315269; border-radius: 2px; color: #eef7fb; background: #08131c; padding: 12px 14px; outline: none; font: inherit; }
    textarea { min-height: 96px; resize: vertical; line-height: 1.5; }
    textarea:focus, input:focus { border-color: #69b6d9; }
    button { min-height: 40px; border: 1px solid #69b6d9; border-radius: 2px; color: #071018; background: #69b6d9; padding: 0 18px; font-weight: 720; cursor: pointer; }
    button.secondary { color: #c8dce8; background: transparent; }
    button.danger { color: #ffd9dc; border-color: #b95058; background: #461c22; }
    button:disabled { opacity: .42; cursor: not-allowed; }
    .error-banner { display: none; margin-top: 18px; padding: 12px 14px; color: #ffd6d9; border: 1px solid #7b343a; background: #35161b; font-size: 13px; }
    .error-banner.visible { display: block; }
    #planPanel, #confirmationPanel, #activeTask { display: none; }
    #planPanel.visible, #confirmationPanel.visible, #activeTask.visible { display: block; }
    .plan-meta { color: #6fa9c4; font: 11px ui-monospace, monospace; word-break: break-all; }
    .steps { display: grid; gap: 9px; margin: 14px 0 0; padding: 0; list-style: none; }
    .step { display: grid; grid-template-columns: 30px 1fr; gap: 10px; padding: 11px; border: 1px solid #223947; background: #09151e; }
    .step-index { display: grid; place-items: center; width: 28px; height: 28px; border: 1px solid #3d657b; color: #7fc0dc; font: 12px ui-monospace, monospace; }
    .step strong { display: block; margin-bottom: 4px; font-size: 13px; }
    .step code { color: #88a5b7; white-space: pre-wrap; word-break: break-word; font-size: 11px; }
    .constraints { display: flex; flex-wrap: wrap; gap: 7px; margin-top: 13px; }
    .constraint { padding: 5px 8px; border: 1px solid #4d4531; color: #e2c887; background: #211e16; font-size: 11px; }
    details { margin-top: 12px; color: #89a9ba; font-size: 12px; }
    pre { max-height: 240px; overflow: auto; padding: 12px; color: #a9c7d7; background: #071119; white-space: pre-wrap; word-break: break-word; font-size: 11px; }
    .confirmation { border-color: #756133; }
    .confirmation strong { color: #f2cc73; font: 12px ui-monospace, monospace; }
    .confirmation-actions, .task-actions { display: flex; gap: 10px; margin-top: 11px; }
    .warning { margin: 10px 0 0; color: #e6bd71; font-size: 12px; line-height: 1.55; }
    .task-card { padding: 14px 16px; border-left: 3px solid #62afd1; background: #0b1721; }
    .task-id { color: #8eb6ca; font: 11px ui-monospace, monospace; word-break: break-all; }
    #events { max-height: 520px; min-height: 260px; overflow: auto; padding: 4px 16px 12px; }
    .event { display: grid; grid-template-columns: 76px minmax(100px, 150px) 1fr; gap: 11px; padding: 11px 0; border-bottom: 1px solid #1b2c39; font-size: 12px; }
    .event time { color: #66879b; font-variant-numeric: tabular-nums; }
    .event .type { color: #78b9d7; word-break: break-word; }
    .event .message { color: #bed0db; line-height: 1.45; }
    .event .skill-name { display: block; margin-bottom: 3px; color: #e3c779; font: 10px ui-monospace, monospace; }
    #skills { padding: 4px 16px 12px; }
    .skill { padding: 12px 0; border-bottom: 1px solid #1b2c39; }
    .skill:last-child { border: 0; }
    .skill strong { display: block; margin-bottom: 5px; font-size: 13px; }
    .skill p { margin: 0; color: #8da9ba; font-size: 12px; line-height: 1.5; }
    .operation { display: inline-block; margin-top: 7px; color: #78b9d7; font: 11px ui-monospace, monospace; }
    .notice { margin-top: 18px; padding: 14px 16px; color: #9ab5c5; border-left: 2px solid #e3b866; background: #0b1721; font-size: 12px; line-height: 1.6; }
    @media (max-width: 860px) {
      .layout { grid-template-columns: 1fr; }
      header { align-items: start; flex-direction: column; }
      .event { grid-template-columns: 68px 105px 1fr; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div><div class="eyebrow">Embodied intelligence console</div><h1>ROBOT ASSISTANT</h1></div>
      <div class="agent-status"><span id="agentDot" class="dot"></span><span id="agentStatus">AGENT LOADING</span></div>
    </header>
    <div id="errorBanner" class="error-banner" role="alert"></div>
    <section class="layout">
      <div class="stack">
        <section class="panel">
          <div class="panel-head"><span class="label">01 · 指令</span><span id="runtimeLabel">LOADING</span></div>
          <div class="panel-body">
            <form id="taskForm">
              <textarea id="taskInput" autocomplete="off" aria-label="任务指令">整理桌面上的杯子和碗</textarea>
              <button id="submitButton" type="submit">生成执行计划</button>
            </form>
          </div>
        </section>
        <section id="planPanel" class="panel">
          <div class="panel-head"><span class="label">02 · Plan 预览</span><span id="planId" class="plan-meta"></span></div>
          <div class="panel-body">
            <h2 id="planTitle">执行计划</h2>
            <p id="planSummary" class="muted"></p>
            <div id="constraints" class="constraints"></div>
            <ol id="planSteps" class="steps"></ol>
            <details><summary>原始 Plan 数据</summary><pre id="rawPlan"></pre></details>
          </div>
        </section>
        <section id="confirmationPanel" class="panel confirmation">
          <div class="panel-head"><span class="label">03 · 硬件执行确认</span><span>REQUIRED</span></div>
          <div class="panel-body">
            <p class="muted">检查 Plan、约束和现场安全条件后，键入以下完整口令才能提交执行：</p>
            <p><strong>ENABLE_DUAL_V2_2_HARDWARE</strong></p>
            <input id="confirmationInput" autocomplete="off" spellcheck="false" aria-label="硬件确认口令" placeholder="键入确认口令">
            <div class="confirmation-actions">
              <button id="confirmButton" type="button" disabled>确认并执行</button>
              <button id="discardButton" class="secondary" type="button">放弃此计划</button>
            </div>
            <p class="warning">确认仅授权本次计划。执行前仍应确认物理急停可用、人员已离开工作区。</p>
          </div>
        </section>
        <section id="activeTask" class="task-card">
          <div class="label">活动任务 · <span id="activeTaskStatus">QUEUED</span></div>
          <p id="activeTaskId" class="task-id"></p>
          <div class="task-actions"><button id="cancelButton" class="danger" type="button">请求取消任务</button></div>
          <p class="warning">“请求取消”不是物理急停。出现危险时请立即使用机器人实体急停装置。</p>
        </section>
      </div>
      <aside class="stack">
        <section class="panel">
          <div class="panel-head"><span class="label">Skill 时间线</span><span id="taskStatus">READY</span></div>
          <div id="events">
            <div class="event"><time>--:--:--</time><span class="type">system</span><span class="message">等待任务指令。</span></div>
          </div>
        </section>
        <section class="panel">
          <div class="panel-head"><span class="label">Runtime / Skills</span><span id="skillCount">0</span></div>
          <div id="skills"><p class="muted">正在读取运行时状态…</p></div>
        </section>
      </aside>
    </section>
    <div class="notice" id="safetyNotice">正在检查 Agent 连接。此界面不替代机器人安全控制和物理急停。</div>
  </main>
  <script>
    const CONFIRMATION = 'ENABLE_DUAL_V2_2_HARDWARE';
    const model = { runtime: null, planId: null, taskId: null, plan: null, agentReady: false };
    const element = (id) => document.getElementById(id);
    const form = element('taskForm');
    const input = element('taskInput');
    const submitButton = element('submitButton');
    const confirmInput = element('confirmationInput');
    const confirmButton = element('confirmButton');
    const discardButton = element('discardButton');
    const cancelButton = element('cancelButton');
    const events = element('events');

    const showError = (message) => {
      const banner = element('errorBanner');
      banner.textContent = message;
      banner.classList.add('visible');
    };
    const clearError = () => {
      element('errorBanner').classList.remove('visible');
    };
    const requestJson = async (path, options) => {
      const response = await fetch(path, options);
      const payload = await response.json().catch(() => ({ error: '服务器返回了非 JSON 响应。' }));
      if (!response.ok) throw new Error(payload.error || ('HTTP ' + response.status));
      return payload;
    };
    const displayValue = (value) => {
      if (typeof value === 'string') return value;
      try { return JSON.stringify(value); } catch { return String(value); }
    };
    const appendEvent = (event) => {
      const row = document.createElement('div');
      row.className = 'event';
      const timeNode = document.createElement('time');
      timeNode.textContent = new Date(event.timestamp || Date.now()).toLocaleTimeString('zh-CN', { hour12: false });
      const typeNode = document.createElement('span');
      typeNode.className = 'type';
      typeNode.textContent = event.type || 'task.event';
      const messageNode = document.createElement('span');
      messageNode.className = 'message';
      const skill = event.skill || event.skillId || event.name || event.stepId;
      if (skill) {
        const skillNode = document.createElement('span');
        skillNode.className = 'skill-name';
        skillNode.textContent = displayValue(skill);
        messageNode.append(skillNode);
      }
      messageNode.append(document.createTextNode(event.message || event.detail || event.type || '事件'));
      row.append(timeNode, typeNode, messageNode);
      events.append(row);
      events.scrollTop = events.scrollHeight;
      if (event.status) {
        element('taskStatus').textContent = String(event.status).toUpperCase();
        element('activeTaskStatus').textContent = String(event.status).toUpperCase();
      }
      const terminal = ['succeeded', 'success', 'completed', 'failed', 'error', 'cancelled', 'canceled'];
      if (event.status && terminal.includes(String(event.status).toLowerCase())) {
        model.taskId = null;
        element('activeTask').classList.remove('visible');
        submitButton.disabled = false;
        window.setTimeout(refreshState, 100);
      }
    };
    const collectConstraints = (plan, steps) => {
      const values = [];
      const add = (constraints) => {
        if (Array.isArray(constraints)) {
          values.push(...constraints);
        } else if (constraints && typeof constraints === 'object') {
          for (const [key, value] of Object.entries(constraints)) {
            values.push(key + ': ' + displayValue(value));
          }
        }
      };
      add(plan && plan.constraints);
      for (const step of steps) {
        add(step && step.constraints);
      }
      return values;
    };
    const renderPlan = (plan, planId) => {
      model.plan = plan;
      model.planId = planId || (plan && plan.id) || null;
      element('planPanel').classList.add('visible');
      element('planId').textContent = model.planId || 'DRY-RUN';
      element('planTitle').textContent =
        (plan && (plan.title || plan.intent || plan.instruction)) || '执行计划';
      const steps = Array.isArray(plan && plan.steps) ? plan.steps : Array.isArray(plan) ? plan : [];
      element('planSummary').textContent = steps.length
        ? '共 ' + steps.length + ' 个步骤。请逐项检查后再确认。'
        : 'Plan 未提供标准 steps 数组，请检查下方原始数据。';
      const constraintsNode = element('constraints');
      constraintsNode.replaceChildren();
      for (const constraint of collectConstraints(plan, steps)) {
        const node = document.createElement('span');
        node.className = 'constraint';
        node.textContent = typeof constraint === 'object' && constraint
          ? displayValue(constraint.description || constraint.kind || constraint)
          : displayValue(constraint);
        constraintsNode.append(node);
      }
      const stepsNode = element('planSteps');
      stepsNode.replaceChildren();
      steps.forEach((step, index) => {
        const item = document.createElement('li');
        item.className = 'step';
        const number = document.createElement('span');
        number.className = 'step-index';
        number.textContent = String(index + 1).padStart(2, '0');
        const body = document.createElement('div');
        const title = document.createElement('strong');
        title.textContent = (step && (step.title || step.skill || step.skillId || step.name || step.operation)) || ('步骤 ' + (index + 1));
        const detail = document.createElement('code');
        const args = step && (step.args || step.input || step.parameters || step.description);
        detail.textContent = args === undefined ? '' : displayValue(args);
        body.append(title, detail);
        item.append(number, body);
        stepsNode.append(item);
      });
      element('rawPlan').textContent = JSON.stringify(plan, null, 2);
    };
    const renderSkills = (skills, fallback) => {
      const container = element('skills');
      container.replaceChildren();
      if (!Array.isArray(skills) || skills.length === 0) {
        const node = document.createElement('p');
        node.className = 'muted';
        node.textContent = fallback;
        container.append(node);
        element('skillCount').textContent = '—';
        return;
      }
      element('skillCount').textContent = String(skills.length);
      for (const skill of skills) {
        const node = document.createElement('div');
        node.className = 'skill';
        const title = document.createElement('strong');
        title.textContent = skill.title || skill.name || skill.id || 'Skill';
        const description = document.createElement('p');
        description.textContent = skill.description || '';
        const operation = document.createElement('span');
        operation.className = 'operation';
        operation.textContent = skill.operation || skill.id || '';
        node.append(title, description, operation);
        container.append(node);
      }
    };
    const setAgentState = (agent) => {
      const dot = element('agentDot');
      model.agentReady = Boolean(agent && agent.ready && agent.status === 'ready');
      dot.className = 'dot ' + (agent && agent.status ? agent.status : '');
      if (!agent || agent.status === 'loading') element('agentStatus').textContent = 'AGENT LOADING';
      else if (agent.status === 'ready') element('agentStatus').textContent = 'AGENT READY';
      else element('agentStatus').textContent = 'AGENT ERROR';
      confirmButton.disabled =
        confirmInput.value !== CONFIRMATION || !model.planId || !model.agentReady;
      if (agent && agent.error) showError(agent.error);
    };
    const applyServerState = (state) => {
      model.runtime = state.runtime || 'mvp';
      if (model.runtime === 'python-agent') {
        element('runtimeLabel').textContent = 'PYTHON AGENT';
        submitButton.textContent = '生成执行计划';
        setAgentState(state.agent);
        renderSkills(
          state.agent && state.agent.details && state.agent.details.skills,
          'Skills 由 Python Agent 管理；执行事件会显示在上方时间线。',
        );
        element('safetyNotice').textContent =
          'Python Agent 仅通过 127.0.0.1 NDJSON 接入。执行必须先预览 Plan，再输入完整硬件确认口令。取消请求不是物理急停。';
        if (state.activePreview) submitButton.disabled = true;
        if (state.activePreview && state.activePreview.plan && !model.planId) {
          renderPlan(state.activePreview.plan, state.activePreview.planId);
          element('confirmationPanel').classList.add('visible');
        }
        if (state.activeTask) {
          model.taskId = state.activeTask.taskId;
          element('activeTask').classList.add('visible');
          element('activeTaskId').textContent = state.activeTask.taskId;
          element('activeTaskStatus').textContent = String(state.activeTask.status || 'running').toUpperCase();
          submitButton.disabled = true;
        } else if (!state.activePreview && !model.planId) {
          submitButton.disabled = false;
        }
      } else {
        const planner = state.plannerMode === 'deepseek' ? 'DEEPSEEK' : 'DETERMINISTIC';
        const driver = state.driverMode === 'dry-run' ? 'DRY-RUN' : 'ROBOTRAM';
        element('runtimeLabel').textContent = planner + ' · ' + driver;
        element('agentStatus').textContent = planner + ' READY';
        element('agentDot').className = 'dot ready';
        submitButton.textContent = '规划并执行';
        renderSkills(state.skills, '未挂载 Skill。');
        element('safetyNotice').textContent = state.driverMode === 'dry-run'
          ? '当前为旧 Dry Run MVP：创建计划后会自动模拟执行，不发送真实机器人命令。'
          : '旧 RobotRAM Bridge 模式已启用；请遵循其独立安全约束。';
        if (state.activeTaskId) submitButton.disabled = true;
      }
    };
    const refreshState = async () => {
      try {
        applyServerState(await requestJson('/api/state'));
      } catch (error) {
        setAgentState({ status: 'error', error: String(error) });
      }
    };

    form.addEventListener('submit', async (event) => {
      event.preventDefault();
      clearError();
      submitButton.disabled = true;
      appendEvent({ type: 'user.request', message: input.value, timestamp: new Date().toISOString(), status: 'planning' });
      try {
        const payload = await requestJson('/api/tasks', {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ text: input.value, instruction: input.value }),
        });
        renderPlan(payload.plan, payload.planId);
        if (payload.planId) {
          element('confirmationPanel').classList.add('visible');
          confirmInput.value = '';
          confirmButton.disabled = true;
          element('taskStatus').textContent = 'AWAITING CONFIRMATION';
        } else {
          model.taskId = payload.plan && payload.plan.id;
          element('taskStatus').textContent = 'RUNNING';
        }
      } catch (error) {
        showError(String(error));
        appendEvent({ type: 'request.failed', message: String(error), timestamp: new Date().toISOString(), status: 'failed' });
        submitButton.disabled = false;
      }
    });
    confirmInput.addEventListener('input', () => {
      confirmButton.disabled =
        confirmInput.value !== CONFIRMATION || !model.planId || !model.agentReady;
    });
    confirmButton.addEventListener('click', async () => {
      clearError();
      confirmButton.disabled = true;
      try {
        const payload = await requestJson('/api/tasks/' + encodeURIComponent(model.planId) + '/confirm', {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ confirmation: confirmInput.value }),
        });
        model.planId = null;
        model.taskId = payload.taskId;
        element('confirmationPanel').classList.remove('visible');
        element('activeTask').classList.add('visible');
        element('activeTaskId').textContent = payload.taskId;
        element('activeTaskStatus').textContent = 'QUEUED';
      } catch (error) {
        showError(String(error));
        confirmButton.disabled = false;
      }
    });
    discardButton.addEventListener('click', async () => {
      if (!model.planId) return;
      clearError();
      discardButton.disabled = true;
      try {
        await requestJson('/api/tasks/' + encodeURIComponent(model.planId), {
          method: 'DELETE',
        });
        model.planId = null;
        model.plan = null;
        confirmInput.value = '';
        element('confirmationPanel').classList.remove('visible');
        element('planPanel').classList.remove('visible');
        element('taskStatus').textContent = 'READY';
        submitButton.disabled = false;
        await refreshState();
      } catch (error) {
        showError(String(error));
      } finally {
        discardButton.disabled = false;
      }
    });
    cancelButton.addEventListener('click', async () => {
      if (!model.taskId) return;
      clearError();
      cancelButton.disabled = true;
      try {
        await requestJson('/api/tasks/' + encodeURIComponent(model.taskId) + '/cancel', {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: '{}',
        });
        element('activeTaskStatus').textContent = 'CANCELING';
      } catch (error) {
        showError(String(error));
      } finally {
        cancelButton.disabled = false;
      }
    });

    const stream = new EventSource('/api/events');
    stream.onmessage = ({ data }) => {
      try { appendEvent(JSON.parse(data)); } catch { showError('收到无法解析的 SSE 事件。'); }
    };
    stream.onerror = () => showError('事件流已中断，正在等待浏览器自动重连。');
    void refreshState();
    window.setInterval(refreshState, 2000);
  </script>
</body>
</html>`
