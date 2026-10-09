import { PRODUCT_CATALOG } from './catalog.ts'
import { SITE_CONTENT } from './content.ts'
import { HARDWARE_CONFIRMATION } from './live.ts'

const bootstrap = JSON.stringify({
  catalog: PRODUCT_CATALOG,
  content: SITE_CONTENT,
  confirmation: HARDWARE_CONFIRMATION,
}).replaceAll('<', '\\u003c')

export const ROBOT_SOUL_HTML = String.raw`<!doctype html>
<html lang="en" data-theme="light">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="description" content="HKCLR Harness Agent is an agentic operating layer that turns human intent and world context into auditable robot Skill plans.">
  <meta name="color-scheme" content="light dark">
  <meta name="theme-color" content="#fc4c02">
  <title>HKCLR Harness Agent — The agentic operating layer for robots</title>
  <style>
    :root {
      --bg: #f4f4f2;
      --surface: rgba(255, 255, 255, .9);
      --solid: #fff;
      --ink: #2d2926;
      --muted: #6d6b69;
      --line: rgba(45, 41, 38, .14);
      --blue: #fc4c02;
      --blue-2: #ffae62;
      --blue-wash: rgba(252, 76, 2, .1);
      --green: #087c55;
      --amber: #b44a0a;
      --red: #ba3047;
      --shadow: 0 22px 70px rgba(45, 41, 38, .12);
      --radius: 24px;
      --nav-h: 64px;
      --ease: cubic-bezier(.2, .8, .2, 1);
    }
    html[data-theme="dark"] {
      --bg: #151515;
      --surface: rgba(38, 38, 38, .86);
      --solid: #222;
      --ink: #fffaf4;
      --muted: #bbb8b5;
      --line: rgba(255, 255, 255, .12);
      --blue: #ff7040;
      --blue-2: #ffae62;
      --blue-wash: rgba(252, 76, 2, .16);
      --green: #40c887;
      --amber: #ffae62;
      --red: #ff6177;
      --shadow: 0 22px 80px rgba(0, 0, 0, .36);
    }
    * { box-sizing: border-box; }
    html { scroll-behavior: smooth; }
    body {
      margin: 0;
      color: var(--ink);
      background:
        linear-gradient(rgba(45, 41, 38, .035) 1px, transparent 1px),
        linear-gradient(90deg, rgba(45, 41, 38, .035) 1px, transparent 1px),
        var(--bg);
      background-size: 32px 32px;
      overflow-x: clip;
      font-family: Inter, ui-sans-serif, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      -webkit-font-smoothing: antialiased;
      line-height: 1.5;
    }
    button, input, textarea { font: inherit; }
    button { color: inherit; }
    a { color: inherit; }
    :focus-visible { outline: 3px solid color-mix(in srgb, var(--blue) 62%, transparent); outline-offset: 3px; }
    .skip {
      position: fixed; z-index: 100; top: 8px; left: 8px; transform: translateY(-160%);
      padding: 9px 13px; border-radius: 12px; color: white; background: var(--blue);
    }
    .skip:focus { transform: translateY(0); }
    .nav {
      position: fixed; z-index: 50; inset: 0 0 auto; height: var(--nav-h);
      border-bottom: 1px solid var(--line); background: color-mix(in srgb, var(--bg) 74%, transparent);
      backdrop-filter: saturate(180%) blur(22px);
    }
    .nav-inner {
      width: min(1180px, calc(100% - 32px)); height: 100%; margin: auto;
      display: flex; align-items: center; gap: 18px;
    }
    .brand { display: flex; align-items: center; gap: 10px; text-decoration: none; white-space: nowrap; }
    .brand-logo { width: 34px; height: 34px; border-radius: 50%; display: block; object-fit: cover; box-shadow: 0 0 0 1px var(--line); }
    .brand-copy { display: grid; line-height: 1.05; }
    .brand-copy strong { font-size: 13px; font-weight: 820; letter-spacing: .06em; }
    .brand-copy small { margin-top: 4px; color: var(--muted); font-size: 9px; font-weight: 680; letter-spacing: .12em; }
    .nav-links { display: flex; align-items: center; gap: 3px; margin: auto; }
    .nav-links a { padding: 7px 10px; border-radius: 999px; color: var(--muted); font-size: 12px; text-decoration: none; transition: .2s var(--ease); }
    .nav-links a:hover, .nav-links a[aria-current="true"] { color: var(--ink); background: var(--surface); }
    .nav-actions { display: flex; align-items: center; gap: 7px; }
    .icon-button, .live-button {
      border: 1px solid var(--line); background: var(--surface); cursor: pointer;
      height: 34px; border-radius: 999px; transition: .2s var(--ease);
    }
    .icon-button { width: 34px; display: grid; place-items: center; }
    .live-button { display: flex; align-items: center; gap: 7px; padding: 0 12px; font-size: 12px; }
    .icon-button:hover, .live-button:hover { transform: translateY(-1px); border-color: var(--blue); }
    .status-dot { width: 7px; height: 7px; border-radius: 50%; background: var(--muted); }
    .status-dot.connected { background: var(--green); box-shadow: 0 0 0 4px color-mix(in srgb, var(--green) 18%, transparent); }
    main { overflow: hidden; }
    section { position: relative; scroll-margin-top: calc(var(--nav-h) + 18px); }
    .section-shell { width: min(1220px, calc(100% - 40px)); margin: auto; padding: 112px 0; }
    .section-head { max-width: 860px; margin-bottom: 44px; }
    .eyebrow { margin: 0 0 13px; color: var(--blue); font-size: 12px; line-height: 1; font-weight: 780; letter-spacing: .12em; }
    h1, h2, h3, p { margin-top: 0; }
    h1 { margin-bottom: 22px; font-size: clamp(52px, 6.2vw, 82px); line-height: .96; letter-spacing: -.065em; font-weight: 760; }
    h2 { margin-bottom: 18px; max-width: 920px; font-size: clamp(38px, 5.2vw, 64px); line-height: 1.03; letter-spacing: -.052em; font-weight: 735; }
    h3 { margin-bottom: 10px; font-size: 22px; letter-spacing: -.035em; }
    .lede { max-width: 760px; color: var(--muted); font-size: clamp(18px, 2.2vw, 25px); line-height: 1.42; letter-spacing: -.025em; }
    .hero {
      min-height: 94svh; display: grid; place-items: center; padding-top: var(--nav-h);
      background:
        radial-gradient(circle at 78% 34%, rgba(252, 76, 2, .14), transparent 27%),
        radial-gradient(circle at 8% 78%, rgba(45, 41, 38, .07), transparent 30%);
    }
    .hero-inner { width: min(1180px, calc(100% - 40px)); display: grid; grid-template-columns: 1.08fr .92fr; align-items: center; gap: 60px; padding: 74px 0 58px; }
    .hero-copy { position: relative; z-index: 2; min-width: 0; }
    .hero h1 span { display: block; color: var(--blue); }
    .hero h1 .ink-line { color: var(--ink); background: none; }
    .hero-actions { display: flex; gap: 12px; flex-wrap: wrap; margin-top: 34px; }
    .button {
      min-height: 46px; padding: 0 20px; border: 0; border-radius: 999px; cursor: pointer;
      display: inline-flex; align-items: center; justify-content: center; gap: 9px; text-decoration: none;
      background: var(--blue); color: white; font-weight: 650; transition: .24s var(--ease);
    }
    .button:hover { transform: translateY(-2px); box-shadow: 0 10px 30px color-mix(in srgb, var(--blue) 26%, transparent); }
    .button.secondary { color: var(--ink); background: var(--solid); border: 1px solid var(--line); box-shadow: none; }
    .button.small { min-height: 36px; padding: 0 14px; font-size: 13px; }
    .button[disabled] { opacity: .48; cursor: not-allowed; transform: none; box-shadow: none; }
    .hero-visual { min-height: 520px; display: grid; place-items: center; position: relative; }
    .soul-button {
      width: min(430px, 86vw); min-height: 430px; padding: 28px; border: 1px solid var(--line); border-radius: 32px; cursor: pointer;
      position: relative; overflow: hidden; text-align: left; color: var(--ink);
      background:
        linear-gradient(rgba(45,41,38,.045) 1px, transparent 1px),
        linear-gradient(90deg, rgba(45,41,38,.045) 1px, transparent 1px),
        var(--solid);
      background-size: 32px 32px, 32px 32px, auto;
      box-shadow: 0 34px 90px rgba(45, 41, 38, .18);
      animation: float 6s var(--ease) infinite;
    }
    .soul-button::before, .soul-button::after {
      content: ""; position: absolute; border-radius: 50%; pointer-events: none;
      transition: .3s var(--ease);
    }
    .soul-button::before { width: 240px; height: 240px; top: -120px; right: -90px; background: rgba(252, 76, 2, .1); }
    .soul-button::after { width: 170px; height: 170px; bottom: -100px; left: -70px; background: rgba(45, 41, 38, .06); }
    .soul-button:hover, .soul-button[data-open="true"] { border-color: rgba(252, 76, 2, .48); transform: translateY(-4px); }
    .agent-card-head { position: relative; z-index: 2; display: flex; align-items: center; gap: 12px; }
    .agent-card-logo { width: 58px; height: 58px; border-radius: 50%; object-fit: cover; background: white; }
    .agent-card-brand { display: grid; }
    .agent-card-brand strong { font-size: 13px; letter-spacing: .08em; }
    .agent-card-brand small { color: var(--muted); font-size: 10px; letter-spacing: .08em; }
    .runtime-pill { margin-left: auto; padding: 5px 8px; border-radius: 999px; color: #a93400; background: rgba(252,76,2,.12); font-size: 9px; font-weight: 800; letter-spacing: .08em; }
    .agent-card-title { position: relative; z-index: 2; margin: 42px 0 22px; max-width: 310px; font-size: 34px; line-height: 1.03; font-weight: 760; letter-spacing: -.05em; }
    .agent-loop { position: relative; z-index: 2; display: grid; grid-template-columns: repeat(2, 1fr); gap: 9px; }
    .agent-loop span { padding: 12px; border: 1px solid var(--line); border-radius: 13px; color: var(--muted); background: color-mix(in srgb, var(--solid) 86%, var(--bg)); font-size: 10px; font-weight: 720; }
    .agent-loop b { display: block; margin-bottom: 4px; color: var(--blue); font: 800 9px ui-monospace, SFMono-Regular, Menlo, monospace; letter-spacing: .08em; }
    .agent-boundary { position: relative; z-index: 2; margin-top: 18px; padding-top: 14px; border-top: 1px solid var(--line); display: flex; justify-content: space-between; color: var(--muted); font-size: 10px; }
    .agent-boundary strong { color: var(--ink); }
    .core-label { position: absolute; z-index: 5; top: calc(50% + 182px); padding: 6px 10px; border-radius: 999px; color: var(--muted); background: var(--surface); box-shadow: var(--shadow); text-align: center; font-size: 9px; font-weight: 760; letter-spacing: .08em; pointer-events: none; }
    .orbit-chip {
      position: absolute; z-index: 3; padding: 8px 12px; border: 1px solid var(--line);
      border-radius: 999px; background: var(--surface); box-shadow: var(--shadow); backdrop-filter: blur(18px);
      font-size: 12px; font-weight: 650; pointer-events: none;
    }
    .orbit-chip.one { top: 18%; left: -2%; }
    .orbit-chip.two { top: 33%; right: -2%; }
    .orbit-chip.three { bottom: 18%; left: 3%; }
    .hero-footnote { position: absolute; bottom: 26px; color: var(--muted); font-size: 12px; text-align: center; }
    .stats {
      width: min(1040px, calc(100% - 40px)); margin: -42px auto 0; position: relative; z-index: 4;
      display: grid; grid-template-columns: repeat(4, 1fr); border: 1px solid var(--line); border-radius: var(--radius);
      background: var(--surface); box-shadow: var(--shadow); backdrop-filter: blur(24px);
    }
    .stat { padding: 24px; text-align: center; border-right: 1px solid var(--line); }
    .stat:last-child { border: 0; }
    .stat strong { display: block; font-size: 28px; letter-spacing: -.05em; }
    .stat span { color: var(--muted); font-size: 12px; }
    .control-center { background: var(--solid); }
    .control-status {
      display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin-bottom: 16px;
    }
    .control-status-card {
      padding: 16px 18px; border: 1px solid var(--line); border-radius: 18px; background: var(--bg);
    }
    .control-status-card span { display: block; color: var(--muted); font-size: 11px; }
    .control-status-card strong { display: block; margin-top: 5px; font-size: 14px; }
    .control-status-card strong.online { color: var(--green); }
    .control-status-card strong.offline { color: var(--red); }
    .control-status-card strong.pending { color: var(--amber); }
    .stack-startup {
      display: grid; grid-template-columns: minmax(0, 1fr) auto; gap: 18px; align-items: center;
      margin-bottom: 16px; padding: 20px; border: 1px solid color-mix(in srgb, var(--blue) 32%, var(--line));
      border-radius: 18px; background: color-mix(in srgb, var(--blue) 5%, var(--solid));
    }
    .stack-startup h3 { margin: 0 0 5px; font-size: 16px; }
    .stack-startup p { margin: 0; color: var(--muted); font-size: 13px; }
    .stack-startup button { white-space: nowrap; }
    .stack-fault {
      display: grid; grid-template-columns: minmax(0, 1fr) auto; gap: 18px; align-items: center;
      margin-bottom: 16px; padding: 18px 20px; border: 1px solid color-mix(in srgb, var(--red) 35%, var(--line));
      border-radius: 18px; background: color-mix(in srgb, var(--red) 7%, var(--solid));
    }
    .stack-fault[hidden] { display: none; }
    .stack-fault h3 { margin: 0 0 5px; font-size: 16px; }
    .stack-fault p { margin: 0; color: var(--muted); font-size: 13px; }
    .stack-fault button { white-space: nowrap; }
    .control-workspace {
      display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 14px;
    }
    .operation-step {
      min-height: 230px; padding: 26px; border: 1px solid var(--line); border-radius: 22px;
      display: flex; flex-direction: column; background: var(--bg); transition: .25s var(--ease);
    }
    .operation-step:hover { transform: translateY(-4px); border-color: color-mix(in srgb, var(--blue) 40%, var(--line)); box-shadow: var(--shadow); }
    .operation-step .step-number { color: var(--blue); font: 760 12px/1 ui-monospace, SFMono-Regular, Menlo, monospace; letter-spacing: .08em; }
    .operation-step h3 { margin-top: 24px; }
    .operation-step p { color: var(--muted); font-size: 14px; }
    .operation-step .button { align-self: flex-start; margin-top: auto; }
    .control-note {
      grid-column: 1 / -1; display: flex; gap: 12px; align-items: flex-start;
      padding: 16px 18px; border: 1px solid color-mix(in srgb, var(--amber) 28%, var(--line));
      border-radius: 16px; color: var(--muted); background: color-mix(in srgb, var(--amber) 6%, var(--solid)); font-size: 13px;
    }
    .control-note strong { color: var(--ink); white-space: nowrap; }
    .store-toolbar {
      display: grid; grid-template-columns: minmax(220px, 1fr) auto; gap: 16px; align-items: center; margin-bottom: 28px;
    }
    .search-wrap { position: relative; }
    .search-wrap span { position: absolute; top: 50%; left: 16px; transform: translateY(-50%); color: var(--muted); }
    .search {
      width: 100%; height: 48px; padding: 0 18px 0 44px; border: 1px solid var(--line); border-radius: 16px;
      color: var(--ink); background: var(--solid); outline: none;
    }
    .search:focus { border-color: var(--blue); box-shadow: 0 0 0 4px var(--blue-wash); }
    .filters { display: flex; gap: 7px; flex-wrap: wrap; }
    .filter {
      min-height: 38px; padding: 0 13px; border: 1px solid var(--line); border-radius: 999px;
      color: var(--muted); background: var(--surface); cursor: pointer;
    }
    .filter[aria-pressed="true"] { color: white; border-color: var(--blue); background: var(--blue); }
    .safety-select {
      min-height: 38px; padding: 0 31px 0 13px; border: 1px solid var(--line); border-radius: 999px;
      color: var(--muted); background: var(--surface); cursor: pointer;
    }
    .catalog-meta { display: flex; justify-content: space-between; gap: 20px; color: var(--muted); font-size: 13px; margin-bottom: 16px; }
    .product-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 16px; }
    .product-card {
      min-height: 286px; padding: 24px; border: 1px solid var(--line); border-radius: 22px; text-align: left;
      display: flex; flex-direction: column; cursor: pointer; color: var(--ink); background: var(--solid);
      box-shadow: 0 1px 0 rgba(255,255,255,.12) inset; transition: .3s var(--ease);
    }
    .product-card:hover { transform: translateY(-7px); border-color: color-mix(in srgb, var(--blue) 44%, var(--line)); box-shadow: var(--shadow); }
    .product-card .card-icon {
      width: 46px; height: 46px; margin-bottom: 24px; border-radius: 14px; display: grid; place-items: center;
      color: var(--blue); background: var(--blue-wash);
    }
    .product-card .card-icon svg { width: 24px; height: 24px; fill: none; stroke: currentColor; stroke-width: 1.8; stroke-linecap: round; stroke-linejoin: round; }
    .product-card[data-kind="skill"] .card-icon { color: var(--ink); background: color-mix(in srgb, var(--ink) 8%, var(--solid)); }
    .product-card[data-kind="tool"] .card-icon { color: var(--blue); background: var(--blue-wash); }
    .product-card[data-kind="harness-tool"] .card-icon { color: white; background: var(--blue); }
    .product-card .kind { color: var(--blue); font-size: 11px; font-weight: 760; letter-spacing: .09em; text-transform: uppercase; }
    .product-card h3 { margin-top: 7px; }
    .product-card p { color: var(--muted); font-size: 14px; }
    .card-bottom { margin-top: auto; display: flex; justify-content: space-between; align-items: center; }
    .safety { padding: 5px 8px; border-radius: 8px; color: var(--muted); background: var(--bg); font-size: 11px; font-weight: 760; }
    .card-arrow { color: var(--blue); font-size: 20px; transition: transform .2s; }
    .product-card:hover .card-arrow { transform: translateX(4px); }
    .empty {
      grid-column: 1 / -1; padding: 70px 20px; border: 1px dashed var(--line); border-radius: 24px;
      color: var(--muted); text-align: center;
    }
    .playground { background: #242424; color: #fff; }
    .playground .section-shell { width: min(1240px, calc(100% - 40px)); }
    .playground .eyebrow { color: #ffae62; }
    .playground .lede { color: #d4d6dc; }
    .scenario-row { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 15px; }
    .scenario {
      padding: 8px 12px; border: 1px solid rgba(255,255,255,.15); border-radius: 999px;
      color: #f3e9df; background: rgba(255,255,255,.06); cursor: pointer;
    }
    .scenario:hover { border-color: #ffae62; background: rgba(255,174,98,.12); }
    .workspace-mode-switch {
      width: fit-content; margin-bottom: 22px; padding: 5px; border: 1px solid rgba(255,255,255,.13);
      border-radius: 16px; display: flex; gap: 5px; background: rgba(255,255,255,.05);
    }
    .workspace-mode-button {
      min-width: 150px; padding: 11px 16px; border: 0; border-radius: 11px;
      color: #aeb5c1; background: transparent; cursor: pointer; font-weight: 760; transition: .2s var(--ease);
    }
    .workspace-mode-button[aria-pressed="true"] { color: #1c1c1c; background: #ffae62; }
    .workspace-mode-button[data-workspace-mode="hardware"][aria-pressed="true"] { color: #fff; background: #fc4c02; }
    .hardware-mode-banner {
      margin-bottom: 18px; padding: 20px; border: 1px solid rgba(252,76,2,.42); border-radius: 22px;
      display: flex; align-items: center; justify-content: space-between; gap: 22px;
      background: linear-gradient(135deg, rgba(252,76,2,.17), rgba(252,76,2,.05));
    }
    .hardware-mode-banner[hidden] { display: none; }
    .hardware-mode-banner h3 { margin: 4px 0 5px; color: #fff; }
    .hardware-mode-banner p { margin: 0; color: #c7cbd3; font-size: 13px; }
    .hardware-mode-banner .eyebrow { margin: 0; font-size: 10px; }
    .play-layout { display: grid; grid-template-columns: .84fr 1.16fr; gap: 16px; }
    .console-panel { border: 1px solid rgba(255,255,255,.12); border-radius: 25px; background: #303030; overflow: hidden; }
    .panel-head { min-height: 54px; padding: 0 18px; border-bottom: 1px solid rgba(255,255,255,.1); display: flex; align-items: center; justify-content: space-between; gap: 12px; }
    .panel-title { font-size: 13px; font-weight: 700; }
    .mode-pill { padding: 4px 8px; border-radius: 999px; color: #ffbe87; background: rgba(252,76,2,.16); font-size: 10px; font-weight: 760; }
    .mode-pill.hardware { color: #fff; background: #fc4c02; }
    .composer { padding: 18px; }
    .composer textarea, .protocol-editor {
      width: 100%; resize: vertical; border: 1px solid rgba(255,255,255,.12); border-radius: 16px;
      color: #fff; background: #1b1b1b; outline: none;
    }
    .composer textarea { min-height: 142px; padding: 16px; font-size: 17px; line-height: 1.5; }
    .composer textarea:focus, .protocol-editor:focus { border-color: #ffae62; box-shadow: 0 0 0 4px rgba(252,76,2,.14); }
    .composer-hint { min-height: 42px; margin: 10px 2px; color: #8d96a8; font-size: 12px; }
    .composer-actions { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
    .composer-buttons { display: flex; align-items: center; justify-content: flex-end; gap: 8px; flex-wrap: wrap; }
    .simulation-badge { display: flex; align-items: center; gap: 6px; color: #8d96a8; font-size: 11px; }
    .simulation-badge::before { content: ""; width: 7px; height: 7px; border-radius: 50%; background: #ffae62; }
    .simulation-badge.hardware::before { background: #fc4c02; box-shadow: 0 0 9px rgba(252,76,2,.75); }
    .pipeline { min-height: 256px; padding: 18px; display: flex; gap: 10px; align-items: stretch; overflow-x: auto; }
    .pipeline-empty { width: 100%; min-height: 218px; display: grid; place-items: center; color: #7f8796; text-align: center; }
    .pipeline-step { min-width: 190px; max-width: 220px; padding: 16px; border: 1px solid rgba(255,255,255,.11); border-radius: 18px; background: #202020; animation: reveal .45s var(--ease) both; }
    .step-index { color: #ffae62; font-size: 11px; font-weight: 760; }
    .pipeline-step strong { display: block; margin: 8px 0; font-size: 16px; }
    .tool-chip { display: inline-block; padding: 4px 6px; margin: 3px 2px 0 0; border-radius: 7px; color: #c9cfdb; background: rgba(255,255,255,.07); font-size: 10px; }
    .pipeline-arrow { align-self: center; color: #596173; flex: 0 0 auto; }
    .result-tabs { display: flex; gap: 4px; }
    .dark-tab { border: 0; padding: 7px 9px; border-radius: 8px; color: #8992a3; background: transparent; cursor: pointer; font-size: 11px; }
    .dark-tab[aria-selected="true"] { color: #fff; background: rgba(255,255,255,.09); }
    .raw { min-height: 220px; max-height: 390px; overflow: auto; padding: 17px; margin: 0; color: #ffd0b9; background: #181818; font: 12px/1.65 ui-monospace, SFMono-Regular, Menlo, monospace; white-space: pre-wrap; word-break: break-word; }
    .timeline { padding: 18px; }
    .event { position: relative; padding: 0 0 19px 25px; color: #afb6c3; font-size: 12px; opacity: .46; transition: .3s; }
    .event::before { content: ""; position: absolute; left: 2px; top: 4px; width: 8px; height: 8px; border: 2px solid #777; border-radius: 50%; background: #303030; }
    .event::after { content: ""; position: absolute; left: 6px; top: 15px; bottom: 0; width: 1px; background: #3b414e; }
    .event:last-child::after { display: none; }
    .event.active { color: #fff; opacity: 1; }
    .event.active::before { border-color: #ffae62; box-shadow: 0 0 12px rgba(252,76,2,.55); }
    .architecture { background: var(--solid); }
    .architecture-flow { display: flex; align-items: stretch; gap: 7px; overflow-x: auto; padding: 12px 2px 24px; }
    .arch-node {
      min-width: 128px; min-height: 132px; padding: 16px 12px; border: 1px solid var(--line); border-radius: 18px;
      display: flex; flex-direction: column; justify-content: space-between; text-align: left; color: var(--ink); background: var(--bg); cursor: pointer; transition: .23s;
    }
    .arch-node:hover, .arch-node[aria-pressed="true"] { border-color: var(--blue); background: var(--blue-wash); transform: translateY(-3px); }
    .arch-node small { color: var(--blue); font-weight: 720; }
    .arch-node strong { font-size: 14px; line-height: 1.25; }
    .arch-arrow { align-self: center; min-width: 22px; color: var(--muted); text-align: center; }
    .arch-detail { display: grid; grid-template-columns: .8fr 1.2fr; gap: 34px; padding: 32px; border: 1px solid var(--line); border-radius: var(--radius); background: var(--bg); }
    .arch-summary h3 { font-size: 34px; }
    .arch-summary p { color: var(--muted); }
    .process-pill { display: inline-flex; padding: 7px 10px; border-radius: 9px; color: var(--blue); background: var(--blue-wash); font-size: 12px; font-weight: 650; }
    .owns-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .owns-grid > div { padding: 18px; border-radius: 17px; background: var(--solid); }
    .owns-grid h4 { margin: 0 0 10px; font-size: 12px; letter-spacing: .05em; }
    .owns-grid ul { margin: 0; padding-left: 18px; color: var(--muted); font-size: 13px; }
    .protocol { background: linear-gradient(180deg, var(--bg), color-mix(in srgb, var(--blue) 5%, var(--bg))); }
    .lab-shell { display: grid; grid-template-columns: .86fr 1.14fr; border: 1px solid var(--line); border-radius: var(--radius); overflow: hidden; background: var(--solid); box-shadow: var(--shadow); }
    .lab-info { padding: 34px; border-right: 1px solid var(--line); }
    .protocol-tabs { display: grid; gap: 8px; margin: 26px 0; }
    .protocol-tab { padding: 14px; border: 1px solid var(--line); border-radius: 14px; text-align: left; background: var(--bg); cursor: pointer; }
    .protocol-tab[aria-selected="true"] { border-color: var(--blue); background: var(--blue-wash); }
    .field-help { padding: 16px; border-radius: 14px; color: var(--muted); background: var(--bg); font-size: 12px; }
    .lab-console { padding: 22px; color: #fff; background: #242424; }
    .protocol-editor { min-height: 210px; padding: 15px; font: 12px/1.6 ui-monospace, SFMono-Regular, Menlo, monospace; }
    .lab-actions { display: flex; justify-content: space-between; gap: 12px; margin: 12px 0; }
    .protocol-output { min-height: 164px; padding: 15px; border-radius: 16px; color: #ffd0b9; background: #181818; font: 12px/1.6 ui-monospace, SFMono-Regular, Menlo, monospace; white-space: pre-wrap; word-break: break-word; }
    .safety-layers { display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; margin-bottom: 42px; }
    .safety-layer { position: relative; padding: 22px; border: 1px solid var(--line); border-radius: 20px; background: var(--solid); }
    .safety-layer::before { content: attr(data-level); display: block; margin-bottom: 25px; color: var(--blue); font-weight: 800; font-size: 12px; }
    .safety-layer p { color: var(--muted); font-size: 13px; }
    .game { padding: 28px; border: 1px solid var(--line); border-radius: var(--radius); background: var(--solid); }
    .game-top { display: flex; justify-content: space-between; gap: 20px; align-items: center; margin-bottom: 20px; }
    .score { color: var(--muted); font-size: 13px; }
    .scenario-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 10px; }
    .scenario-card { min-height: 220px; padding: 17px; border: 1px solid var(--line); border-radius: 18px; display: flex; flex-direction: column; background: var(--bg); }
    .scenario-number { color: var(--blue); font-size: 11px; font-weight: 800; }
    .scenario-card p { margin: 12px 0; font-size: 13px; }
    .decision-row { display: grid; grid-template-columns: 1fr 1fr; gap: 7px; margin-top: auto; }
    .decision { min-height: 34px; border: 1px solid var(--line); border-radius: 10px; background: var(--solid); cursor: pointer; font-size: 11px; }
    .decision:hover { border-color: var(--blue); }
    .scenario-card.answered.correct { border-color: var(--green); }
    .scenario-card.answered.wrong { border-color: var(--red); }
    .scenario-explain { margin-top: 9px; color: var(--muted); font-size: 10px; }
    .hardware-bay { color: #fffaf4; background: #2d2926; }
    .hardware-bay .eyebrow { color: #ffae62; }
    .hardware-bay .lede { color: #d2cbc5; }
    .hardware-status {
      margin-bottom: 18px; padding: 14px 17px; border: 1px solid rgba(255,255,255,.11);
      border-radius: 16px; display: flex; align-items: center; justify-content: space-between; gap: 16px;
      color: #d2cbc5; background: #37312e; font-size: 13px;
    }
    .hardware-status strong { color: #ffae62; text-align: right; }
    .approval-layout { display: grid; grid-template-columns: .88fr 1.12fr; gap: 16px; }
    .approval-card {
      padding: 24px; border: 1px solid rgba(255,255,255,.11); border-radius: 24px;
      background: #37312e;
    }
    .approval-card h3 { color: #fff; }
    .approval-card p, .approval-card label { color: #d2cbc5; font-size: 13px; }
    .approval-card textarea, .approval-card input[type="text"] {
      width: 100%; padding: 14px; border: 1px solid rgba(255,255,255,.14); border-radius: 14px;
      color: #fff; background: #211e1c; outline: none;
    }
    .approval-card textarea { min-height: 112px; resize: vertical; }
    .approval-card textarea:focus, .approval-card input[type="text"]:focus {
      border-color: #ffae62; box-shadow: 0 0 0 4px rgba(252,76,2,.16);
    }
    .approval-actions { display: flex; gap: 9px; align-items: center; flex-wrap: wrap; margin-top: 13px; }
    .amber-button { background: #fc4c02; }
    .amber-button:hover { box-shadow: 0 10px 30px rgba(252,76,2,.3); }
    .hardware-lock {
      margin-top: 15px; padding: 13px; border-radius: 13px; color: #ffc69e;
      background: rgba(252,76,2,.1); font-size: 12px;
    }
    .approval-steps { display: grid; gap: 8px; margin: 15px 0; }
    .approval-step { padding: 12px; border-radius: 13px; color: #eee7e1; background: #26211f; font-size: 12px; }
    .approval-step strong { display: block; color: #ffae62; }
    .checklist { display: grid; gap: 8px; margin: 16px 0; }
    .check-item {
      padding: 11px 12px; border: 1px solid rgba(255,255,255,.1); border-radius: 12px;
      display: flex; gap: 10px; align-items: flex-start; background: #26211f; cursor: pointer;
    }
    .check-item input { margin-top: 3px; accent-color: #fc4c02; }
    .confirmation-label { display: grid; gap: 7px; }
    .confirmation-label code { color: #ffae62; }
    .danger-note { margin-top: 13px; color: #ff8798 !important; font-size: 12px !important; }
    .task-console { min-height: 150px; max-height: 330px; overflow: auto; padding: 14px; border-radius: 14px; color: #ffd0b9; background: #211e1c; font: 12px/1.6 ui-monospace, SFMono-Regular, Menlo, monospace; white-space: pre-wrap; word-break: break-word; }
    .task-event { padding: 8px 0; border-bottom: 1px solid rgba(255,255,255,.08); }
    .task-event:last-child { border-bottom: 0; }
    .task-event strong { color: #ffae62; }
    .cancel-button { color: #ff8798; border-color: rgba(255,97,119,.38) !important; background: rgba(255,97,119,.08) !important; }
    .extension { background: var(--solid); }
    .extension-steps { display: grid; grid-template-columns: repeat(3, 1fr); gap: 16px; }
    .extension-step { padding: 28px; border: 1px solid var(--line); border-radius: 24px; background: var(--bg); }
    .extension-step .number { color: var(--blue); font-size: 44px; font-weight: 700; letter-spacing: -.06em; }
    .extension-step p { color: var(--muted); }
    .source-button { max-width: 100%; padding: 8px 10px; border: 1px solid var(--line); border-radius: 9px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; color: var(--muted); background: var(--solid); cursor: pointer; font: 11px ui-monospace, SFMono-Regular, Menlo, monospace; }
    footer { padding: 50px 20px 70px; color: var(--muted); text-align: center; border-top: 1px solid var(--line); background: var(--solid); }
    footer strong { display: block; margin-bottom: 7px; color: var(--ink); }
    dialog { width: min(720px, calc(100% - 28px)); max-height: min(820px, calc(100svh - 28px)); padding: 0; border: 1px solid var(--line); border-radius: 28px; color: var(--ink); background: var(--solid); box-shadow: 0 30px 120px rgba(0,0,0,.34); overflow: auto; }
    dialog::backdrop { background: rgba(0,0,0,.48); backdrop-filter: blur(8px); }
    .dialog-head { position: sticky; z-index: 2; top: 0; padding: 18px 20px; border-bottom: 1px solid var(--line); display: flex; align-items: center; justify-content: space-between; background: color-mix(in srgb, var(--solid) 88%, transparent); backdrop-filter: blur(16px); }
    .dialog-head strong { font-size: 14px; }
    .dialog-body { padding: 30px; }
    .close { width: 34px; height: 34px; border: 1px solid var(--line); border-radius: 50%; background: var(--bg); cursor: pointer; }
    .detail-hero { padding: 25px; border-radius: 22px; background: var(--blue-wash); }
    .detail-hero h2 { margin: 7px 0 12px; font-size: 42px; }
    .detail-hero p { color: var(--muted); }
    .detail-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin: 18px 0; }
    .detail-box { padding: 17px; border: 1px solid var(--line); border-radius: 16px; }
    .detail-box h4 { margin: 0 0 8px; font-size: 12px; }
    .detail-box p { margin: 0; color: var(--muted); font-size: 12px; }
    .parameter-list { display: grid; gap: 8px; margin: 15px 0; }
    .parameter { padding: 12px; border-radius: 13px; background: var(--bg); }
    .parameter code { color: var(--blue); }
    .parameter span { color: var(--muted); font-size: 12px; }
    .tour-progress { display: grid; grid-template-columns: repeat(4, 1fr); gap: 5px; margin-bottom: 24px; }
    .tour-progress span { height: 4px; border-radius: 999px; background: var(--line); }
    .tour-progress span.active { background: var(--blue); }
    .tour-card { min-height: 280px; padding: 28px; border-radius: 24px; background: var(--bg); }
    .tour-visual { height: 105px; margin-bottom: 25px; display: flex; align-items: center; justify-content: center; gap: 9px; }
    .tour-visual span { padding: 12px; border-radius: 12px; color: var(--blue); background: var(--blue-wash); font-weight: 750; }
    .tour-actions { display: flex; justify-content: space-between; margin-top: 20px; }
    .toast {
      position: fixed; z-index: 100; left: 50%; bottom: 24px; transform: translate(-50%, 130%);
      max-width: calc(100% - 30px); padding: 11px 16px; border: 1px solid var(--line); border-radius: 999px;
      color: var(--ink); background: var(--solid); box-shadow: var(--shadow); font-size: 13px; transition: .3s var(--ease);
    }
    .toast.show { transform: translate(-50%, 0); }
    .hardware-power-toggle {
      position: fixed; z-index: 90; right: max(20px, env(safe-area-inset-right)); bottom: max(20px, env(safe-area-inset-bottom));
      min-width: 164px; min-height: 50px; padding: 0 18px; border: 1px solid var(--line); border-radius: 999px;
      display: inline-flex; align-items: center; justify-content: center; gap: 10px;
      color: #fff; background: #5f5b58; box-shadow: 0 16px 42px rgba(0,0,0,.24);
      cursor: pointer; font-weight: 760; transition: .24s var(--ease); backdrop-filter: blur(18px);
    }
    .hardware-power-toggle:not([disabled]):hover { transform: translateY(-2px); box-shadow: 0 20px 48px rgba(0,0,0,.3); }
    .hardware-power-toggle[data-state="offline"] { background: var(--blue); }
    .hardware-power-toggle[data-state="online"] { background: var(--red); }
    .hardware-power-toggle[data-state="busy"] { background: var(--amber); }
    .hardware-power-toggle[disabled] { cursor: not-allowed; opacity: .78; }
    .hardware-power-indicator { width: 9px; height: 9px; border: 2px solid currentColor; border-radius: 50%; }
    .hardware-power-toggle[data-state="online"] .hardware-power-indicator {
      background: currentColor; box-shadow: 0 0 0 4px rgba(255,255,255,.18);
    }
    .demo-mode-toggle {
      position: fixed; z-index: 90; right: max(20px, env(safe-area-inset-right)); bottom: max(82px, calc(env(safe-area-inset-bottom) + 62px));
      min-width: 164px; min-height: 44px; padding: 0 18px; border: 1px solid rgba(255,255,255,.28); border-radius: 999px;
      color: #fff; background: #1c1c1e; box-shadow: 0 14px 38px rgba(0,0,0,.28);
      cursor: pointer; font-weight: 760; transition: .24s var(--ease);
    }
    .demo-mode-toggle:hover { color: #000; background: #f5f5f7; transform: translateY(-2px); box-shadow: 0 18px 44px rgba(0,0,0,.38); }
    /* Memory Store: a quiet, product-first surface with one clear action. */
    .memory-mode-toggle {
      position: fixed; z-index: 90; right: max(20px, env(safe-area-inset-right)); bottom: max(136px, calc(env(safe-area-inset-bottom) + 116px));
      min-width: 164px; min-height: 44px; padding: 0 18px; cursor: pointer; transition: .24s var(--ease);
      border-color: rgba(255,255,255,.28); border-radius: 999px; color: #fff; background: #1c1c1e;
      box-shadow: 0 14px 38px rgba(0,0,0,.28); font-family: Inter, ui-sans-serif, sans-serif;
      font-size: 16px; font-weight: 760; letter-spacing: normal; text-transform: none; clip-path: none;
    }
    .memory-mode-toggle::before { display: none; }
    .memory-mode-toggle:hover { color: #000; background: #f5f5f7; box-shadow: 0 18px 44px rgba(0,0,0,.38); }
    .memory-overlay {
      --memory-blue: #0071e3;
      position: fixed; z-index: 115; inset: 0; overflow: auto; scrollbar-width: none; -ms-overflow-style: none;
      color: #1d1d1f; background: #f5f5f7; background-size: auto;
      font-family: Inter, ui-sans-serif, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      scroll-behavior: smooth; animation: reveal .24s var(--ease);
    }
    .memory-overlay::before, .memory-overlay::after { display: none; }
    .memory-overlay[hidden] { display: none; }
    .memory-overlay::-webkit-scrollbar { display: none; width: 0; height: 0; }
    .memory-shell {
      position: relative; z-index: 1; display: block; width: 100%; min-height: 100svh; padding: 0 0 96px;
    }
    .memory-header {
      position: sticky; z-index: 8; top: 0; width: 100%; min-height: 64px;
      padding: 0 max(24px, calc((100vw - 1180px) / 2)); border-bottom: 1px solid rgba(0,0,0,.08);
      display: flex; align-items: center; gap: 18px; background: rgba(250,250,252,.82);
      backdrop-filter: saturate(180%) blur(24px);
    }
    .memory-identity { display: flex; align-items: center; gap: 11px; }
    .memory-brand-logo {
      width: 32px; height: 32px; border-radius: 50%; display: block; object-fit: cover;
      background: #fff; box-shadow: 0 0 0 1px rgba(0,0,0,.1);
    }
    .memory-identity strong { display: block; color: #1d1d1f; font-size: 17px; font-weight: 650; letter-spacing: -.02em; }
    .memory-identity small { display: block; margin-top: 1px; color: #6e6e73; font-size: 10px; font-weight: 560; letter-spacing: .02em; }
    .memory-status-line {
      margin-left: auto; padding: 6px 11px; border-radius: 999px; color: #6e6e73; background: rgba(0,0,0,.045);
      font: 500 11px/1.2 Inter, ui-sans-serif, sans-serif; letter-spacing: 0;
    }
    .memory-status-line b { color: #1d1d1f; font-weight: 650; }
    .memory-close {
      width: 34px; height: 34px; border: 0; border-radius: 50%; color: #1d1d1f; background: #e8e8ed;
      display: grid; place-items: center; cursor: pointer; font-size: 20px; line-height: 1; clip-path: none;
      transition: transform .2s var(--ease), background .2s var(--ease);
    }
    .memory-close:hover { color: #1d1d1f; background: #d2d2d7; transform: scale(1.04); }
    .memory-layout {
      width: min(1180px, calc(100% - 48px)); min-height: 0; margin: 32px auto 0;
      display: flex; flex-direction: column; gap: 52px;
    }
    .memory-store-hero {
      position: relative; min-height: 330px; padding: clamp(42px, 7vw, 82px) 28px; border-radius: 32px; overflow: hidden;
      display: grid; grid-template-columns: minmax(0, 1.08fr) minmax(280px, .92fr); align-items: center; gap: 24px;
      color: #f5f5f7; background: #000; text-align: left;
      isolation: isolate;
    }
    .memory-store-hero::before {
      content: ""; position: absolute; z-index: -1; width: 560px; height: 560px; border-radius: 50%;
      background: radial-gradient(circle at 37% 32%, #fff 0, #a7a7ac 8%, #343438 28%, #09090a 58%, transparent 70%);
      opacity: .7; transform: translate(46%, 34%);
    }
    .memory-hero-copy { position: relative; z-index: 2; width: min(650px, 100%); padding-left: clamp(8px, 3vw, 36px); }
    .memory-hero-eyebrow {
      margin: 0 0 14px; color: #a1a1a6; font-size: 12px; font-weight: 650; letter-spacing: .06em;
    }
    .memory-store-hero h1 {
      margin: 0; color: #f5f5f7; font-size: clamp(42px, 6vw, 68px); font-weight: 650;
      line-height: .98; letter-spacing: -.055em;
    }
    .memory-store-hero p:last-child {
      width: min(560px, 100%); margin: 21px 0 0; color: #a1a1a6; font-size: clamp(16px, 2vw, 20px);
      line-height: 1.45; letter-spacing: -.015em;
    }
    .memory-hero-visual {
      position: relative; z-index: 1; min-height: 270px; display: grid; place-items: center;
    }
    .memory-hero-robot {
      width: min(360px, 100%); max-height: 360px; display: block; object-fit: contain;
      filter: drop-shadow(0 28px 38px rgba(0,0,0,.72)); transform: translateY(28px) scale(1.08);
      user-select: none; pointer-events: none;
    }
    .memory-zone-rail {
      min-width: 0; overflow: hidden; border: 0; border-radius: 0; background: transparent; box-shadow: none;
    }
    .memory-zone-rail > .memory-panel-heading, .memory-legend { display: none; }
    .memory-zone-heading { margin: 0 0 18px; font-size: 28px; font-weight: 650; letter-spacing: -.035em; }
    .memory-zone-list {
      display: grid; grid-template-columns: repeat(7, minmax(112px, 1fr)); gap: 10px; padding: 0 0 4px;
      overflow-x: auto; scrollbar-width: none;
    }
    .memory-zone-list::-webkit-scrollbar { display: none; }
    .memory-zone-button {
      width: auto; min-width: 112px; min-height: 126px; padding: 16px 10px 14px; border: 0; border-radius: 22px;
      display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 9px;
      color: #6e6e73; background: #fff; cursor: pointer; text-align: center; box-shadow: 0 8px 28px rgba(0,0,0,.045);
      transition: transform .24s var(--ease), box-shadow .24s var(--ease), color .24s var(--ease);
    }
    .memory-zone-button:hover { color: #1d1d1f; border-color: transparent; background: #fff; transform: translateY(-3px); box-shadow: 0 14px 34px rgba(0,0,0,.09); }
    .memory-zone-button.active { color: #1d1d1f; border-color: transparent; background: #fff; box-shadow: 0 0 0 2px #1d1d1f inset, 0 14px 34px rgba(0,0,0,.08); }
    .memory-zone-icon {
      width: 50px; height: 50px; border-radius: 16px; display: grid; place-items: center;
      color: #1d1d1f; background: #f5f5f7; text-align: center;
      box-shadow: 0 0 0 1px rgba(0,0,0,.035) inset;
    }
    .memory-zone-icon svg {
      width: 27px; height: 27px; fill: none; stroke: currentColor; stroke-linecap: round;
      stroke-linejoin: round; stroke-width: 1.65;
    }
    .memory-zone-icon[data-system="visual"] { color: #0869d6; background: #e8f2ff; }
    .memory-zone-icon[data-system="hand"] { color: #d94f1c; background: #fff0e9; }
    .memory-zone-icon[data-system="os"] { color: #6c3bc7; background: #f1ebff; }
    .memory-zone-icon[data-system="haptic"] { color: #c72f71; background: #ffedf5; }
    .memory-zone-icon[data-system="chassis"] { color: #087d61; background: #e5f7f1; }
    .memory-zone-icon[data-system="arm"] { color: #38647c; background: #eaf2f6; }
    .memory-zone-icon[data-system="waist"] { color: #a86400; background: #fff4d6; }
    .memory-zone-button.active .memory-zone-icon { color: #fff; background: #1d1d1f; }
    .memory-zone-button strong { display: block; font-size: 12px; font-weight: 620; line-height: 1.2; letter-spacing: 0; }
    .memory-zone-button small { display: none; }
    .memory-zone-count { color: #86868b; font-size: 10px; line-height: 1; }
    .memory-zone-button.active .memory-zone-count { color: #6e6e73; }
    .memory-loadout-panel {
      min-height: 0; overflow: visible; border: 0; border-radius: 0; display: block;
      background: transparent; box-shadow: none;
    }
    .memory-loadout-panel > .memory-panel-heading { display: none; }
    .memory-zone-summary { padding: 0 0 34px; border: 0; }
    .memory-zone-summary-top { display: flex; align-items: flex-end; justify-content: space-between; gap: 18px; }
    .memory-zone-summary h2 {
      margin: 0; color: #1d1d1f; font-size: clamp(38px, 5vw, 58px); font-weight: 650;
      line-height: 1; letter-spacing: -.045em;
    }
    .memory-zone-summary p {
      max-width: 720px; margin: 14px 0 0; color: #6e6e73; font: 16px/1.5 Inter, ui-sans-serif, sans-serif;
      letter-spacing: -.012em;
    }
    .memory-zone-code {
      padding: 7px 11px; border-radius: 999px; color: #6e6e73; background: #e8e8ed;
      font: 560 11px/1.2 Inter, ui-sans-serif, sans-serif; letter-spacing: 0; white-space: nowrap;
    }
    .memory-available { min-height: 0; padding: 0; overflow: visible; }
    .memory-equipped { padding: 0 0 48px; border: 0; }
    .memory-section-title {
      margin-bottom: 16px; display: flex; align-items: center; justify-content: space-between;
      color: #6e6e73; font: 560 12px/1.2 Inter, ui-sans-serif, sans-serif; letter-spacing: 0;
    }
    .memory-section-title strong { color: #1d1d1f; font-size: 24px; font-weight: 650; letter-spacing: -.03em; }
    .memory-slot-grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; }
    .memory-slot {
      position: relative; min-height: 116px; padding: 18px 42px 18px 18px; border: 0; border-radius: 20px;
      color: #86868b; background: rgba(255,255,255,.62); text-align: left; box-shadow: 0 8px 28px rgba(0,0,0,.035);
    }
    .memory-slot.filled, .memory-slot[data-kind="skill"], .memory-slot[data-kind="tool"], .memory-slot[data-kind="harness-tool"] {
      border: 0; color: #6e6e73; background: #fff;
    }
    .memory-slot-index {
      display: block; margin-bottom: 12px; color: #86868b; font: 560 10px/1.2 Inter, ui-sans-serif, sans-serif;
      letter-spacing: .02em;
    }
    .memory-slot strong { display: block; overflow: hidden; color: #1d1d1f; font-size: 15px; font-weight: 620; text-overflow: ellipsis; white-space: nowrap; }
    .memory-slot small { display: block; margin-top: 5px; overflow: hidden; color: #86868b; font-size: 11px; text-overflow: ellipsis; white-space: nowrap; }
    .memory-unequip {
      position: absolute; top: 14px; right: 14px; width: 26px; height: 26px; padding: 0; border: 0; border-radius: 50%;
      display: grid; place-items: center; color: #6e6e73; background: #e8e8ed; cursor: pointer; font-size: 15px;
      transition: color .2s var(--ease), background .2s var(--ease), transform .2s var(--ease);
    }
    .memory-unequip:hover { color: #fff; background: #1d1d1f; transform: scale(1.05); }
    .memory-product-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 16px; }
    .memory-product-card {
      position: relative; min-height: 320px; padding: 24px; border-radius: 26px; display: flex; flex-direction: column;
      background: #fff; box-shadow: 0 10px 34px rgba(0,0,0,.055);
      transition: transform .28s var(--ease), box-shadow .28s var(--ease);
    }
    .memory-product-card:hover { transform: translateY(-4px); box-shadow: 0 18px 46px rgba(0,0,0,.09); }
    .memory-product-top { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
    .memory-product-icon {
      width: 56px; height: 56px; border-radius: 17px; display: grid; place-items: center;
      color: #fff; background: linear-gradient(145deg, #4f4f54, #1d1d1f);
      box-shadow: 0 9px 22px rgba(0,0,0,.14), 0 0 0 1px rgba(255,255,255,.18) inset;
    }
    .memory-product-icon[data-tone="0"] { background: linear-gradient(145deg, #55a7ff, #0568d5); }
    .memory-product-icon[data-tone="1"] { background: linear-gradient(145deg, #b27cff, #7040d8); }
    .memory-product-icon[data-tone="2"] { background: linear-gradient(145deg, #41d5b0, #07856d); }
    .memory-product-icon[data-tone="3"] { background: linear-gradient(145deg, #ff8c66, #e64d25); }
    .memory-product-icon[data-tone="4"] { background: linear-gradient(145deg, #f5c84c, #d48a00); }
    .memory-product-icon[data-tone="5"] { background: linear-gradient(145deg, #ff72aa, #cf326f); }
    .memory-product-icon svg {
      width: 28px; height: 28px; fill: none; stroke: currentColor; stroke-linecap: round;
      stroke-linejoin: round; stroke-width: 1.7;
    }
    .memory-product-kind { color: #86868b; font-size: 10px; font-weight: 620; letter-spacing: .04em; text-transform: uppercase; }
    .memory-product-card h3 {
      margin: 28px 0 8px; color: #1d1d1f; font-size: 24px; font-weight: 650; line-height: 1.08; letter-spacing: -.035em;
    }
    .memory-product-card p { margin: 0; color: #6e6e73; font-size: 14px; line-height: 1.45; }
    .memory-product-footer {
      margin-top: auto; padding-top: 28px; display: flex; align-items: center; justify-content: space-between; gap: 14px;
    }
    .memory-price { display: flex; align-items: baseline; gap: 7px; flex-wrap: wrap; }
    .memory-price-current { color: #1d1d1f; font-size: 18px; font-weight: 680; letter-spacing: -.025em; }
    .memory-price-original { color: #86868b; font-size: 11px; text-decoration: line-through; }
    .memory-price-discount { color: #b42318; font-size: 10px; font-weight: 650; }
    .memory-add {
      min-width: 58px; min-height: 30px; padding: 0 13px; border: 0; border-radius: 999px;
      color: #fff; background: var(--memory-blue); cursor: pointer; font-size: 12px; font-weight: 650;
      transition: filter .2s var(--ease), transform .2s var(--ease);
    }
    .memory-add:hover { filter: brightness(.92); transform: scale(1.03); }
    .memory-add[disabled] { color: #6e6e73; background: #e8e8ed; cursor: default; filter: none; transform: none; }
    .memory-product-detail {
      position: absolute; z-index: 4; inset: 88px 14px 62px; padding: 17px; border-radius: 19px; overflow: auto;
      color: #f5f5f7; background: rgba(29,29,31,.96); box-shadow: 0 18px 44px rgba(0,0,0,.28);
      backdrop-filter: saturate(150%) blur(18px); opacity: 0; visibility: hidden; pointer-events: none;
      transform: translateY(8px) scale(.98); transition: opacity .2s var(--ease), transform .24s var(--ease), visibility .2s;
      scrollbar-width: thin;
    }
    .memory-product-card:hover .memory-product-detail,
    .memory-product-card:focus-within .memory-product-detail,
    .memory-product-card:focus-visible .memory-product-detail {
      opacity: 1; visibility: visible; transform: none;
    }
    .memory-product-detail > strong { display: block; margin-bottom: 7px; font-size: 12px; font-weight: 650; }
    .memory-product-detail p { margin: 0 0 12px; color: #d2d2d7; font-size: 11px; line-height: 1.45; }
    .memory-detail-list { margin: 0; display: grid; gap: 7px; }
    .memory-detail-list div { display: grid; grid-template-columns: 72px minmax(0, 1fr); gap: 8px; }
    .memory-detail-list dt { color: #8e8e93; font-size: 9px; font-weight: 620; text-transform: uppercase; }
    .memory-detail-list dd { margin: 0; color: #f5f5f7; font-size: 10px; line-height: 1.35; overflow-wrap: anywhere; }
    .memory-empty { padding: 28px; border: 0; border-radius: 20px; color: #6e6e73; background: #fff; font-size: 13px; text-align: center; }
    .demo-wizard {
      position: fixed; z-index: 110; inset: 0; padding: 0; overflow: auto;
      display: grid; place-items: stretch; color: #f5f5f7;
      background: radial-gradient(circle at 50% 16%, #242426 0, #0c0c0d 34%, #000 74%);
      scrollbar-width: none; -ms-overflow-style: none; animation: reveal .35s var(--ease);
    }
    .demo-wizard::-webkit-scrollbar, .hardware-boot-overlay::-webkit-scrollbar { display: none; width: 0; height: 0; }
    html:has(.demo-wizard:not([hidden])), html:has(.hardware-boot-overlay:not([hidden])), html:has(.memory-overlay:not([hidden])),
    body:has(.demo-wizard:not([hidden])), body:has(.hardware-boot-overlay:not([hidden])), body:has(.memory-overlay:not([hidden])) { overflow: hidden; }
    .demo-wizard[hidden] { display: none; }
    .demo-shell {
      box-sizing: border-box; width: 100%; min-height: 100svh;
      padding: clamp(22px, 4vw, 52px) clamp(20px, 6vw, 88px);
      display: flex; flex-direction: column;
    }
    .demo-topbar { display: flex; align-items: center; justify-content: space-between; gap: 18px; }
    .demo-brand { display: flex; align-items: center; gap: 11px; color: #d2d2d7; font-size: 12px; font-weight: 760; letter-spacing: .08em; }
    .demo-brand img { width: 34px; height: 34px; border-radius: 50%; object-fit: cover; background: #fff; }
    .demo-close {
      width: 36px; height: 36px; border: 1px solid #3a3a3c; border-radius: 50%;
      color: #f5f5f7; background: rgba(28,28,30,.78); cursor: pointer; font-size: 20px;
    }
    .demo-stage-dots { width: min(980px, 100%); margin: 38px auto 18px; display: grid; grid-template-columns: repeat(5, 1fr); gap: 7px; }
    .demo-stage-dot { height: 4px; border-radius: 999px; background: #2c2c2e; transition: background .3s; }
    .demo-stage-dot.active { background: #f5f5f7; }
    .demo-content { width: min(980px, 100%); margin-inline: auto; flex: 1; padding: 34px 0 24px; display: flex; flex-direction: column; justify-content: center; }
    .demo-eyebrow { margin-bottom: 12px; color: #8e8e93; font-size: 11px; font-weight: 760; letter-spacing: .12em; }
    .demo-title { max-width: 700px; margin-bottom: 14px; color: #fff; font-size: clamp(38px, 6vw, 58px); line-height: 1.02; letter-spacing: -.055em; }
    .demo-copy { max-width: 650px; margin-bottom: 30px; color: #a1a1a6; font-size: 16px; }
    .demo-command {
      width: 100%; min-height: 58px; padding: 0 20px; border: 1px solid #48484a; border-radius: 17px;
      color: #fff; background: rgba(28,28,30,.88); outline: none; font-size: 18px;
    }
    .demo-command:focus { border-color: #f5f5f7; box-shadow: 0 0 0 4px rgba(255,255,255,.12); }
    .demo-actions { width: min(980px, 100%); margin-inline: auto; min-height: 58px; display: flex; align-items: center; justify-content: flex-end; gap: 10px; flex-wrap: wrap; }
    .demo-action {
      min-height: 44px; padding: 0 20px; border: 0; border-radius: 999px;
      color: #000; background: #f5f5f7; cursor: pointer; font-weight: 720;
    }
    .demo-action.secondary { color: #f5f5f7; background: #2c2c2e; }
    .demo-action.danger { color: #000; background: #f5f5f7; box-shadow: 0 0 0 1px #fff inset; }
    .demo-action[disabled] { opacity: .42; cursor: not-allowed; }
    .demo-progress-meta { margin: 8px 0 12px; display: flex; justify-content: space-between; gap: 18px; color: #d2d2d7; font-size: 13px; }
    .demo-progress-meta strong { font: 650 14px ui-monospace, SFMono-Regular, Menlo, monospace; }
    .demo-progress-track { height: 7px; border-radius: 999px; overflow: hidden; background: #2c2c2e; }
    .demo-progress-bar { width: 0; height: 100%; border-radius: inherit; background: #f5f5f7; transition: width .65s var(--ease), background .2s; }
    .demo-progress-bar.error { background: #f5f5f7; }
    .demo-progress-bar.success { background: #f5f5f7; }
    .demo-step-list { display: grid; gap: 9px; margin-top: 22px; }
    .demo-step {
      padding: 14px 16px; border: 1px solid #38383a; border-radius: 15px;
      display: grid; grid-template-columns: 38px minmax(0, 1fr) auto; align-items: center; gap: 12px;
      color: #8e8e93; background: rgba(28,28,30,.72); transition: .3s var(--ease);
    }
    .demo-step-index { width: 30px; height: 30px; border-radius: 50%; display: grid; place-items: center; background: #2c2c2e; font-size: 11px; font-weight: 760; }
    .demo-step strong { display: block; color: #d2d2d7; }
    .demo-step small { display: block; margin-top: 3px; color: #636366; }
    .demo-step.active { border-color: #f5f5f7; color: #fff; transform: translateX(4px); }
    .demo-step.active .demo-step-index { color: #000; background: #f5f5f7; }
    .demo-step.completed { border-color: rgba(255,255,255,.42); }
    .demo-step.completed .demo-step-index { color: #000; background: #f5f5f7; }
    .demo-step.failed { border-color: #f5f5f7; border-style: dashed; }
    .demo-step.failed .demo-step-index { color: #000; background: #f5f5f7; }
    .demo-choice-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
    .demo-choice {
      min-height: 170px; padding: 24px; border: 1px solid #3a3a3c; border-radius: 22px;
      color: #f5f5f7; background: rgba(28,28,30,.78); cursor: pointer; text-align: left; transition: .25s var(--ease);
    }
    .demo-choice:hover { border-color: #f5f5f7; transform: translateY(-3px); }
    .demo-choice strong { display: block; margin-bottom: 9px; font-size: 20px; }
    .demo-choice span { color: #8e8e93; font-size: 13px; }
    .demo-safety-list { display: grid; gap: 9px; }
    .demo-safety-item {
      padding: 13px 15px; border: 1px solid #3a3a3c; border-radius: 14px;
      display: flex; gap: 12px; color: #d2d2d7; background: rgba(28,28,30,.72); cursor: pointer; font-size: 13px;
    }
    .demo-safety-item input { accent-color: #f5f5f7; }
    .demo-event-log { max-height: 128px; margin-top: 18px; overflow: auto; color: #8e8e93; font: 11px/1.6 ui-monospace, SFMono-Regular, Menlo, monospace; }
    .demo-result {
      padding: 18px; border: 1px solid #3a3a3c; border-radius: 16px;
      color: #d2d2d7; background: rgba(28,28,30,.76); white-space: pre-wrap; word-break: break-word;
    }
    .hardware-boot-overlay {
      position: fixed; z-index: 120; inset: 0; padding: 0; overflow: auto;
      display: grid; place-items: stretch; color: #f5f5f7; background: rgba(0,0,0,.97);
      scrollbar-width: none; -ms-overflow-style: none; backdrop-filter: blur(24px); animation: reveal .35s var(--ease);
    }
    .hardware-boot-overlay[hidden] { display: none; }
    .hardware-boot-panel {
      box-sizing: border-box; width: 100%; min-height: 100svh;
      padding: clamp(28px, 7vw, 96px);
      display: flex; flex-direction: column; align-items: center; justify-content: center;
      text-align: center;
    }
    .hardware-boot-logo {
      width: 88px; height: 88px; margin-bottom: 34px; border-radius: 50%; object-fit: cover;
      background: #fff; box-shadow: 0 0 0 1px rgba(255,255,255,.16), 0 18px 55px rgba(255,255,255,.08);
    }
    .hardware-boot-title { margin-bottom: 8px; color: #fff; font-size: 25px; letter-spacing: -.035em; }
    .hardware-boot-subtitle { width: min(640px, 100%); min-height: 24px; margin-bottom: 42px; color: #98989d; font-size: 14px; }
    .hardware-boot-meta {
      width: min(640px, 100%); margin-bottom: 11px; display: flex; align-items: flex-end; justify-content: space-between; gap: 20px;
      color: #d2d2d7; text-align: left; font-size: 13px;
    }
    .hardware-boot-meta strong { flex: 0 0 auto; color: #f5f5f7; font: 650 14px ui-monospace, SFMono-Regular, Menlo, monospace; }
    .hardware-boot-track { width: min(640px, 100%); height: 7px; border-radius: 999px; overflow: hidden; background: #2c2c2e; }
    .hardware-boot-progress {
      width: 0; height: 100%; border-radius: inherit; background: #f5f5f7;
      box-shadow: 0 0 18px rgba(255,255,255,.28); transition: width .7s var(--ease), background .25s;
    }
    .hardware-boot-overlay[data-state="error"] .hardware-boot-progress { background: #f5f5f7; box-shadow: 0 0 18px rgba(255,255,255,.24); }
    .hardware-boot-overlay[data-state="success"] .hardware-boot-progress { background: #f5f5f7; box-shadow: 0 0 18px rgba(255,255,255,.24); }
    .hardware-boot-error {
      box-sizing: border-box; width: min(640px, 100%); margin-top: 28px; padding: 15px 17px; border: 1px solid rgba(255,255,255,.34); border-radius: 14px;
      color: #f5f5f7; background: rgba(255,255,255,.07); text-align: left; font-size: 13px; white-space: pre-wrap; word-break: break-word;
    }
    .hardware-boot-error[hidden] { display: none; }
    .hardware-boot-note { margin: 26px 0 0; color: #636366; font-size: 11px; }
    .hardware-boot-close {
      min-height: 38px; margin-top: 20px; padding: 0 18px; border: 1px solid #48484a; border-radius: 999px;
      color: #f5f5f7; background: #1c1c1e; cursor: pointer;
    }
    .hardware-boot-close[hidden] { display: none; }
    .live-panel { display: none; position: fixed; z-index: 45; top: 66px; right: max(16px, calc((100vw - 1180px) / 2)); width: min(360px, calc(100% - 32px)); padding: 18px; border: 1px solid var(--line); border-radius: 18px; background: var(--solid); box-shadow: var(--shadow); }
    .live-panel.open { display: block; animation: reveal .2s var(--ease); }
    .live-panel h3 { font-size: 16px; }
    .live-panel p { color: var(--muted); font-size: 12px; }
    .live-details { padding: 11px; border-radius: 12px; background: var(--bg); font: 11px/1.5 ui-monospace, SFMono-Regular, Menlo, monospace; white-space: pre-wrap; word-break: break-word; }
    .hardware-stages { display: grid; gap: 7px; margin: 12px 0; }
    .hardware-stage { padding: 10px; border: 1px solid var(--line); border-radius: 12px; display: flex; align-items: center; justify-content: space-between; gap: 10px; background: var(--bg); font-size: 11px; }
    .hardware-stage span { color: var(--muted); }
    .hardware-stage strong { color: var(--blue); text-align: right; }
    @keyframes orbit { to { transform: rotate(360deg); } }
    @keyframes float { 50% { transform: translateY(-14px) scale(1.015); } }
    @keyframes reveal { from { opacity: 0; transform: translateY(9px); } to { opacity: 1; transform: none; } }
    @media (max-width: 1180px) {
      .memory-loadout-panel { min-height: 0; }
    }
    @media (max-width: 980px) {
      .nav-links { display: none; }
      .hero-inner { grid-template-columns: 1fr; text-align: center; }
      .hero-copy .lede { margin-inline: auto; }
      .hero-actions { justify-content: center; }
      .hero-visual { min-height: 430px; }
      .product-grid { grid-template-columns: repeat(2, 1fr); }
      .play-layout, .lab-shell, .arch-detail, .approval-layout, .control-workspace { grid-template-columns: 1fr; }
      .lab-info { border-right: 0; border-bottom: 1px solid var(--line); }
      .scenario-grid { grid-template-columns: repeat(2, 1fr); }
      .safety-layers { grid-template-columns: repeat(2, 1fr); }
      .memory-product-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .memory-slot-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
    @media (max-width: 640px) {
      .section-shell { width: min(100% - 28px, 1180px); padding: 90px 0; }
      .hero-inner { width: calc(100% - 28px); padding-top: 60px; gap: 10px; }
      h1 { width: 100%; max-width: 100%; font-size: 44px; }
      .hero h1 span, .hero h1 .ink-line { max-width: 100%; white-space: normal; }
      .hero-copy { width: 100%; max-width: 100%; }
      h2 { font-size: clamp(39px, 13vw, 60px); }
      .hero-visual { min-height: 360px; }
      .orbit-chip { display: none; }
      .stats { grid-template-columns: repeat(2, 1fr); }
      .control-status { grid-template-columns: 1fr; }
      .stack-startup, .stack-fault { grid-template-columns: 1fr; }
      .stack-startup button, .stack-fault button { justify-self: start; }
      .stat:nth-child(2) { border-right: 0; }
      .stat:nth-child(-n+2) { border-bottom: 1px solid var(--line); }
      .store-toolbar { grid-template-columns: 1fr; }
      .product-grid, .extension-steps, .detail-grid, .owns-grid { grid-template-columns: 1fr; }
      .product-card { min-height: 280px; }
      .scenario-grid { grid-template-columns: 1fr; }
      .safety-layers { grid-template-columns: 1fr; }
      .pipeline { min-height: 220px; }
      .workspace-mode-switch { width: 100%; }
      .workspace-mode-button { min-width: 0; flex: 1; }
      .hardware-mode-banner, .composer-actions { align-items: stretch; flex-direction: column; }
      .hardware-mode-banner button { align-self: flex-start; }
      .composer-buttons { justify-content: stretch; }
      .composer-buttons .button { flex: 1; }
      .live-button span:not(.status-dot) { display: none; }
      .nav-inner { width: calc(100% - 22px); }
      .dialog-body { padding: 18px; }
      .hardware-power-toggle { right: 14px; bottom: 14px; min-width: 148px; min-height: 46px; padding: 0 14px; font-size: 13px; }
      .demo-mode-toggle { right: 14px; bottom: 72px; min-width: 148px; min-height: 42px; padding: 0 14px; font-size: 13px; }
      .memory-mode-toggle { right: 14px; bottom: 126px; min-width: 148px; min-height: 42px; padding: 0 14px; font-size: 13px; }
      .memory-shell { padding: 0 0 72px; }
      .memory-header { min-height: 58px; padding: 0 14px; }
      .memory-status-line { display: none; }
      .memory-layout { width: calc(100% - 28px); margin-top: 20px; gap: 40px; }
      .memory-store-hero {
        min-height: 500px; padding: 44px 20px 0; border-radius: 24px;
        grid-template-columns: 1fr; grid-template-rows: auto 220px; gap: 12px; text-align: center;
      }
      .memory-store-hero::before { width: 420px; height: 420px; transform: translate(44%, 42%); }
      .memory-hero-copy { padding-left: 0; }
      .memory-store-hero p:last-child { margin-inline: auto; }
      .memory-hero-visual { min-height: 220px; align-self: end; overflow: hidden; }
      .memory-hero-robot { width: 250px; max-height: 260px; transform: translateY(38px) scale(1.05); }
      .memory-zone-heading { margin-bottom: 14px; font-size: 24px; }
      .memory-zone-list { display: flex; gap: 9px; }
      .memory-zone-button { flex: 0 0 108px; min-width: 108px; min-height: 116px; }
      .memory-loadout-panel { min-height: 0; }
      .memory-zone-summary-top { align-items: flex-start; flex-direction: column; }
      .memory-zone-summary h2 { font-size: 40px; }
      .memory-zone-summary p { font-size: 15px; }
      .memory-equipped { padding-bottom: 36px; }
      .memory-slot-grid { grid-template-columns: 1fr; }
      .memory-product-grid { grid-template-columns: 1fr; }
      .memory-product-card { min-height: 300px; }
      .memory-section-title strong { font-size: 22px; }
      .demo-wizard { padding: 0; }
      .demo-shell { min-height: 100svh; padding: 18px 16px 22px; }
      .demo-stage-dots { margin-top: 24px; }
      .demo-content { padding-top: 24px; justify-content: flex-start; }
      .demo-choice-grid { grid-template-columns: 1fr; }
      .demo-step { grid-template-columns: 34px minmax(0, 1fr); }
      .demo-step > span:last-child { grid-column: 2; }
      .hardware-boot-overlay { padding: 0; }
      .hardware-boot-panel { min-height: 100svh; padding: 24px 18px; }
      .hardware-boot-logo { width: 72px; height: 72px; margin-bottom: 28px; }
      .hardware-boot-subtitle { margin-bottom: 34px; }
    }
    @media (prefers-reduced-motion: reduce) {
      *, *::before, *::after { scroll-behavior: auto !important; animation: none !important; transition-duration: .01ms !important; animation-duration: .01ms !important; }
    }
  </style>
</head>
<body>
  <a class="skip" href="#main">Skip to main content</a>
  <header class="nav" aria-label="Primary navigation">
    <div class="nav-inner">
      <a class="brand" href="#hero">
        <img class="brand-logo" src="/assets/hkclr-logo.jpeg" alt="HKCLR">
        <span class="brand-copy"><strong>HARNESS AGENT</strong><small>HKCLR ROBOTICS</small></span>
      </a>
      <nav class="nav-links" aria-label="Page sections">
        <a href="#control">Console</a>
        <a href="#store">Capabilities</a>
        <a href="#architecture">Architecture</a>
        <a href="#protocol">Protocol</a>
        <a href="#safety">Safety</a>
        <a href="#extend">Develop</a>
      </nav>
      <div class="nav-actions">
        <button class="live-button" id="liveButton" type="button" aria-expanded="false" aria-controls="livePanel"><span class="status-dot" id="liveDot"></span><span id="liveLabel">Local Agent</span></button>
        <button class="icon-button" id="themeButton" type="button" aria-label="Toggle color theme">◐</button>
      </div>
    </div>
  </header>
  <aside class="live-panel" id="livePanel" aria-live="polite">
    <h3>Local Agent access</h3>
    <p>The supervised Python Agent is the sole hardware owner. This page exposes only reviewed high-level Skills.</p>
    <div class="hardware-stages">
      <div class="hardware-stage"><span>Stage 1 · Read-only</span><strong id="stageOneState">Checking policy…</strong></div>
      <div class="hardware-stage"><span>Stage 2 · Approved execution</span><strong id="stageTwoState">Checking policy…</strong></div>
    </div>
    <div class="live-details" id="liveDetails">Not connected. Select “Refresh status” to query the loopback Agent.</div>
    <button class="button small" id="refreshLive" type="button" style="margin-top:12px">Refresh status</button>
  </aside>

  <main id="main">
    <section class="hero" id="hero">
      <div class="hero-inner">
        <div class="hero-copy">
          <p class="eyebrow">HKCLR · AGENTIC ROBOT RUNTIME</p>
          <h1><span class="ink-line">The agentic operating layer</span><span>for robots.</span></h1>
          <p class="lede">Harness Agent turns human intent and live world context into auditable Skill plans, then coordinates safe execution through the supervised robot runtime.</p>
          <div class="hero-actions">
            <a class="button" href="#control">Explore the runtime <span aria-hidden="true">→</span></a>
            <button class="button secondary" id="startTour" type="button">How it works</button>
          </div>
        </div>
        <div class="hero-visual" aria-label="Interactive system data flow">
          <button class="soul-button" id="soulButton" type="button" aria-pressed="false" data-open="false" aria-label="Show the Harness Agent operating flow">
            <span class="agent-card-head">
              <img class="agent-card-logo" src="/assets/hkclr-logo.jpeg" alt="">
              <span class="agent-card-brand"><strong>HARNESS AGENT</strong><small>AGENTIC OPERATING LAYER</small></span>
              <span class="runtime-pill">RUNTIME</span>
            </span>
            <span class="agent-card-title">One intelligent layer from goal to verified action.</span>
            <span class="agent-loop">
              <span><b>01 · INTENT</b>Understand the goal</span>
              <span><b>02 · CONTEXT</b>Observe the world</span>
              <span><b>03 · SKILLS</b>Select capabilities</span>
              <span><b>04 · PLAN</b>Coordinate execution</span>
            </span>
            <span class="agent-boundary"><span>Hardware boundary</span><strong>Python Agent → Control Stack</strong></span>
          </button>
          <span class="core-label" id="coreLabel">SELECT TO TRACE THE OPERATING FLOW</span>
          <span class="orbit-chip one">Intent → Context</span>
          <span class="orbit-chip two">Skill Registry</span>
          <span class="orbit-chip three">Policy Gate</span>
          <div class="hero-footnote">Agent orchestration · Auditable Plans · Supervised execution</div>
        </div>
      </div>
    </section>
    <div class="stats" aria-label="Capability overview">
      <div class="stat"><strong>5</strong><span>Composite Skills</span></div>
      <div class="stat"><strong>8</strong><span>Atomic Tools</span></div>
      <div class="stat"><strong>5</strong><span>Harness Tools</span></div>
      <div class="stat"><strong>2×</strong><span>Server-side execution gates</span></div>
    </div>

    <section class="control-center" id="control">
      <div class="section-shell">
        <header class="section-head">
          <p class="eyebrow">ROBOT STATUS &amp; OPERATIONS</p>
          <h2>Check readiness first, then move from simulation to supervised execution.</h2>
          <p class="lede">This dashboard reports the Python Agent, active Task, control-stack lifecycle, and enforced execution policy. Hardware motion remains locked until an operator reviews and approves a Plan.</p>
        </header>
        <div class="control-status" aria-live="polite">
          <div class="control-status-card"><span>Supervised Python Agent</span><strong id="opsAgentState">Checking…</strong></div>
          <div class="control-status-card"><span>Control-stack lifecycle</span><strong id="opsStackState">Checking…</strong></div>
          <div class="control-status-card"><span>Execution policy</span><strong id="opsPolicyState">Checking…</strong></div>
          <div class="control-status-card"><span>Active hardware Task</span><strong id="opsTaskState">Checking…</strong></div>
        </div>
        <div class="stack-startup" id="stackStartupPanel" aria-live="polite">
          <div>
            <h3 id="stackStartupTitle">One-click robot startup</h3>
            <p id="stackStartupDetail">Checking whether local operator startup is available…</p>
          </div>
          <button class="button" id="startHardwareStack" type="button" disabled>Checking…</button>
        </div>
        <div class="stack-fault" id="stackFaultPanel" hidden aria-live="assertive">
          <div>
            <h3 id="stackFaultTitle">Hardware execution is locked</h3>
            <p id="stackFaultDetail">This dashboard remains online. Inspect the robot and emergency stop before reinitializing the hardware stack.</p>
          </div>
          <button class="button amber-button" id="recoverHardwareStack" type="button">Reinitialize hardware</button>
        </div>
        <div class="control-workspace">
          <div class="operation-step">
            <span class="step-number">STEP 01 · SIMULATE</span>
            <h3>Validate the task without moving hardware.</h3>
            <p>Enter a goal in plain English and inspect the generated high-level Skill sequence, validation results, and expected events.</p>
          </div>
          <div class="operation-step">
            <span class="step-number">STEP 02 · VERIFY</span>
            <h3>Inspect the capabilities used by the Plan.</h3>
            <p>Confirm each Skill's purpose, parameters, dependencies, return data, and hardware safety level before execution.</p>
            <a class="button secondary" href="#store">Browse capabilities</a>
          </div>
          <div class="operation-step">
            <span class="step-number">STEP 03 · APPROVE</span>
            <h3>Review pending hardware requests.</h3>
            <p>A planning client may submit an execution request, but only this panel can complete the safety checklist and approve that specific Plan.</p>
          </div>
          <div class="control-note">
            <strong>Emergency procedure</strong>
            <span>Use the physical emergency stop for unexpected motion. Web cancellation is cooperative task control and is not an emergency stop.</span>
          </div>
        </div>
      </div>
    </section>

    <section id="store">
      <div class="section-shell">
        <header class="section-head">
          <p class="eyebrow">CAPABILITY CATALOG</p>
          <h2>Know what every Skill and Tool does before you use it.</h2>
          <p class="lede">Search the reviewed capability catalog. Open any card to inspect its parameters, dependencies, return type, source, and hardware safety level.</p>
        </header>
        <div class="store-toolbar">
          <label class="search-wrap"><span aria-hidden="true">⌕</span><span class="skip">Search capabilities</span><input class="search" id="catalogSearch" type="search" placeholder="Search: grasp, perception, reset…" autocomplete="off"></label>
          <div class="filters" id="kindFilters" aria-label="Capability filters">
            <button class="filter" type="button" data-kind="all" aria-pressed="true">All</button>
            <button class="filter" type="button" data-kind="skill" aria-pressed="false">Skill</button>
            <button class="filter" type="button" data-kind="tool" aria-pressed="false">Atomic Tool</button>
            <button class="filter" type="button" data-kind="harness-tool" aria-pressed="false">Harness Tool</button>
            <button class="filter" id="dependencyFilter" type="button" aria-pressed="false">Has dependencies</button>
            <label><span class="skip">Safety level</span><select class="safety-select" id="safetyFilter" aria-label="Filter by safety level"><option value="all">All safety levels</option><option value="L0">L0 · Read-only</option><option value="L1">L1 · Plan/request</option><option value="L2">L2 · Hardware motion</option><option value="L3">L3 · Control stack</option></select></label>
          </div>
        </div>
        <div class="catalog-meta"><span id="catalogCount">18 capabilities</span><span>Use arrow keys to browse · Enter to open</span></div>
        <div class="product-grid" id="productGrid" role="list"></div>
      </div>
    </section>

    <section class="playground" id="playground" hidden aria-hidden="true">
      <div class="section-shell">
        <header class="section-head">
          <p class="eyebrow">DUAL-MODE PLAN WORKSPACE</p>
          <h2>Plan safely in simulation, then run through the supervised robot path.</h2>
          <p class="lede" id="playgroundLead">Simulation never connects to hardware. Switch to Real robot only from the local operator console when you are beside the robot.</p>
        </header>
        <div class="workspace-mode-switch" id="workspaceModeSwitch" role="group" aria-label="Planning mode">
          <button class="workspace-mode-button" type="button" data-workspace-mode="simulation" aria-pressed="true">Simulation</button>
          <button class="workspace-mode-button" type="button" data-workspace-mode="hardware" aria-pressed="false">Real robot</button>
        </div>
        <div class="hardware-mode-banner" id="hardwareModeBanner" hidden aria-live="polite">
          <div>
            <p class="eyebrow">REAL ROBOT · LOCAL OPERATOR ONLY</p>
            <h3 id="workspaceHardwareTitle">Checking robot readiness…</h3>
            <p id="workspaceHardwareDetail">The control stack must be started before a hardware Plan can be created.</p>
          </div>
          <button class="button amber-button" id="workspaceStartHardware" type="button" disabled>Checking…</button>
        </div>
        <div class="scenario-row" aria-label="Example goals">
          <button class="scenario" type="button">Tidy the table</button>
          <button class="scenario" type="button">Place the blue cup beside the yellow plate</button>
          <button class="scenario" type="button">Find the green bowl and the bread</button>
          <button class="scenario" type="button">Return both arms to the safe pose</button>
        </div>
        <div class="play-layout">
          <form class="console-panel" id="planForm">
            <div class="panel-head"><span class="panel-title">Task goal</span><span class="mode-pill" id="planModePill">SIMULATION</span></div>
            <div class="composer">
              <label for="instruction" class="skip">Robot task goal</label>
              <textarea id="instruction" maxlength="2000" placeholder="Example: Place the blue cup beside the yellow plate">Place the blue cup beside the yellow plate</textarea>
              <div class="composer-hint" id="planHint">Describe the result you want. Do not enter joint angles, force, velocity, CAN, or shell commands.</div>
              <div class="composer-actions">
                <span class="simulation-badge" id="planConnectionBadge">No hardware connection</span>
                <div class="composer-buttons">
                  <button class="button secondary" id="reviewHardwarePlan" type="button" hidden>Review and execute</button>
                  <button class="button" id="planButton" type="submit">Generate simulated Plan</button>
                </div>
              </div>
            </div>
          </form>
          <div class="console-panel">
            <div class="panel-head"><span class="panel-title">Skill pipeline</span><button class="dark-tab" id="replayPlan" type="button" disabled>Replay timeline</button></div>
            <div class="pipeline" id="pipeline"><div class="pipeline-empty">Enter a goal to see the selected Skills and their Tool dependencies.</div></div>
          </div>
          <div class="console-panel">
            <div class="panel-head">
              <span class="panel-title">Raw plan data</span>
              <div class="result-tabs" role="tablist">
                <button class="dark-tab" id="jsonTab" type="button" role="tab" aria-selected="true">JSON</button>
                <button class="dark-tab" id="ndjsonTab" type="button" role="tab" aria-selected="false">NDJSON</button>
                <button class="dark-tab" id="copyRaw" type="button">Copy</button>
              </div>
            </div>
            <pre class="raw" id="rawOutput" tabindex="0">Waiting for a Plan…</pre>
          </div>
          <div class="console-panel">
            <div class="panel-head"><span class="panel-title" id="planEventsTitle">Simulation events</span><span class="mode-pill" id="planEventsPill">NO MOTION</span></div>
            <div class="timeline" id="timeline"><div class="pipeline-empty">Events will explain how the Plan was built.</div></div>
          </div>
        </div>
      </div>
    </section>

    <section class="architecture" id="architecture">
      <div class="section-shell">
        <header class="section-head">
          <p class="eyebrow">ARCHITECTURE EXPLORER</p>
          <h2>Trace a request from the browser to the physical control boundary.</h2>
          <p class="lede">Select any layer to see what it owns, what it explicitly refuses, and which protocol connects it to the next process.</p>
        </header>
        <div class="architecture-flow" id="architectureFlow" aria-label="Robot system data flow"></div>
        <div class="arch-detail" id="architectureDetail" aria-live="polite"></div>
      </div>
    </section>

    <section class="protocol" id="protocol">
      <div class="section-shell">
        <header class="section-head">
          <p class="eyebrow">PROTOCOL LAB</p>
          <h2>Inspect the exact NDJSON protocol without contacting hardware.</h2>
          <p class="lede">Edit an envelope and run it locally. The sandbox accepts only health, manifest, and plan; execute, cancel, task, and call are always rejected.</p>
        </header>
        <div class="lab-shell">
          <div class="lab-info">
            <h3>Select a request</h3>
            <div class="protocol-tabs" id="protocolTabs" role="tablist">
              <button class="protocol-tab" type="button" role="tab" data-protocol="health" aria-selected="true"><strong>health</strong><br><small>Check process mode and readiness</small></button>
              <button class="protocol-tab" type="button" role="tab" data-protocol="manifest" aria-selected="false"><strong>manifest</strong><br><small>Read the Tool and Skill catalog</small></button>
              <button class="protocol-tab" type="button" role="tab" data-protocol="plan" aria-selected="false"><strong>plan</strong><br><small>Create a non-executable preview</small></button>
            </div>
            <div class="field-help"><strong>Envelope fields</strong><br><code>id</code> matches requests to responses. <code>method</code> selects one narrow operation. <code>params</code> carries only declared arguments. <code>ok</code> selects either <code>data</code> or <code>error</code>.<br><br><strong>Plan ≠ Task</strong><br>A Plan is inert. A Task is created only after both server gates are open, the Skill allowlist passes, the on-site checklist is complete, and the confirmation phrase matches exactly.</div>
          </div>
          <div class="lab-console">
            <label for="protocolEditor">One JSON request envelope per line</label>
            <textarea class="protocol-editor" id="protocolEditor" spellcheck="false"></textarea>
            <div class="lab-actions"><span class="simulation-badge">Local protocol sandbox</span><button class="button small" id="runProtocol" type="button">Run request</button></div>
            <div class="protocol-output" id="protocolOutput" role="status">Waiting for a request…</div>
          </div>
        </div>
      </div>
    </section>

    <section id="safety">
      <div class="section-shell">
        <header class="section-head">
          <p class="eyebrow">SAFETY MODEL</p>
          <h2>No single check is trusted to protect the entire system.</h2>
          <p class="lede">The website, Cordis Provider, Python Agent, lifecycle supervisor, and physical emergency stop enforce separate responsibilities.</p>
        </header>
        <div class="safety-layers">
          <div class="safety-layer" data-level="01 · WEBSITE"><h3>Isolated write routes</h3><p>Execution APIs are disabled by default and require both server gates, a short-lived Plan, and an on-site operator.</p></div>
          <div class="safety-layer" data-level="02 · CORDIS"><h3>No approval authority</h3><p>The model may read, plan, request, and query. It has no approval phrase or direct execute method.</p></div>
          <div class="safety-layer" data-level="03 · PYTHON AGENT"><h3>Runtime validation</h3><p>The Agent checks source, Skill allowlist, motion budget, Plan lifetime, readiness, and one active Task at a time.</p></div>
          <div class="safety-layer" data-level="04 · SUPERVISOR + E-STOP"><h3>Independent shutdown</h3><p>The launcher rolls back unhealthy services. Use the physical emergency stop for unexpected motion.</p></div>
        </div>
        <div class="game">
          <div class="game-top"><div><p class="eyebrow">ALLOW OR DENY?</p><h3>Test your understanding of the execution boundary</h3></div><div class="score" id="safetyScore">0 / 5 answered</div></div>
          <div class="scenario-grid" id="safetyGrid"></div>
        </div>
      </div>
    </section>

    <section class="hardware-bay" id="hardware" hidden aria-hidden="true">
      <div class="section-shell">
        <header class="section-head">
          <p class="eyebrow">OPERATOR APPROVAL</p>
          <h2>Review the complete Plan before creating a hardware Task.</h2>
          <p class="lede">This surface is separate from the simulator and model. It rejects joint, CAN, shell, atomic Tool, and arbitrary call inputs; only four reviewed high-level Skills are accepted.</p>
        </header>
        <div class="hardware-status">
          <span>Current execution policy</span>
          <strong id="hardwarePolicyState">Reading server gates…</strong>
        </div>
        <div class="approval-layout">
          <div class="approval-card" style="grid-column:1/-1">
            <p class="eyebrow">AI CONTROL · PER-PLAN APPROVAL</p>
            <h3>Harness Agent approval queue</h3>
            <p>The AI can submit only an existing server Plan. It never receives the confirmation phrase. The Python Agent creates a hardware Task only after your approval.</p>
            <div class="task-console" id="harnessApprovalQueue">Waiting for a Harness Agent execution request…</div>
            <div class="approval-actions">
              <button class="button secondary small" id="refreshHarnessApprovals" type="button" disabled>Refresh requests</button>
              <button class="button secondary small cancel-button" id="denyHarnessApproval" type="button" disabled>Deny current Plan</button>
            </div>
          </div>
          <div class="approval-card">
            <p class="eyebrow">01 · CREATE PLAN</p>
            <h3>Create a short-lived hardware Plan</h3>
            <p>This step plans only. The Plan must come from this website process and expires after ten minutes.</p>
            <form id="hardwarePlanForm">
              <label for="hardwareInstruction">Semantic task goal</label>
              <textarea id="hardwareInstruction" maxlength="2000" disabled>Place the blue cup beside the yellow plate</textarea>
              <div class="approval-actions">
                <button class="button secondary small" id="usePlaygroundGoal" type="button">Use Playground goal</button>
                <button class="button amber-button" id="hardwarePlanButton" type="submit" disabled>Create hardware Plan</button>
              </div>
            </form>
            <div class="hardware-lock" id="hardwareLock">Hardware execution is locked by default. Simulation remains available.</div>
            <div id="hardwarePlanPreview" hidden>
              <div class="approval-steps" id="hardwarePlanSteps"></div>
              <p id="hardwarePlanExpiry"></p>
            </div>
          </div>
          <div class="approval-card">
            <p class="eyebrow">02 · ON-SITE CHECKS</p>
            <h3>Confirm each physical condition</h3>
            <p>A checked box means the on-site operator personally completed the inspection; it is not a software sensor reading.</p>
            <form id="hardwareExecuteForm">
              <div class="checklist">
                <label class="check-item"><input type="checkbox" data-approval-check disabled><span>I will remain beside the robot and observe the complete Task.</span></label>
                <label class="check-item"><input type="checkbox" data-approval-check disabled><span>The workspace is clear of people, cables, and unknown obstacles.</span></label>
                <label class="check-item"><input type="checkbox" data-approval-check disabled><span>The independent physical emergency stop is reachable and verified.</span></label>
                <label class="check-item"><input type="checkbox" data-approval-check disabled><span>The robot, grippers, cameras, and CAN state have been inspected.</span></label>
              </div>
              <label class="confirmation-label" for="hardwareConfirmation">Enter the exact confirmation phrase <code>ENABLE_DUAL_V2_2_HARDWARE</code>
                <input id="hardwareConfirmation" type="text" autocomplete="off" spellcheck="false" disabled>
              </label>
              <div class="approval-actions">
                <button class="button amber-button" id="hardwareExecuteButton" type="submit" disabled>Approve and create Task</button>
              </div>
              <p class="danger-note">This button can trigger real robot motion when every gate passes. Cooperative cancellation is not an emergency stop.</p>
            </form>
          </div>
          <div class="approval-card">
            <p class="eyebrow">03 · TASK STATUS</p>
            <h3>Authoritative Task state</h3>
            <div class="task-console" id="hardwareTaskStatus">No hardware Task has been created.</div>
            <div class="approval-actions">
              <button class="button secondary small" id="refreshHardwareTask" type="button" disabled>Refresh status</button>
              <button class="button secondary small cancel-button" id="cancelHardwareTask" type="button" disabled>Request cooperative cancel</button>
            </div>
          </div>
          <div class="approval-card">
            <p class="eyebrow">04 · AGENT EVENTS</p>
            <h3>Execution timeline</h3>
            <div class="task-console" id="hardwareTaskEvents">Events come from the Python Agent; the browser cannot claim hardware success.</div>
          </div>
        </div>
      </div>
    </section>

    <section class="extension" id="extend">
      <div class="section-shell">
        <header class="section-head">
          <p class="eyebrow">DEVELOPER GUIDE</p>
          <h2>Add a capability without bypassing the architecture.</h2>
          <p class="lede">Start with one typed Tool, compose it into a Skill, and expose it through the Provider. Keep planning, execution, and lifecycle responsibilities separate.</p>
        </header>
        <div class="extension-steps" id="extensionSteps"></div>
      </div>
    </section>
  </main>

  <footer><strong>HKCLR Harness Agent</strong>Intent understanding · World context · Skill orchestration · Auditable execution<br><small>The agentic operating layer delegates hardware ownership to the supervised Python Agent and control stack.</small></footer>

  <dialog id="productDialog" aria-labelledby="productDialogTitle">
    <div class="dialog-head"><strong id="productDialogTitle">Capability details</strong><button class="close" type="button" data-close="productDialog" aria-label="Close">×</button></div>
    <div class="dialog-body" id="productDialogBody"></div>
  </dialog>
  <dialog id="tourDialog" aria-labelledby="tourTitle">
    <div class="dialog-head"><strong id="tourTitle">Robot operation in 60 seconds</strong><button class="close" type="button" data-close="tourDialog" aria-label="Close">×</button></div>
    <div class="dialog-body">
      <div class="tour-progress" id="tourProgress" aria-label="Guide progress"></div>
      <div class="tour-card" id="tourCard"></div>
      <div class="tour-actions"><button class="button secondary small" id="tourBack" type="button">Back</button><button class="button small" id="tourNext" type="button">Next</button></div>
    </div>
  </dialog>
  <div class="demo-wizard" id="demoWizard" role="dialog" aria-modal="true" aria-labelledby="demoTitle" hidden>
    <div class="demo-shell">
      <div class="demo-topbar">
        <div class="demo-brand"><img src="/assets/hkclr-logo.jpeg" alt="">GUIDED ROBOT DEMO</div>
        <button class="demo-close" id="demoClose" type="button" aria-label="Close guided demo">×</button>
      </div>
      <div class="demo-stage-dots" id="demoStageDots" aria-label="Demo progress"></div>
      <div class="demo-content">
        <p class="demo-eyebrow" id="demoEyebrow">STEP 1 OF 5</p>
        <h2 class="demo-title" id="demoTitle">What should the robot do?</h2>
        <p class="demo-copy" id="demoCopy">Describe the result in one sentence, then press Enter.</p>
        <div id="demoBody"></div>
      </div>
      <div class="demo-actions" id="demoActions"></div>
    </div>
  </div>
  <button class="memory-mode-toggle" id="memoryModeToggle" type="button">Memory</button>
  <div class="memory-overlay" id="memoryOverlay" role="dialog" aria-modal="true" aria-labelledby="memoryTitle" hidden>
    <div class="memory-shell">
      <header class="memory-header">
        <div class="memory-identity">
          <img class="memory-brand-logo" src="/assets/hkclr-logo.jpeg" alt="">
          <span><strong id="memoryTitle">Memory Store</strong><small>HKCLR ROBOTICS</small></span>
        </div>
        <span class="memory-status-line"><b id="memoryCatalogCount">18</b> memories · <b id="memoryEquippedCount">0</b> installed</span>
        <button class="memory-close" id="memoryClose" type="button" aria-label="Close Memory Store">×</button>
      </header>
      <div class="memory-layout">
        <section class="memory-store-hero" aria-labelledby="memoryHeroTitle">
          <div class="memory-hero-copy">
            <p class="memory-hero-eyebrow">MEMORY FOR DUAL V2.2</p>
            <h1 id="memoryHeroTitle">More capable.<br>Uniquely yours.</h1>
            <p>Choose the memories your robot needs. Install a capability in one click, and keep every system focused.</p>
          </div>
          <div class="memory-hero-visual" aria-hidden="true">
            <img class="memory-hero-robot" src="/assets/memory-robot-minimal.png" alt="">
          </div>
        </section>
        <aside class="memory-zone-rail" aria-label="Robot systems">
          <h2 class="memory-zone-heading">Choose a system.</h2>
          <div class="memory-zone-list" id="memoryZoneList"></div>
        </aside>
        <aside class="memory-loadout-panel" aria-live="polite">
          <section class="memory-zone-summary">
            <div class="memory-zone-summary-top">
              <div><h2 id="memoryZoneTitle">VISUAL</h2><p id="memoryZoneDescription">Perception, semantic grounding, grasp estimation, and robot-frame localization.</p></div>
              <span class="memory-zone-code" id="memoryZoneCode">Vision</span>
            </div>
          </section>
          <div class="memory-available">
            <section class="memory-equipped">
              <div class="memory-section-title"><strong>Installed</strong><span id="memorySlotUsage">0 of 3</span></div>
              <div class="memory-slot-grid" id="memorySlotGrid"></div>
            </section>
            <section>
              <div class="memory-section-title"><strong id="memoryAvailableTitle">Explore Vision</strong><span>Choose Add to install</span></div>
              <div class="memory-available-list" id="memoryAvailableList"></div>
            </section>
          </div>
        </aside>
      </div>
    </div>
  </div>
  <button class="demo-mode-toggle" id="demoModeToggle" type="button">Guided demo</button>
  <div class="hardware-boot-overlay" id="hardwareBootOverlay" data-state="loading" role="dialog" aria-modal="true" aria-labelledby="hardwareBootTitle" hidden>
    <div class="hardware-boot-panel">
      <img class="hardware-boot-logo" src="/assets/hkclr-logo.jpeg" alt="">
      <h3 class="hardware-boot-title" id="hardwareBootTitle">Starting robot hardware</h3>
      <p class="hardware-boot-subtitle" id="hardwareBootSubtitle">Preparing the supervised hardware runtime…</p>
      <div class="hardware-boot-meta" aria-live="polite">
        <span id="hardwareBootOperation">Submitting startup request</span>
        <strong id="hardwareBootPercent">2%</strong>
      </div>
      <div class="hardware-boot-track" role="progressbar" id="hardwareBootTrack" aria-valuemin="0" aria-valuemax="100" aria-valuenow="2" aria-labelledby="hardwareBootOperation">
        <div class="hardware-boot-progress" id="hardwareBootProgress"></div>
      </div>
      <div class="hardware-boot-error" id="hardwareBootError" role="alert" hidden></div>
      <button class="hardware-boot-close" id="hardwareBootClose" type="button" hidden>Close</button>
      <p class="hardware-boot-note">Use the physical emergency stop for unexpected motion.</p>
    </div>
  </div>
  <button class="hardware-power-toggle" id="hardwarePowerToggle" type="button" data-state="unavailable" data-action="start" disabled aria-label="Hardware status unavailable">
    <span class="hardware-power-indicator" aria-hidden="true"></span>
    <span id="hardwarePowerLabel">Checking hardware…</span>
  </button>
  <div class="toast" id="toast" role="status" aria-live="polite"></div>
  <div class="skip" id="announcer" aria-live="polite"></div>
  <script id="bootstrap" type="application/json">${bootstrap}</script>
  <script>
    (function () {
      'use strict';
      var boot = JSON.parse(document.getElementById('bootstrap').textContent);
      var catalog = boot.catalog;
      var content = boot.content;
      var state = {
        kind: 'all',
        safety: 'all',
        hasDependencies: false,
        query: '',
        workspaceMode: 'simulation',
        opsSnapshot: null,
        plan: null,
        simulationPlan: null,
        hardwareWorkspacePlan: null,
        rawMode: 'json',
        protocol: 'health',
        replayTimers: [],
        safetyAnswered: 0,
        tourIndex: 0,
        hardwarePolicy: null,
        hardwarePlan: null,
        hardwareApprovalRequest: null,
        hardwareApprovalTimer: null,
        hardwareTaskId: null,
        hardwareCursor: 0,
        hardwareEvents: [],
        hardwarePollTimer: null,
        opsTimer: null,
        startupInFlight: false,
        stopInFlight: false,
        recoveryInFlight: false,
        bootOverlayActive: false,
        bootProgress: 0,
        bootHideTimer: null,
        lifecycleRequestStartedAt: 0,
        demoInstruction: '',
        demoPlan: null,
        demoHardwarePlan: null,
        demoTaskId: null,
        demoCursor: 0,
        demoEvents: [],
        demoTimers: [],
        demoPollTimer: null,
        demoPollFailures: 0,
        demoLastProgress: 0,
        demoResumeAfterStartup: false,
        demoResumeScheduled: false,
        memoryZone: 'visual',
        memoryEquipped: {}
      };
      var MEMORY_STORAGE_KEY = 'robot-soul-memory-loadout-v1';
      var memoryZones = [
        {
          id: 'visual',
          label: 'VISUAL',
          subtitle: 'OPTICAL STACK',
          code: 'Vision',
          slots: 3,
          description: 'Perception, semantic grounding, grasp estimation, and robot-frame localization.',
          products: ['perceive_scene', 'ground', 'estimate_grasp_poses', 'localize'],
          defaults: ['perceive_scene', 'ground', 'localize']
        },
        {
          id: 'hand',
          label: 'HANDS',
          subtitle: 'GRIP SYSTEM',
          code: 'Hands',
          slots: 3,
          description: 'End-effector approach, grasp, placement, and coordinated pick-and-place capabilities.',
          products: ['pick_and_place', 'approach', 'grasp', 'place'],
          defaults: ['pick_and_place', 'grasp', 'place']
        },
        {
          id: 'os',
          label: 'OPERATING SYSTEM',
          subtitle: 'AGENT MODEL',
          code: 'Agent model',
          slots: 4,
          description: 'Agent-model planning, capability discovery, approval requests, and execution telemetry.',
          products: ['robot_status', 'robot_manifest', 'robot_plan', 'robot_request_execution', 'robot_execution_status', 'tidy_table'],
          defaults: ['robot_plan', 'robot_status', 'robot_execution_status', 'tidy_table']
        },
        {
          id: 'haptic',
          label: 'HAPTIC',
          subtitle: 'FORCE SENSE',
          code: 'Tactile',
          slots: 2,
          description: 'Contact-sensitive grasp and release operations associated with force-aware manipulation.',
          products: ['grasp', 'place', 'pick_and_place'],
          defaults: ['grasp']
        },
        {
          id: 'chassis',
          label: 'CHASSIS',
          subtitle: 'CONTROL BASE',
          code: 'Mobile base',
          slots: 3,
          description: 'Control-stack startup, transport phases, and system-wide reset capabilities.',
          products: ['start_robot', 'transport', 'reset', 'safety_reset'],
          defaults: ['start_robot', 'reset']
        },
        {
          id: 'arm',
          label: 'ARMS',
          subtitle: 'MOTION CHAIN',
          code: 'Dual-arm motion',
          slots: 4,
          description: 'Dual-arm motion primitives, transport, safe recovery, and composite manipulation.',
          products: ['approach', 'transport', 'reset', 'safety_reset', 'pick_and_place', 'grasp', 'place'],
          defaults: ['approach', 'transport', 'safety_reset']
        },
        {
          id: 'waist',
          label: 'WAIST',
          subtitle: 'POSTURE LINK',
          code: 'Posture',
          slots: 2,
          description: 'Whole-body posture recovery represented by reset, safety, and transport capabilities.',
          products: ['reset', 'safety_reset', 'transport'],
          defaults: ['reset']
        }
      ];
      var byId = function (id) { return document.getElementById(id); };
      var esc = function (value) {
        return String(value).replace(/[&<>"']/g, function (char) {
          return ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[char];
        });
      };
      var toastTimer;
      function toast(message) {
        var node = byId('toast');
        node.textContent = message;
        node.classList.add('show');
        clearTimeout(toastTimer);
        toastTimer = setTimeout(function () { node.classList.remove('show'); }, 2300);
      }
      function announce(message) { byId('announcer').textContent = message; }
      function pretty(value) { return JSON.stringify(value, null, 2); }
      async function requestJson(path, options) {
        var response = await fetch(path, options);
        var body = await response.json();
        if (!response.ok || body.ok === false) {
          throw new Error(body && body.error && body.error.message ? body.error.message : 'Request failed');
        }
        return body;
      }

      var savedTheme = localStorage.getItem('robot-soul-theme');
      if (savedTheme === 'dark' || savedTheme === 'light') {
        document.documentElement.dataset.theme = savedTheme;
      } else if (window.matchMedia('(prefers-color-scheme: dark)').matches) {
        document.documentElement.dataset.theme = 'dark';
      }
      byId('themeButton').addEventListener('click', function () {
        var next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
        document.documentElement.dataset.theme = next;
        localStorage.setItem('robot-soul-theme', next);
        toast(next === 'dark' ? 'Dark theme enabled' : 'Light theme enabled');
      });

      byId('soulButton').addEventListener('click', function (event) {
        var open = event.currentTarget.getAttribute('aria-pressed') !== 'true';
        event.currentTarget.setAttribute('aria-pressed', String(open));
        event.currentTarget.dataset.open = String(open);
        byId('coreLabel').textContent = open
          ? 'INTENT → CONTEXT → SKILLS → PLAN → EXECUTION'
          : 'SELECT TO TRACE THE OPERATING FLOW';
        announce(open ? 'Harness Agent operating flow expanded' : 'Harness Agent operating flow collapsed');
      });

      function kindLabel(kind) {
        return kind === 'skill' ? 'Composite Skill' : kind === 'tool' ? 'Atomic Tool' : 'Harness Tool';
      }
      function iconForKind(kind) {
        if (kind === 'skill') {
          return '<svg viewBox="0 0 24 24"><circle cx="6" cy="7" r="2.5"></circle><circle cx="18" cy="7" r="2.5"></circle><circle cx="12" cy="17" r="2.5"></circle><path d="M8.2 8.3 10.7 15M15.8 8.3 13.3 15M8.5 7h7"></path></svg>';
        }
        if (kind === 'tool') {
          return '<svg viewBox="0 0 24 24"><path d="M8 3.5h8l4.5 8-4.5 9H8l-4.5-9z"></path><circle cx="12" cy="12" r="3"></circle></svg>';
        }
        return '<svg viewBox="0 0 24 24"><path d="M12 3 21 12 12 21 3 12z"></path><path d="M8.5 12h7M12 8.5v7"></path></svg>';
      }
      function memoryZoneById(id) {
        return memoryZones.find(function (zone) { return zone.id === id; }) || memoryZones[0];
      }
      function memoryProductByName(name) {
        return catalog.find(function (item) { return item.name === name; });
      }
      function memoryKindLabel(kind) {
        return kind === 'skill' ? 'Composite Skill' : kind === 'tool' ? 'Atomic Tool' : 'Harness Interface';
      }
      function memoryZoneIcon(zoneId) {
        var icons = {
          visual: '<svg viewBox="0 0 24 24"><rect x="3" y="5.5" width="18" height="13" rx="3"/><circle cx="12" cy="12" r="4"/><path d="M7 5.5 8.5 3.5h7L17 5.5"/></svg>',
          hand: '<svg viewBox="0 0 24 24"><path d="M7.5 11V6.8a1.4 1.4 0 0 1 2.8 0V10M10.3 9V5.8a1.4 1.4 0 0 1 2.8 0V10M13.1 9V6.5a1.4 1.4 0 0 1 2.8 0v4M15.9 9a1.4 1.4 0 0 1 2.8 0v5.2c0 4-2.6 6.3-6.2 6.3h-.7c-2.1 0-3.8-1-4.9-2.7l-2.1-3.2a1.5 1.5 0 0 1 2.3-1.9l2 1.8"/></svg>',
          os: '<svg viewBox="0 0 24 24"><rect x="6" y="6" width="12" height="12" rx="3"/><rect x="9" y="9" width="6" height="6" rx="1.5"/><path d="M9 2.5V6M15 2.5V6M9 18v3.5M15 18v3.5M2.5 9H6M2.5 15H6M18 9h3.5M18 15h3.5"/></svg>',
          haptic: '<svg viewBox="0 0 24 24"><path d="M8.2 17.8c1.1-1.8 1.5-4 1.3-6.2a2.5 2.5 0 0 1 5-.5c.4 4.1-.5 7.5-2.1 10M5.2 15.8c.7-1.6.8-3.3.6-5a6.2 6.2 0 0 1 12.4-1.2c.4 4-.1 7.4-1.4 10.4M4.2 6.8A9.2 9.2 0 0 1 20.5 5"/></svg>',
          chassis: '<svg viewBox="0 0 24 24"><path d="M5 7h14l2 8H3l2-8Z"/><path d="M7 7V4.5h10V7M6 15v2M18 15v2"/><circle cx="6" cy="19" r="2"/><circle cx="18" cy="19" r="2"/></svg>',
          arm: '<svg viewBox="0 0 24 24"><circle cx="7" cy="17" r="2.5"/><circle cx="15" cy="9" r="2.5"/><path d="m8.8 15.2 4.4-4.4M16.8 10.7l2.7 2.8v4M4 20h8M13.5 6.8 10.7 4H6.5"/></svg>',
          waist: '<svg viewBox="0 0 24 24"><path d="M8 4h8l2 6-2 10H8L6 10l2-6Z"/><path d="M6 10h12M9 14c1.8 1.4 4.2 1.4 6 0"/><path d="m4 13-2-2 2-2M20 9l2 2-2 2"/></svg>'
        };
        return icons[zoneId] || '<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="8"/></svg>';
      }
      function memoryIconTone(name) {
        return String(Array.from(name).reduce(function (total, char) { return total + char.charCodeAt(0); }, 0) % 6);
      }
      function memoryProductIcon(product) {
        var icons = {
          robot_status: '<svg viewBox="0 0 24 24"><path d="M3.5 12h3l2-5 4 10 2.5-5h5.5"/><path d="M5 4.5h14a2 2 0 0 1 2 2v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-11a2 2 0 0 1 2-2Z"/></svg>',
          robot_manifest: '<svg viewBox="0 0 24 24"><path d="M7 5h13M7 12h13M7 19h13"/><path d="m3 5 .8.8L5.5 4M3 12l.8.8 1.7-1.8M3 19l.8.8 1.7-1.8"/></svg>',
          robot_plan: '<svg viewBox="0 0 24 24"><circle cx="5" cy="6" r="2"/><circle cx="19" cy="18" r="2"/><circle cx="12" cy="11" r="2"/><path d="m6.8 6.9 3.4 3M13.7 12.4l3.7 4.1M7 18h4v-4"/></svg>',
          robot_request_execution: '<svg viewBox="0 0 24 24"><path d="M12 3 20 6v5c0 5.1-3.4 8.6-8 10-4.6-1.4-8-4.9-8-10V6l8-3Z"/><path d="m10 8 5 3-5 3Z"/></svg>',
          robot_execution_status: '<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="8.5"/><path d="M12 7v5l3.5 2M4 12H2M22 12h-2"/></svg>',
          tidy_table: '<svg viewBox="0 0 24 24"><rect x="3.5" y="5" width="17" height="14" rx="2"/><path d="M9 5v14M15 5v14M3.5 11h17M18.5 2.5v4M16.5 4.5h4"/></svg>',
          start_robot: '<svg viewBox="0 0 24 24"><path d="M12 3v8"/><path d="M7.2 5.8a8 8 0 1 0 9.6 0"/></svg>',
          perceive_scene: '<svg viewBox="0 0 24 24"><path d="M2.5 12s3.4-5.5 9.5-5.5 9.5 5.5 9.5 5.5-3.4 5.5-9.5 5.5S2.5 12 2.5 12Z"/><circle cx="12" cy="12" r="2.6"/></svg>',
          ground: '<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/></svg>',
          estimate_grasp_poses: '<svg viewBox="0 0 24 24"><path d="M7 4H4v5M17 4h3v5M7 20H4v-5M17 20h3v-5"/><circle cx="9" cy="12" r="2"/><circle cx="15" cy="9" r="2"/><circle cx="15" cy="15" r="2"/></svg>',
          localize: '<svg viewBox="0 0 24 24"><path d="M12 21s6-5.8 6-11a6 6 0 1 0-12 0c0 5.2 6 11 6 11Z"/><circle cx="12" cy="10" r="2.2"/></svg>',
          pick_and_place: '<svg viewBox="0 0 24 24"><rect x="3" y="5" width="6" height="6" rx="1"/><rect x="15" y="13" width="6" height="6" rx="1"/><path d="M9 8h3a5 5 0 0 1 5 5M14 10l3 3 3-3"/></svg>',
          approach: '<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="2"/><path d="M3 12h6M6 9l3 3-3 3M21 12h-6M18 9l-3 3 3 3"/></svg>',
          grasp: '<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="3"/><path d="M4 8v8l4 3M20 8v8l-4 3M4 8l4-3M20 8l-4-3"/></svg>',
          place: '<svg viewBox="0 0 24 24"><path d="M12 3v11M8 10l4 4 4-4"/><rect x="5" y="16" width="14" height="5" rx="1"/></svg>',
          transport: '<svg viewBox="0 0 24 24"><rect x="3" y="8" width="7" height="8" rx="1"/><path d="M11 12h9M17 9l3 3-3 3"/></svg>',
          reset: '<svg viewBox="0 0 24 24"><path d="M5.2 7.2A8 8 0 1 1 4 14"/><path d="M4 4v5h5"/></svg>',
          safety_reset: '<svg viewBox="0 0 24 24"><path d="M12 3 20 6v5c0 5.1-3.4 8.6-8 10-4.6-1.4-8-4.9-8-10V6l8-3Z"/><path d="M8.5 12a3.5 3.5 0 1 0 1-2.5M8 7.5v3h3"/></svg>'
        };
        return icons[product.name] || iconForKind(product.kind);
      }
      function memoryPriceFor(product) {
        var score = Array.from(product.name).reduce(function (total, char) { return total + char.charCodeAt(0); }, 0);
        var priceBands = product.kind === 'tool'
          ? [8.49, 8.99, 9.49, 9.99]
          : product.kind === 'skill'
            ? [4.49, 4.99, 5.49, 5.99, 6.49]
            : [0.99, 1.49, 1.99, 2.49, 2.99, 3.49];
        var original = priceBands[score % priceBands.length];
        var discount = product.kind === 'tool' ? 80 : 60;
        return {
          current: '$' + (original * (1 - discount / 100)).toFixed(2),
          original: '$' + original.toFixed(2),
          discount: discount + '% off'
        };
      }
      function memorySkillDetailMarkup(product) {
        var parameters = Object.keys(product.parameters || {}).join(' · ') || 'None';
        var dependencies = (product.dependsOn || []).join(' · ') || 'None';
        return '<div class="memory-product-detail" id="memory-detail-' + esc(product.name) + '" role="tooltip">' +
          '<strong>About this memory</strong><p>' + esc(product.description) + '</p>' +
          '<dl class="memory-detail-list">' +
          '<div><dt>Parameters</dt><dd>' + esc(parameters) + '</dd></div>' +
          '<div><dt>Depends on</dt><dd>' + esc(dependencies) + '</dd></div>' +
          '<div><dt>Returns</dt><dd>' + esc(product.returns) + '</dd></div>' +
          '<div><dt>Safety</dt><dd>' + esc(product.safety) + ' · ' + esc(memoryKindLabel(product.kind)) + '</dd></div>' +
          '</dl></div>';
      }
      function restoreMemoryLoadout() {
        var saved = null;
        try {
          saved = JSON.parse(localStorage.getItem(MEMORY_STORAGE_KEY) || 'null');
        } catch (_error) {
          saved = null;
        }
        memoryZones.forEach(function (zone) {
          var stored = saved && Array.isArray(saved[zone.id]) ? saved[zone.id] : null;
          var requested = stored || zone.defaults;
          state.memoryEquipped[zone.id] = requested.filter(function (name, index) {
            return zone.products.includes(name)
              && Boolean(memoryProductByName(name))
              && requested.indexOf(name) === index;
          }).slice(0, zone.slots);
        });
      }
      function persistMemoryLoadout() {
        try {
          localStorage.setItem(MEMORY_STORAGE_KEY, JSON.stringify(state.memoryEquipped));
        } catch (_error) {
          toast('Memory loadout could not be saved in this browser');
        }
      }
      function memoryEquippedTotal() {
        return memoryZones.reduce(function (total, zone) {
          return total + (state.memoryEquipped[zone.id] || []).length;
        }, 0);
      }
      function memoryCapabilityMarkup(product, equipped, slots) {
        var isEquipped = equipped.includes(product.name);
        var isFull = equipped.length >= slots;
        var disabled = isEquipped || isFull;
        var actionLabel = isEquipped ? 'Added' : isFull ? 'Full' : 'Add';
        var price = memoryPriceFor(product);
        return '<article class="memory-product-card" data-kind="' + esc(product.kind) +
          '" tabindex="0" aria-describedby="memory-detail-' + esc(product.name) + '">' +
          '<div class="memory-product-top"><span class="memory-product-icon" data-tone="' + memoryIconTone(product.name) +
          '" aria-hidden="true">' + memoryProductIcon(product) +
          '</span><span class="memory-product-kind">' + esc(memoryKindLabel(product.kind)) + '</span></div>' +
          '<h3>' + esc(product.title) + '</h3><p>' + esc(product.tagline) + '</p>' +
          memorySkillDetailMarkup(product) +
          '<div class="memory-product-footer"><span class="memory-price"><strong class="memory-price-current">' +
          esc(price.current) + '</strong><span class="memory-price-original">' + esc(price.original) +
          '</span><span class="memory-price-discount">' + esc(price.discount) + '</span></span>' +
          '<button class="memory-add" type="button" data-memory-equip="' + esc(product.name) + '"' +
          (disabled ? ' disabled' : '') + ' aria-label="' + esc(actionLabel + ' ' + product.title) + '">' +
          esc(actionLabel) + '</button></div></article>';
      }
      function renderMemoryLoadout() {
        var zone = memoryZoneById(state.memoryZone);
        var equipped = state.memoryEquipped[zone.id] || [];
        byId('memoryCatalogCount').textContent = String(catalog.length);
        byId('memoryEquippedCount').textContent = String(memoryEquippedTotal());
        byId('memoryZoneTitle').textContent = zone.label;
        byId('memoryZoneDescription').textContent = zone.description;
        byId('memoryZoneCode').textContent = zone.code;
        byId('memorySlotUsage').textContent = equipped.length + ' of ' + zone.slots;
        byId('memoryAvailableTitle').textContent = 'Explore ' + zone.code;
        byId('memoryZoneList').innerHTML = memoryZones.map(function (item) {
          var itemEquipped = (state.memoryEquipped[item.id] || []).length;
          return '<button class="memory-zone-button' + (item.id === zone.id ? ' active' : '') +
            '" type="button" data-memory-zone="' + esc(item.id) + '" aria-pressed="' + String(item.id === zone.id) + '">' +
            '<span class="memory-zone-icon" data-system="' + esc(item.id) + '" aria-hidden="true">' + memoryZoneIcon(item.id) + '</span>' +
            '<span><strong>' + esc(item.label) + '</strong><small>' + esc(item.subtitle) + '</small></span>' +
            '<span class="memory-zone-count">' + itemEquipped + ' of ' + item.slots + '</span></button>';
        }).join('');
        byId('memorySlotGrid').innerHTML = Array.from({ length: zone.slots }, function (_value, index) {
          var product = memoryProductByName(equipped[index]);
          if (!product) {
            return '<div class="memory-slot"><span class="memory-slot-index">Slot ' + String(index + 1).padStart(2, '0') +
              '</span><strong>Available</strong><small>Ready for a new memory</small></div>';
          }
          return '<div class="memory-slot filled" data-kind="' + esc(product.kind) + '">' +
            '<span class="memory-slot-index">Installed · ' + esc(memoryKindLabel(product.kind)) + '</span>' +
            '<strong>' + esc(product.title) + '</strong><small>' + esc(product.name) + ' · ' + esc(product.safety) + '</small>' +
            '<button class="memory-unequip" type="button" data-memory-remove="' + index + '" aria-label="Remove ' + esc(product.title) + '">×</button></div>';
        }).join('');
        var availableProducts = zone.products.map(memoryProductByName).filter(Boolean);
        byId('memoryAvailableList').innerHTML = availableProducts.length
          ? '<div class="memory-product-grid">' + availableProducts.map(function (product) {
              return memoryCapabilityMarkup(product, equipped, zone.slots);
            }).join('') + '</div>'
          : '<div class="memory-empty">No memories are available for this system yet.</div>';
      }
      function selectMemoryZone(zoneId) {
        var zone = memoryZoneById(zoneId);
        state.memoryZone = zone.id;
        renderMemoryLoadout();
        announce(zone.label + ' memory system selected');
      }
      function equipMemoryProduct(name) {
        var zone = memoryZoneById(state.memoryZone);
        var equipped = state.memoryEquipped[zone.id] || [];
        if (!zone.products.includes(name) || !memoryProductByName(name) || equipped.includes(name)) return;
        if (equipped.length >= zone.slots) {
          toast(zone.label + ' has no empty skill slots');
          return;
        }
        state.memoryEquipped[zone.id] = equipped.concat(name);
        persistMemoryLoadout();
        renderMemoryLoadout();
        toast(memoryProductByName(name).title + ' equipped to ' + zone.label);
      }
      function unequipMemoryProduct(index) {
        var zone = memoryZoneById(state.memoryZone);
        var equipped = (state.memoryEquipped[zone.id] || []).slice();
        var removed = equipped.splice(index, 1)[0];
        if (!removed) return;
        state.memoryEquipped[zone.id] = equipped;
        persistMemoryLoadout();
        renderMemoryLoadout();
        toast(memoryProductByName(removed).title + ' removed from ' + zone.label);
      }
      function renderCatalog() {
        var query = state.query.toLowerCase();
        var visible = catalog.filter(function (item) {
          var kindMatches = state.kind === 'all' || item.kind === state.kind;
          var safetyMatches = state.safety === 'all' || item.safety === state.safety;
          var dependencyMatches = !state.hasDependencies || item.dependsOn.length > 0;
          var haystack = [item.name, item.title, item.tagline, item.description, item.category].join(' ').toLowerCase();
          return kindMatches && safetyMatches && dependencyMatches && (!query || haystack.indexOf(query) >= 0);
        });
        byId('catalogCount').textContent = visible.length + (visible.length === 1 ? ' capability' : ' capabilities');
        byId('productGrid').innerHTML = visible.length ? visible.map(function (item) {
          return '<button class="product-card" role="listitem" type="button" data-kind="' + esc(item.kind) + '" data-product="' + esc(item.name) + '">' +
            '<span class="card-icon" aria-hidden="true">' + iconForKind(item.kind) + '</span>' +
            '<span class="kind">' + esc(kindLabel(item.kind)) + '</span>' +
            '<h3>' + esc(item.title) + '</h3><p>' + esc(item.tagline) + '</p>' +
            '<span class="card-bottom"><span class="safety">' + esc(item.safety) + ' safety level</span><span class="card-arrow" aria-hidden="true">→</span></span></button>';
        }).join('') : '<div class="empty">No matches. Try “perception” or “reset,” or change the filters.</div>';
        Array.from(document.querySelectorAll('[data-product]')).forEach(function (node) {
          node.addEventListener('click', function () { openProduct(node.dataset.product); });
        });
      }
      function openProduct(name) {
        var item = catalog.find(function (entry) { return entry.name === name; });
        if (!item) return;
        var params = Object.entries(item.parameters);
        byId('productDialogTitle').textContent = item.title + ' · ' + item.name;
        byId('productDialogBody').innerHTML =
          '<div class="detail-hero"><span class="eyebrow">' + esc(kindLabel(item.kind)) + ' · ' + esc(item.safety) + '</span>' +
          '<h2>' + esc(item.title) + '</h2><p>' + esc(item.description) + '</p></div>' +
          '<div class="detail-grid"><div class="detail-box"><h4>Purpose</h4><p>' + esc(item.tagline) + '</p></div>' +
          '<div class="detail-box"><h4>Returns</h4><p>' + esc(item.returns) + '</p></div>' +
          '<div class="detail-box"><h4>Dependencies</h4><p>' + esc(item.dependsOn.join(' · ') || 'None') + '</p></div>' +
          '<div class="detail-box"><h4>Website access</h4><p>' + (item.liveReadonly ? 'Live read-only status available' : 'Simulation and documentation only') + '</p></div></div>' +
          '<h3>Parameters</h3><div class="parameter-list">' + (params.length ? params.map(function (entry) {
            var key = entry[0], spec = entry[1];
            return '<div class="parameter"><code>' + esc(key) + '</code> <strong>' + esc(spec.type) + '</strong> ' +
              (spec.required ? '<small>required</small>' : '<small>optional</small>') + '<br><span>' + esc(spec.description) + '</span></div>';
          }).join('') : '<div class="parameter"><span>No parameters</span></div>') + '</div>' +
          '<div class="detail-box"><h4>Source of truth</h4><p><code>' + esc(item.source) + '</code></p></div>' +
          '<div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:18px"><button class="button" id="tryProduct" type="button">Try in Playground</button>' +
          '<button class="button secondary" id="copySource" type="button">Copy source path</button></div>';
        byId('productDialog').showModal();
        byId('tryProduct').addEventListener('click', function () {
          var instruction = item.example.instruction || (item.kind === 'tool' ? 'Show how ' + item.title + ' fits into a safe robot plan' : item.tagline);
          byId('instruction').value = instruction;
          byId('productDialog').close();
          location.hash = 'playground';
          setTimeout(function () { byId('instruction').focus(); }, 80);
        });
        byId('copySource').addEventListener('click', function () { copyText(item.source, 'Source path copied'); });
      }
      byId('catalogSearch').addEventListener('input', function (event) {
        state.query = event.target.value.trim();
        renderCatalog();
      });
      byId('kindFilters').addEventListener('click', function (event) {
        var button = event.target.closest('[data-kind]');
        if (!button) return;
        state.kind = button.dataset.kind;
        Array.from(byId('kindFilters').querySelectorAll('[data-kind]')).forEach(function (node) {
          node.setAttribute('aria-pressed', String(node === button));
        });
        renderCatalog();
      });
      byId('dependencyFilter').addEventListener('click', function (event) {
        state.hasDependencies = !state.hasDependencies;
        event.currentTarget.setAttribute('aria-pressed', String(state.hasDependencies));
        renderCatalog();
      });
      byId('safetyFilter').addEventListener('change', function (event) {
        state.safety = event.target.value;
        renderCatalog();
      });
      byId('productGrid').addEventListener('keydown', function (event) {
        if (!['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(event.key)) return;
        var cards = Array.from(byId('productGrid').querySelectorAll('.product-card'));
        var index = cards.indexOf(document.activeElement);
        if (index < 0) return;
        var columns = window.innerWidth <= 640 ? 1 : window.innerWidth <= 980 ? 2 : 3;
        var delta = event.key === 'ArrowLeft' ? -1 : event.key === 'ArrowRight' ? 1 : event.key === 'ArrowUp' ? -columns : columns;
        var next = cards[index + delta];
        if (next) { event.preventDefault(); next.focus(); }
      });

      Array.from(document.querySelectorAll('.scenario')).forEach(function (button) {
        button.addEventListener('click', function () {
          byId('instruction').value = button.textContent;
          byId('instruction').focus();
        });
      });
      function productionAgentReady(snapshot) {
        var agentData = snapshot && snapshot.agent && snapshot.agent.data ? snapshot.agent.data : {};
        return Boolean(snapshot && snapshot.agent && snapshot.agent.connected && agentData.ready === true);
      }
      function activeHardwareTask(snapshot) {
        var agentData = snapshot && snapshot.agent && snapshot.agent.data ? snapshot.agent.data : {};
        return Boolean(agentData.activeTask || agentData.activeTaskId);
      }
      function resetWorkspacePlanView() {
        state.plan = null;
        byId('pipeline').innerHTML = '<div class="pipeline-empty">Enter a goal to see the selected Skills and their Tool dependencies.</div>';
        byId('timeline').innerHTML = '<div class="pipeline-empty">Events will explain how the Plan was built.</div>';
        byId('rawOutput').textContent = 'Waiting for a Plan…';
        byId('replayPlan').disabled = true;
      }
      function updateWorkspaceModeUI() {
        var hardwareMode = state.workspaceMode === 'hardware';
        var snapshot = state.opsSnapshot;
        var policy = snapshot && snapshot.policy ? snapshot.policy : state.hardwarePolicy || {};
        var stack = snapshot && snapshot.stack ? snapshot.stack : {};
        var ready = productionAgentReady(snapshot);
        var taskActive = activeHardwareTask(snapshot);
        var startupAvailable = Boolean(snapshot && snapshot.operations && snapshot.operations.startupAvailable);
        var startupButton = byId('workspaceStartHardware');
        byId('playground').dataset.workspaceMode = state.workspaceMode;
        byId('hardwareModeBanner').hidden = !hardwareMode;
        byId('planModePill').textContent = hardwareMode ? 'REAL ROBOT' : 'SIMULATION';
        byId('planModePill').classList.toggle('hardware', hardwareMode);
        byId('planEventsTitle').textContent = hardwareMode ? 'Hardware Plan events' : 'Simulation events';
        byId('planEventsPill').textContent = hardwareMode ? 'APPROVAL REQUIRED' : 'NO MOTION';
        byId('planEventsPill').classList.toggle('hardware', hardwareMode);
        byId('planConnectionBadge').classList.toggle('hardware', hardwareMode);
        byId('reviewHardwarePlan').hidden = !hardwareMode || !state.hardwarePlan;
        Array.from(byId('workspaceModeSwitch').querySelectorAll('[data-workspace-mode]')).forEach(function (button) {
          button.setAttribute('aria-pressed', String(button.dataset.workspaceMode === state.workspaceMode));
        });
        if (!hardwareMode) {
          byId('playgroundLead').textContent = 'Simulation never connects to hardware. Switch to Real robot only from the local operator console when you are beside the robot.';
          byId('planConnectionBadge').textContent = 'No hardware connection';
          byId('planHint').textContent = state.simulationPlan
            ? 'Complete: this simulated Plan cannot execute.'
            : 'Describe the result you want. Do not enter joint angles, force, velocity, CAN, or shell commands.';
          byId('planButton').disabled = false;
          byId('planButton').textContent = 'Generate simulated Plan';
          return;
        }
        byId('playgroundLead').textContent = 'Real robot mode uses the supervised Python Agent. Start the robot stack, create a high-level Plan, then complete the independent on-site approval before motion.';
        byId('planButton').textContent = 'Create hardware Plan';
        byId('planConnectionBadge').textContent = ready
          ? taskActive ? 'Robot busy · Task active' : 'Supervised Agent ready'
          : 'Hardware not ready';
        byId('planButton').disabled = !policy.executeEnabled || !ready || taskActive;
        if (!policy.executeEnabled) {
          byId('workspaceHardwareTitle').textContent = 'Real robot unavailable in this preview';
          byId('workspaceHardwareDetail').textContent = 'Hardware controls are disabled on public and simulation-only pages. Use the local production console beside the robot.';
          startupButton.textContent = 'Local console only';
          startupButton.disabled = true;
          byId('planHint').textContent = 'This page is locked to simulation; no hardware request can be sent.';
        } else if (stack.state === 'faulted' && !state.startupInFlight && !state.recoveryInFlight) {
          byId('workspaceHardwareTitle').textContent = stack.recoverable ? 'Hardware startup can be retried' : 'Hardware execution is locked';
          byId('workspaceHardwareDetail').textContent = stack.detail || 'Inspect the robot and physical emergency stop before retrying.';
          startupButton.textContent = stack.recoverable ? 'Retry hardware' : 'Recovery unavailable';
          startupButton.disabled = !stack.recoverable;
          byId('planHint').textContent = stack.recoverable
            ? 'Retry reinitializes only the hardware plane; this page remains online.'
            : 'The hardware stack must be inspected before planning.';
        } else if (stack.state === 'starting' || stack.state === 'recovering' || stack.state === 'stopping' || state.startupInFlight || state.recoveryInFlight || state.stopInFlight) {
          var workspaceStopping = stack.state === 'stopping' || state.stopInFlight;
          var workspaceRecovering = stack.state === 'recovering' || state.recoveryInFlight;
          byId('workspaceHardwareTitle').textContent = workspaceStopping
            ? 'Stopping robot stack…'
            : workspaceRecovering ? 'Recovering robot stack…' : 'Starting robot stack…';
          byId('workspaceHardwareDetail').textContent = workspaceStopping
            ? 'The Python Agent and hardware control stack are shutting down; this page remains online.'
            : 'The page is monitoring the control stack and supervised Python Agent.';
          startupButton.textContent = workspaceStopping
            ? 'Stopping…'
            : workspaceRecovering ? 'Recovering…' : 'Starting…';
          startupButton.disabled = true;
          byId('planHint').textContent = workspaceStopping
            ? 'Wait for the hardware plane to report offline.'
            : 'Wait for the supervised Agent to report ready.';
        } else if (ready) {
          byId('workspaceHardwareTitle').textContent = taskActive ? 'Robot is executing a Task' : 'Robot stack ready';
          byId('workspaceHardwareDetail').textContent = taskActive
            ? 'Wait for the active Task to finish before creating another hardware Plan.'
            : 'Create a high-level Plan below. Planning does not move the robot; execution still requires the on-site checklist and exact confirmation phrase.';
          startupButton.textContent = taskActive ? 'Task active' : 'Ready';
          startupButton.disabled = true;
          byId('planHint').textContent = taskActive
            ? 'A hardware Task is active.'
            : state.hardwarePlan
              ? 'Hardware Plan ready. Review every Skill, then complete the on-site approval before execution.'
              : 'Planning is enabled. The resulting Plan must be reviewed and approved before motion.';
        } else if (!startupAvailable) {
          byId('workspaceHardwareTitle').textContent = 'Robot startup unavailable';
          byId('workspaceHardwareDetail').textContent = 'This local server does not have a reviewed startup launcher configured.';
          startupButton.textContent = 'Unavailable';
          startupButton.disabled = true;
          byId('planHint').textContent = 'Start the production console with the supervised launcher.';
        } else {
          byId('workspaceHardwareTitle').textContent = 'Robot stack is offline';
          byId('workspaceHardwareDetail').textContent = 'Verify the workspace and physical emergency stop, then initialize the control stack and supervised Python Agent.';
          startupButton.textContent = 'Start real robot';
          startupButton.disabled = false;
          byId('planHint').textContent = 'Start the robot stack before creating a hardware Plan.';
        }
      }
      function setWorkspaceMode(mode) {
        if (mode !== 'simulation' && mode !== 'hardware') return;
        state.workspaceMode = mode;
        var savedPlan = mode === 'hardware' ? state.hardwareWorkspacePlan : state.simulationPlan;
        if (savedPlan) renderPlan(savedPlan);
        else resetWorkspacePlanView();
        updateWorkspaceModeUI();
        announce(mode === 'hardware' ? 'Real robot planning mode selected' : 'Simulation planning mode selected');
      }
      byId('workspaceModeSwitch').addEventListener('click', function (event) {
        var button = event.target.closest('[data-workspace-mode]');
        if (button) setWorkspaceMode(button.dataset.workspaceMode);
      });
      function browserFallbackPlan(instruction) {
        var isReset = /reset|return.+arm|safe pose/i.test(instruction);
        var isObserve = /see|find|locate|detect|inspect|observe/i.test(instruction) && !/pick|place|move|grasp/i.test(instruction);
        var isTidy = /tidy|clean|organize|clear.+table/i.test(instruction);
        var steps = isReset ? [{ ref: 'step-01', skill: 'safety_reset', args: { arms: 'both' }, constraints: { planOnly: true }, tools: ['place', 'reset'] }]
          : isObserve ? [{ ref: 'step-01', skill: 'perceive_scene', args: { instruction: instruction }, constraints: { readOnly: true }, tools: ['ground', 'estimate_grasp_poses', 'localize'] }]
          : [{ ref: 'step-01', skill: 'perceive_scene', args: { instruction: instruction }, constraints: { readOnly: true }, tools: ['ground', 'estimate_grasp_poses', 'localize'] },
             { ref: 'step-02', skill: isTidy ? 'tidy_table' : 'pick_and_place', args: { instruction: instruction, plan_only: true }, constraints: { planOnly: true }, tools: ['approach', 'grasp', 'transport', 'place', 'reset'] },
             { ref: 'step-03', skill: 'safety_reset', args: { arms: 'both' }, constraints: { execution: false }, tools: ['place', 'reset'] }];
        var events = [{ offsetMs: 0, type: 'plan.created', status: 'ready', message: 'The offline browser simulator created a Plan.' }].concat(steps.map(function (step, index) {
          return { offsetMs: 400 + index * 600, type: 'skill.preview', status: 'simulated', message: 'Previewed ' + step.skill, skill: step.skill };
        })).concat([{ offsetMs: 400 + steps.length * 600, type: 'plan.completed', status: 'dry-run', message: 'Simulation complete; no hardware connection was made.' }]);
        return { mode: 'simulation', provider: 'browser-fallback', planId: 'sim_offline_' + Date.now(), instruction: instruction, createdAt: new Date().toISOString(), constraints: { hardwareAccess: false, approvalRequiredForFutureExecution: true }, steps: steps, events: events, ndjson: [JSON.stringify({ id: 'offline-1', method: 'plan', params: { instruction: instruction } })] };
      }
      function hardwarePlanForWorkspace(data) {
        var sourceSteps = data.plan && Array.isArray(data.plan.steps) ? data.plan.steps : [];
        var steps = sourceSteps.map(function (step, index) {
          var product = catalog.find(function (item) { return item.name === step.skill; });
          return {
            ref: step.ref || 'step-' + String(index + 1).padStart(2, '0'),
            skill: step.skill,
            args: step.args || {},
            constraints: { approvalRequired: true, hardwareMotion: false },
            tools: Array.isArray(step.tools) ? step.tools : product ? product.dependsOn : []
          };
        });
        var events = [{
          offsetMs: 0,
          type: 'hardware.plan.created',
          status: 'awaiting-approval',
          message: 'The supervised Python Agent created a high-level Plan. No motion has started.'
        }].concat(steps.map(function (step, index) {
          return {
            offsetMs: 350 + index * 450,
            type: 'skill.validated',
            status: 'review-required',
            message: 'Validated ' + step.skill + ' against the website Skill allowlist.',
            skill: step.skill
          };
        })).concat([{
          offsetMs: 350 + steps.length * 450,
          type: 'operator.approval.required',
          status: 'blocked',
          message: 'Complete the on-site checklist and confirmation before creating a hardware Task.'
        }]);
        return {
          mode: data.mode,
          provider: 'python-agent',
          planId: data.planId,
          instruction: data.instruction,
          createdAt: new Date().toISOString(),
          constraints: { hardwareAccess: true, approvalRequired: true, motionStarted: false },
          steps: steps,
          events: events,
          ndjson: [JSON.stringify({ method: 'hardware.plan', planId: data.planId, status: 'awaiting-approval' })],
          rawPayload: data
        };
      }
      function renderPlan(plan) {
        state.plan = plan;
        byId('pipeline').innerHTML = plan.steps.map(function (step, index) {
          var arrow = index < plan.steps.length - 1 ? '<span class="pipeline-arrow" aria-hidden="true">→</span>' : '';
          return '<article class="pipeline-step" style="animation-delay:' + (index * 80) + 'ms"><span class="step-index">STEP ' + String(index + 1).padStart(2, '0') + '</span>' +
            '<strong>' + esc(step.skill) + '</strong><div>' + step.tools.map(function (tool) { return '<span class="tool-chip">' + esc(tool) + '</span>'; }).join('') + '</div>' +
            '<div style="margin-top:10px">' + Object.entries(step.constraints).map(function (entry) { return '<span class="tool-chip">constraint · ' + esc(entry[0]) + '=' + esc(entry[1]) + '</span>'; }).join('') + '</div></article>' + arrow;
        }).join('');
        byId('timeline').innerHTML = plan.events.map(function (event, index) {
          return '<div class="event" data-event="' + index + '"><strong>' + esc(event.type) + '</strong><br>' + esc(event.message) + '</div>';
        }).join('');
        byId('replayPlan').disabled = false;
        renderRaw();
        replayTimeline();
      }
      function renderRaw() {
        if (!state.plan) return;
        byId('rawOutput').textContent = state.rawMode === 'ndjson'
          ? state.plan.ndjson.join('\n')
          : pretty(state.plan.rawPayload || state.plan);
        byId('jsonTab').setAttribute('aria-selected', String(state.rawMode === 'json'));
        byId('ndjsonTab').setAttribute('aria-selected', String(state.rawMode === 'ndjson'));
      }
      function replayTimeline() {
        state.replayTimers.forEach(clearTimeout);
        state.replayTimers = [];
        var nodes = Array.from(byId('timeline').querySelectorAll('.event'));
        nodes.forEach(function (node) { node.classList.remove('active'); });
        if (!state.plan) return;
        state.plan.events.forEach(function (event, index) {
          var timer = setTimeout(function () { if (nodes[index]) nodes[index].classList.add('active'); }, Math.min(event.offsetMs, 2400));
          state.replayTimers.push(timer);
        });
      }
      byId('planForm').addEventListener('submit', async function (event) {
        event.preventDefault();
        var instruction = byId('instruction').value.trim();
        if (!instruction) { byId('planHint').textContent = 'Enter a task goal first.'; byId('instruction').focus(); return; }
        var hardwareMode = state.workspaceMode === 'hardware';
        if (hardwareMode && (!productionAgentReady(state.opsSnapshot) || !(state.hardwarePolicy && state.hardwarePolicy.executeEnabled))) {
          updateWorkspaceModeUI();
          toast('Start the supervised robot stack before hardware planning');
          return;
        }
        var button = byId('planButton');
        button.disabled = true; button.textContent = hardwareMode ? 'Agent is planning…' : 'Building Skill plan…';
        byId('planHint').textContent = hardwareMode
          ? 'Requesting a high-level Plan from the supervised Python Agent. No motion starts during planning.'
          : 'Generating a Plan in the local no-motion simulator.';
        try {
          var result = await requestJson(hardwareMode ? '/api/hardware/plan' : '/api/simulate/plan', {
            method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ instruction: instruction })
          });
          if (hardwareMode) {
            byId('hardwareInstruction').value = instruction;
            clearHardwareApproval();
            renderHardwarePlan(result.data);
            byId('planHint').textContent = 'Hardware Plan ready. Review every Skill, then complete the on-site approval before execution.';
            toast('Hardware Plan created; no motion has started');
            announce('Hardware Plan created and awaiting operator approval');
          } else {
            state.simulationPlan = result.data;
            renderPlan(result.data);
            byId('planHint').textContent = 'Complete: this simulated Plan cannot execute.';
            announce('Simulated Plan generated with ' + result.data.steps.length + ' Skill steps');
          }
        } catch (error) {
          if (hardwareMode) {
            byId('planHint').textContent = 'Hardware Plan rejected: ' + error.message;
            toast('The hardware Plan was rejected');
          } else if (/joint|low-level|must|cannot exceed|rejected/i.test(error.message)) {
            byId('planHint').textContent = error.message;
            toast('The goal was rejected by the safety policy');
          } else {
            state.simulationPlan = browserFallbackPlan(instruction);
            renderPlan(state.simulationPlan);
            byId('planHint').textContent = 'The server is unavailable; using the offline browser simulator.';
            toast('Switched to offline simulation');
          }
        } finally {
          button.disabled = hardwareMode
            ? !(state.hardwarePolicy && state.hardwarePolicy.executeEnabled)
              || !productionAgentReady(state.opsSnapshot)
              || activeHardwareTask(state.opsSnapshot)
            : false;
          button.textContent = hardwareMode ? 'Create hardware Plan' : 'Generate simulated Plan';
        }
      });
      byId('reviewHardwarePlan').addEventListener('click', function () {
        location.hash = 'hardware';
        setTimeout(function () { byId('hardwareConfirmation').focus(); }, 80);
      });
      byId('jsonTab').addEventListener('click', function () { state.rawMode = 'json'; renderRaw(); });
      byId('ndjsonTab').addEventListener('click', function () { state.rawMode = 'ndjson'; renderRaw(); });
      byId('copyRaw').addEventListener('click', function () { copyText(byId('rawOutput').textContent, 'Raw plan data copied'); });
      byId('replayPlan').addEventListener('click', replayTimeline);

      function renderArchitecture() {
        var nodes = content.architecture.nodes;
        byId('architectureFlow').innerHTML = nodes.map(function (node, index) {
          var edge = content.architecture.edges[index];
          return '<button class="arch-node" type="button" data-arch="' + esc(node.id) + '" aria-pressed="' + String(index === 0) + '">' +
            '<small>' + esc(node.eyebrow) + '</small><strong>' + esc(node.title) + '</strong></button>' +
            (edge ? '<span class="arch-arrow" title="' + esc(edge.protocol) + '" aria-label="' + esc(edge.label) + '">→</span>' : '');
        }).join('');
        Array.from(document.querySelectorAll('[data-arch]')).forEach(function (button) {
          button.addEventListener('click', function () { selectArchitecture(button.dataset.arch); });
        });
        selectArchitecture(nodes[0].id);
      }
      function selectArchitecture(id) {
        var node = content.architecture.nodes.find(function (entry) { return entry.id === id; });
        if (!node) return;
        Array.from(document.querySelectorAll('[data-arch]')).forEach(function (button) {
          button.setAttribute('aria-pressed', String(button.dataset.arch === id));
        });
        byId('architectureDetail').innerHTML = '<div class="arch-summary"><p class="eyebrow">' + esc(node.eyebrow) + '</p><h3>' + esc(node.title) + '</h3>' +
          '<p>' + esc(node.summary) + '</p><span class="process-pill">' + esc(node.process) + '</span></div>' +
          '<div><div class="owns-grid"><div><h4>Responsible for</h4><ul>' + node.owns.map(function (item) { return '<li>' + esc(item) + '</li>'; }).join('') +
          '</ul></div><div><h4>Explicitly refuses</h4><ul>' + node.refuses.map(function (item) { return '<li>' + esc(item) + '</li>'; }).join('') +
          '</ul></div></div><button class="source-button" type="button" data-source="' + esc(node.source) + '" style="margin-top:12px">' + esc(node.source) + '</button></div>';
        byId('architectureDetail').querySelector('[data-source]').addEventListener('click', function () { copyText(node.source, 'Source path copied'); });
      }

      function renderProtocolEditor() {
        var example = content.protocol[state.protocol];
        byId('protocolEditor').value = JSON.stringify(example.request, null, 2);
        byId('protocolOutput').textContent = 'Waiting for a request…';
      }
      byId('protocolTabs').addEventListener('click', function (event) {
        var button = event.target.closest('[data-protocol]');
        if (!button) return;
        state.protocol = button.dataset.protocol;
        Array.from(byId('protocolTabs').querySelectorAll('[data-protocol]')).forEach(function (node) {
          node.setAttribute('aria-selected', String(node === button));
        });
        renderProtocolEditor();
      });
      byId('runProtocol').addEventListener('click', function () {
        var output = byId('protocolOutput');
        try {
          var request = JSON.parse(byId('protocolEditor').value);
          if (!request || typeof request !== 'object' || Array.isArray(request)) throw new Error('The envelope must be a JSON object.');
          if (typeof request.id !== 'string' || !request.id) throw new Error('id must be a non-empty string.');
          if (!['health', 'manifest', 'plan'].includes(request.method)) {
            output.textContent = pretty({ id: request.id, ok: false, error: { code: 'METHOD_NOT_ALLOWED', message: 'The plan-only protocol rejects ' + String(request.method) } });
            toast('The protocol sandbox rejected this method');
            return;
          }
          if (request.method === 'plan' && (!request.params || typeof request.params.instruction !== 'string' || !request.params.instruction.trim())) {
            throw new Error('plan.params.instruction must be a non-empty string.');
          }
          var response = JSON.parse(JSON.stringify(content.protocol[request.method].response));
          response.id = request.id;
          if (request.method === 'plan') response.data.instruction = request.params.instruction;
          output.textContent = pretty(response);
          announce(request.method + ' protocol simulation completed');
        } catch (error) {
          output.textContent = pretty({ ok: false, error: { code: 'INVALID_ENVELOPE', message: error.message } });
        }
      });

      function renderSafety() {
        byId('safetyGrid').innerHTML = content.safety.map(function (scenario, index) {
          return '<article class="scenario-card" data-safety="' + esc(scenario.id) + '"><span class="scenario-number">CASE ' + String(index + 1).padStart(2, '0') + '</span>' +
            '<p>' + esc(scenario.prompt) + '</p><div class="decision-row"><button class="decision" type="button" data-answer="allow">Allow</button>' +
            '<button class="decision" type="button" data-answer="deny">Deny</button></div><div class="scenario-explain" aria-live="polite"></div></article>';
        }).join('');
      }
      byId('safetyGrid').addEventListener('click', function (event) {
        var button = event.target.closest('[data-answer]');
        if (!button) return;
        var card = button.closest('[data-safety]');
        if (card.classList.contains('answered')) return;
        var scenario = content.safety.find(function (entry) { return entry.id === card.dataset.safety; });
        var correct = button.dataset.answer === scenario.expected;
        card.classList.add('answered', correct ? 'correct' : 'wrong');
        Array.from(card.querySelectorAll('button')).forEach(function (node) { node.disabled = true; });
        card.querySelector('.scenario-explain').textContent = (correct ? 'Correct: ' : 'Review this: ') + scenario.explanation;
        state.safetyAnswered += 1;
        byId('safetyScore').textContent = state.safetyAnswered + ' / ' + content.safety.length + ' answered';
        announce(correct ? 'Correct decision' : 'Incorrect decision');
      });

      function renderExtensions() {
        byId('extensionSteps').innerHTML = content.extension.map(function (step) {
          return '<article class="extension-step"><span class="number">' + esc(step.number) + '</span><h3>' + esc(step.title) + '</h3><p>' + esc(step.copy) +
            '</p><button class="source-button" type="button" data-extension-source="' + esc(step.source) + '">' + esc(step.source) + '</button></article>';
        }).join('');
        Array.from(document.querySelectorAll('[data-extension-source]')).forEach(function (button) {
          button.addEventListener('click', function () { copyText(button.dataset.extensionSource, 'Source path copied'); });
        });
      }

      var tourSlides = [
        { title: 'Describe the result', copy: 'Say “Place the blue cup beside the yellow plate.” You do not need joint angles, velocities, or control code.', visual: ['You', '→', 'Goal'] },
        { title: 'Review the Plan', copy: 'Harness converts the goal into inspectable steps such as perceive_scene, pick_and_place, and safety_reset.', visual: ['Plan', '→', 'Skill'] },
        { title: 'Inspect Tool dependencies', copy: 'Perception expands to ground, estimate, and localize. Manipulation expands to approach, grasp, transport, and place.', visual: ['Skill', '→', 'Tool'] },
        { title: 'Approve at the hardware boundary', copy: 'Real motion requires launcher-supervised readiness, both server gates, the Skill allowlist, on-site checks, and the exact phrase.', visual: ['Review', '✓', 'Agent'] }
      ];
      function renderTour() {
        var slide = tourSlides[state.tourIndex];
        byId('tourProgress').innerHTML = tourSlides.map(function (_, index) { return '<span class="' + (index <= state.tourIndex ? 'active' : '') + '"></span>'; }).join('');
        byId('tourCard').innerHTML = '<div class="tour-visual">' + slide.visual.map(function (item) { return '<span>' + esc(item) + '</span>'; }).join('') + '</div><p class="eyebrow">STEP ' + (state.tourIndex + 1) + ' / ' + tourSlides.length + '</p><h2 style="font-size:40px">' + esc(slide.title) + '</h2><p>' + esc(slide.copy) + '</p>';
        byId('tourBack').disabled = state.tourIndex === 0;
        byId('tourNext').textContent = state.tourIndex === tourSlides.length - 1 ? 'Try the simulator' : 'Next';
      }
      byId('startTour').addEventListener('click', function () { state.tourIndex = 0; renderTour(); byId('tourDialog').showModal(); });
      byId('tourBack').addEventListener('click', function () { if (state.tourIndex > 0) { state.tourIndex -= 1; renderTour(); } });
      byId('tourNext').addEventListener('click', function () {
        if (state.tourIndex < tourSlides.length - 1) { state.tourIndex += 1; renderTour(); return; }
        byId('tourDialog').close(); location.hash = 'playground'; setTimeout(function () { byId('instruction').focus(); }, 80);
      });

      Array.from(document.querySelectorAll('[data-close]')).forEach(function (button) {
        button.addEventListener('click', function () { byId(button.dataset.close).close(); });
      });
      Array.from(document.querySelectorAll('dialog')).forEach(function (dialog) {
        dialog.addEventListener('click', function (event) {
          var rect = dialog.getBoundingClientRect();
          if (event.clientX < rect.left || event.clientX > rect.right || event.clientY < rect.top || event.clientY > rect.bottom) dialog.close();
        });
      });
      async function copyText(value, message) {
        try { await navigator.clipboard.writeText(value); toast(message); }
        catch (_) { toast('Clipboard access is unavailable; copy the text manually'); }
      }

      function setHardwarePolicyUI(policy) {
        state.hardwarePolicy = policy;
        var enabled = policy.executeEnabled === true;
        byId('stageOneState').textContent = policy.readonlyEnabled ? 'Enabled · Read-only' : 'Disabled · Simulation';
        byId('stageTwoState').textContent = enabled ? 'Enabled · Approval-gated' : 'Disabled · Locked';
        byId('hardwarePolicyState').textContent = enabled
          ? 'Approval-gated execution · Python Agent is the sole hardware owner'
          : 'Locked · Read-only and execution gates are required';
        byId('hardwareInstruction').disabled = !enabled;
        byId('hardwarePlanButton').disabled = !enabled;
        byId('refreshHarnessApprovals').disabled = !enabled;
        byId('hardwareLock').textContent = enabled
          ? 'Both server gates are open. Creating a Plan does not move the robot; an approved Task can.'
          : 'Execution is locked by default. An operator must enable both gates in the supervised launcher environment.';
        if (enabled && !state.hardwareApprovalTimer) {
          state.hardwareApprovalTimer = setTimeout(refreshHarnessApprovals, 0);
        }
        if (!enabled) {
          clearTimeout(state.hardwareApprovalTimer);
          state.hardwareApprovalTimer = null;
        }
      }
      function clearHardwareApproval() {
        state.hardwarePlan = null;
        state.hardwareWorkspacePlan = null;
        state.hardwareApprovalRequest = null;
        byId('hardwarePlanPreview').hidden = true;
        byId('reviewHardwarePlan').hidden = true;
        Array.from(document.querySelectorAll('[data-approval-check]')).forEach(function (node) {
          node.checked = false;
          node.disabled = true;
        });
        byId('hardwareConfirmation').value = '';
        byId('hardwareConfirmation').disabled = true;
        byId('hardwareExecuteButton').disabled = true;
      }
      function updateHardwareApprovalButton() {
        var checks = Array.from(document.querySelectorAll('[data-approval-check]'));
        var allChecked = checks.every(function (node) { return node.checked; });
        byId('hardwareExecuteButton').disabled = !state.hardwarePlan
          || !allChecked
          || byId('hardwareConfirmation').value !== 'ENABLE_DUAL_V2_2_HARDWARE';
      }
      function renderHardwarePlan(data) {
        state.hardwarePlan = data;
        state.hardwareWorkspacePlan = hardwarePlanForWorkspace(data);
        var steps = data.plan && Array.isArray(data.plan.steps) ? data.plan.steps : [];
        byId('hardwarePlanSteps').innerHTML = steps.map(function (step, index) {
          return '<div class="approval-step"><strong>STEP ' + String(index + 1).padStart(2, '0') + ' · ' + esc(step.skill) + '</strong>' +
            esc(step.summary || step.description || pretty(step.args || {})) + '</div>';
        }).join('');
        byId('hardwarePlanExpiry').textContent = 'Plan ID: ' + data.planId + (data.expiresAt
          ? ' · Expires: ' + data.expiresAt
          : ' · Approval window: ' + data.expiresInSeconds + ' seconds');
        byId('hardwarePlanPreview').hidden = false;
        Array.from(document.querySelectorAll('[data-approval-check]')).forEach(function (node) {
          node.disabled = false;
          node.checked = false;
        });
        byId('hardwareConfirmation').disabled = false;
        byId('hardwareConfirmation').value = '';
        byId('reviewHardwarePlan').hidden = state.workspaceMode !== 'hardware';
        if (state.workspaceMode === 'hardware') renderPlan(state.hardwareWorkspacePlan);
        updateHardwareApprovalButton();
      }
      function hardwareTaskIsTerminal(data) {
        var value = String(data.status || data.state || '').toLowerCase();
        return ['succeeded', 'success', 'completed', 'failed', 'error', 'cancelled', 'canceled'].includes(value);
      }
      function renderHardwareEvents() {
        var eventsNode = byId('hardwareTaskEvents');
        eventsNode.innerHTML = state.hardwareEvents.length
          ? state.hardwareEvents.map(function (item) {
              return '<div class="task-event"><strong>' + esc(item.type || item.event || 'agent.event') + '</strong><br>' +
                esc(item.message || item.status || pretty(item)) + '</div>';
            }).join('')
          : 'No new Python Agent events.';
        requestAnimationFrame(function () {
          eventsNode.scrollTop = eventsNode.scrollHeight;
        });
      }
      async function refreshHardwareTask() {
        if (!state.hardwareTaskId) return;
        try {
          var query = '?taskId=' + encodeURIComponent(state.hardwareTaskId);
          var results = await Promise.all([
            requestJson('/api/hardware/task/status' + query),
            requestJson('/api/hardware/task/events' + query + '&cursor=' + encodeURIComponent(state.hardwareCursor))
          ]);
          var status = results[0].data;
          var eventData = results[1].data;
          byId('hardwareTaskStatus').textContent = pretty(status);
          if (Array.isArray(eventData.events)) {
            state.hardwareEvents = state.hardwareEvents.concat(eventData.events);
          }
          if (eventData.nextCursor !== undefined) state.hardwareCursor = eventData.nextCursor;
          renderHardwareEvents();
          if (hardwareTaskIsTerminal(status)) {
            clearTimeout(state.hardwarePollTimer);
            byId('cancelHardwareTask').disabled = true;
            toast('Hardware Task finished: ' + String(status.status || status.state));
          } else {
            clearTimeout(state.hardwarePollTimer);
            state.hardwarePollTimer = setTimeout(refreshHardwareTask, 1200);
          }
        } catch (error) {
          clearTimeout(state.hardwarePollTimer);
          byId('hardwareTaskStatus').textContent = 'Could not read Task: ' + error.message;
        }
      }
      byId('usePlaygroundGoal').addEventListener('click', function () {
        byId('hardwareInstruction').value = byId('instruction').value.trim();
        byId('hardwareInstruction').focus();
      });
      byId('hardwarePlanForm').addEventListener('submit', async function (event) {
        event.preventDefault();
        var button = byId('hardwarePlanButton');
        var instruction = byId('hardwareInstruction').value.trim();
        if (!instruction) { toast('Enter a semantic task goal first'); return; }
        clearHardwareApproval();
        button.disabled = true;
        button.textContent = 'Agent is planning…';
        try {
          var result = await requestJson('/api/hardware/plan', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({ instruction: instruction })
          });
          state.hardwareApprovalRequest = null;
          renderHardwarePlan(result.data);
          toast('Hardware Plan created; review it on site');
          announce('Hardware Plan created but not executed');
        } catch (error) {
          byId('hardwareLock').textContent = 'Plan rejected: ' + error.message;
          toast('The hardware Plan was rejected');
        } finally {
          button.disabled = !(state.hardwarePolicy && state.hardwarePolicy.executeEnabled);
          button.textContent = 'Create hardware Plan';
        }
      });
      Array.from(document.querySelectorAll('[data-approval-check]')).forEach(function (node) {
        node.addEventListener('change', updateHardwareApprovalButton);
      });
      byId('hardwareConfirmation').addEventListener('input', updateHardwareApprovalButton);
      byId('hardwareExecuteForm').addEventListener('submit', async function (event) {
        event.preventDefault();
        if (!state.hardwarePlan) return;
        var button = byId('hardwareExecuteButton');
        button.disabled = true;
        button.textContent = 'Submitting approval…';
        var checks = Array.from(document.querySelectorAll('[data-approval-check]'));
        try {
          var harnessRequest = state.hardwareApprovalRequest;
          var result = await requestJson(
            harnessRequest ? '/api/hardware/approvals/approve' : '/api/hardware/execute',
            {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({
              planId: harnessRequest ? undefined : state.hardwarePlan.planId,
              requestId: harnessRequest ? harnessRequest.requestId : undefined,
              confirmation: byId('hardwareConfirmation').value,
              checklist: {
                operatorPresent: checks[0].checked,
                workspaceClear: checks[1].checked,
                emergencyStopReady: checks[2].checked,
                robotInspected: checks[3].checked
              }
            })
          });
          state.hardwareTaskId = result.data.taskId;
          state.hardwareCursor = 0;
          state.hardwareEvents = [];
          state.hardwarePlan = null;
          state.hardwareWorkspacePlan = null;
          state.hardwareApprovalRequest = null;
          byId('reviewHardwarePlan').hidden = true;
          byId('hardwareTaskStatus').textContent = pretty(result.data);
          byId('hardwareTaskEvents').textContent = 'Waiting for Python Agent events…';
          byId('refreshHardwareTask').disabled = false;
          byId('cancelHardwareTask').disabled = false;
          Array.from(document.querySelectorAll('[data-approval-check]')).forEach(function (node) { node.disabled = true; });
          byId('hardwareConfirmation').disabled = true;
          announce('Hardware Task created. Remain on site and observe the robot.');
          toast('Hardware Task created');
          refreshHardwareTask();
        } catch (error) {
          byId('hardwareLock').textContent = 'Execution approval rejected: ' + error.message;
          toast('Execution approval rejected');
          updateHardwareApprovalButton();
        } finally {
          button.textContent = 'Approve and create Task';
        }
      });
      byId('refreshHardwareTask').addEventListener('click', refreshHardwareTask);
      async function refreshHarnessApprovals() {
        clearTimeout(state.hardwareApprovalTimer);
        if (!(state.hardwarePolicy && state.hardwarePolicy.executeEnabled)) return;
        try {
          var result = await requestJson('/api/hardware/approvals');
          var requests = Array.isArray(result.data.requests) ? result.data.requests : [];
          var request = requests[0];
          if (!request) {
            byId('harnessApprovalQueue').textContent = 'No Harness Agent Plan is awaiting approval.';
            byId('denyHarnessApproval').disabled = true;
          } else {
            var isNewRequest = !state.hardwareApprovalRequest
              || state.hardwareApprovalRequest.requestId !== request.requestId;
            state.hardwareApprovalRequest = request;
            if (isNewRequest) renderHardwarePlan(request);
            byId('harnessApprovalQueue').textContent = pretty({
              source: request.source,
              requestId: request.requestId,
              planId: request.planId,
              instruction: request.instruction,
              status: request.status,
              expiresInSeconds: request.expiresInSeconds
            });
            byId('denyHarnessApproval').disabled = false;
            byId('hardwareLock').textContent = 'An AI Plan entered the independent approval surface. Review every Skill and complete the on-site checks.';
            announce('Received a Harness Agent hardware Plan awaiting operator approval');
          }
        } catch (error) {
          byId('harnessApprovalQueue').textContent = 'Could not read the AI approval queue: ' + error.message;
        } finally {
          state.hardwareApprovalTimer = setTimeout(refreshHarnessApprovals, 3000);
        }
      }
      byId('refreshHarnessApprovals').addEventListener('click', refreshHarnessApprovals);
      byId('denyHarnessApproval').addEventListener('click', async function () {
        if (!state.hardwareApprovalRequest) return;
        var requestId = state.hardwareApprovalRequest.requestId;
        try {
          await requestJson('/api/hardware/approvals/deny', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({ requestId: requestId })
          });
          clearHardwareApproval();
          byId('harnessApprovalQueue').textContent = 'Denied AI Plan: ' + requestId;
          byId('denyHarnessApproval').disabled = true;
          toast('AI hardware Plan denied');
        } catch (error) {
          toast('Could not deny the Plan: ' + error.message);
        }
      });
      byId('cancelHardwareTask').addEventListener('click', async function () {
        if (!state.hardwareTaskId) return;
        var button = byId('cancelHardwareTask');
        button.disabled = true;
        try {
          await requestJson('/api/hardware/task/cancel', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({ taskId: state.hardwareTaskId })
          });
          toast('Cooperative cancellation requested; this is not an emergency stop');
          refreshHardwareTask();
        } catch (error) {
          toast('Cancellation request failed: ' + error.message);
          button.disabled = false;
        }
      });

      function clearDemoTimers() {
        state.demoTimers.forEach(clearTimeout);
        state.demoTimers = [];
        clearTimeout(state.demoPollTimer);
        state.demoPollTimer = null;
      }
      function setDemoStage(index, eyebrow, title, copy) {
        byId('demoStageDots').innerHTML = [0, 1, 2, 3, 4].map(function (value) {
          return '<span class="demo-stage-dot ' + (value <= index ? 'active' : '') + '"></span>';
        }).join('');
        byId('demoEyebrow').textContent = eyebrow;
        byId('demoTitle').textContent = title;
        byId('demoCopy').textContent = copy;
      }
      function setDemoActions(html) {
        byId('demoActions').innerHTML = html;
      }
      function demoStepId(step, index) {
        return String(step.ref || step.stepId || 'step-' + index);
      }
      function demoStepSummary(step) {
        if (step.summary) return String(step.summary);
        if (step.description) return String(step.description);
        if (Array.isArray(step.tools) && step.tools.length) return step.tools.join(' · ');
        return Object.keys(step.args || {}).join(' · ') || 'Validated high-level Skill';
      }
      function renderDemoSteps(steps, stepStates) {
        return '<div class="demo-step-list">' + steps.map(function (step, index) {
          var status = stepStates && stepStates[index] ? stepStates[index] : 'pending';
          var marker = status === 'completed' ? '✓' : status === 'failed' ? '!' : String(index + 1);
          return '<div class="demo-step ' + esc(status) + '" data-demo-step="' + esc(demoStepId(step, index)) + '">' +
            '<span class="demo-step-index">' + marker + '</span><span><strong>' + esc(step.skill || 'Skill') +
            '</strong><small>' + esc(demoStepSummary(step)) + '</small></span><span>' + esc(status) + '</span></div>';
        }).join('') + '</div>';
      }
      function renderDemoProgress(progress, operation, stateName) {
        var bounded = Math.max(0, Math.min(100, Math.round(progress)));
        state.demoLastProgress = Math.max(state.demoLastProgress, bounded);
        var bar = byId('demoProgressBar');
        if (!bar) return;
        bar.style.width = state.demoLastProgress + '%';
        bar.classList.toggle('error', stateName === 'error');
        bar.classList.toggle('success', stateName === 'success');
        byId('demoProgressPercent').textContent = state.demoLastProgress + '%';
        byId('demoProgressOperation').textContent = operation;
        byId('demoProgressTrack').setAttribute('aria-valuenow', String(state.demoLastProgress));
      }
      function demoProgressMarkup(operation, progress) {
        return '<div class="demo-progress-meta"><span id="demoProgressOperation">' + esc(operation) +
          '</span><strong id="demoProgressPercent">' + Math.round(progress) + '%</strong></div>' +
          '<div class="demo-progress-track" id="demoProgressTrack" role="progressbar" aria-valuemin="0" aria-valuemax="100" aria-valuenow="' +
          Math.round(progress) + '"><div class="demo-progress-bar" id="demoProgressBar" style="width:' + Math.round(progress) + '%"></div></div>';
      }
      function enterImmersiveFullscreen() {
        if (document.fullscreenElement || !document.documentElement.requestFullscreen) return;
        try {
          var request = document.documentElement.requestFullscreen();
          if (request && typeof request.catch === 'function') request.catch(function () {});
        } catch (_error) {
          // CSS still fills the viewport when browser fullscreen is unavailable.
        }
      }
      function leaveImmersiveFullscreenIfIdle() {
        var demoVisible = !byId('demoWizard').hidden;
        var bootVisible = !byId('hardwareBootOverlay').hidden;
        var memoryVisible = !byId('memoryOverlay').hidden;
        if (demoVisible || bootVisible || memoryVisible || !document.fullscreenElement || !document.exitFullscreen) return;
        try {
          var request = document.exitFullscreen();
          if (request && typeof request.catch === 'function') request.catch(function () {});
        } catch (_error) {
          // The user may already have left fullscreen with Escape.
        }
      }
      function openMemoryOverlay() {
        enterImmersiveFullscreen();
        byId('memoryOverlay').hidden = false;
        renderMemoryLoadout();
        setTimeout(function () { byId('memoryClose').focus(); }, 40);
        announce('Robot Memory skill loadout opened');
      }
      function closeMemoryOverlay() {
        byId('memoryOverlay').hidden = true;
        leaveImmersiveFullscreenIfIdle();
        announce('Robot Memory skill loadout closed');
      }
      function closeDemoWizard() {
        clearDemoTimers();
        byId('demoWizard').hidden = true;
        state.demoResumeAfterStartup = false;
        state.demoResumeScheduled = false;
        if (state.demoTaskId && state.hardwareTaskId === state.demoTaskId) {
          state.hardwareCursor = state.demoCursor;
          state.hardwareEvents = state.demoEvents.slice();
          renderHardwareEvents();
          refreshHardwareTask();
        }
        leaveImmersiveFullscreenIfIdle();
      }
      function pauseDemoForHardwareStartup() {
        clearDemoTimers();
        state.demoResumeAfterStartup = true;
        state.demoResumeScheduled = false;
        byId('demoWizard').hidden = true;
        requestHardwareStartOrRecovery();
      }
      function resumeDemoAfterHardwareStartup() {
        state.demoResumeAfterStartup = false;
        state.demoResumeScheduled = false;
        byId('hardwareBootOverlay').hidden = true;
        state.bootOverlayActive = false;
        byId('demoWizard').hidden = false;
        prepareDemoHardware();
      }
      function renderDemoInput() {
        clearDemoTimers();
        state.demoLastProgress = 0;
        setDemoStage(0, 'STEP 1 OF 5 · COMMAND', 'What should the robot do?', 'Describe the desired result in one sentence, then press Enter.');
        byId('demoBody').innerHTML = '<form id="demoCommandForm"><label class="skip" for="demoCommand">Robot instruction</label>' +
          '<input class="demo-command" id="demoCommand" maxlength="2000" autocomplete="off" placeholder="Example: Tidy the table" value="' +
          esc(state.demoInstruction || byId('instruction').value.trim()) + '"></form>';
        setDemoActions('<button class="demo-action secondary" id="demoCancel" type="button">Cancel</button>' +
          '<button class="demo-action" id="demoContinue" type="button">Continue</button>');
        var submit = function (event) {
          if (event) event.preventDefault();
          startDemoPlanning();
        };
        byId('demoCommandForm').addEventListener('submit', submit);
        byId('demoContinue').addEventListener('click', submit);
        byId('demoCancel').addEventListener('click', closeDemoWizard);
        setTimeout(function () { byId('demoCommand').focus(); }, 50);
      }
      function openDemoWizard() {
        enterImmersiveFullscreen();
        state.demoPlan = null;
        state.demoHardwarePlan = null;
        state.demoTaskId = null;
        state.demoCursor = 0;
        state.demoEvents = [];
        state.demoPollFailures = 0;
        state.demoResumeAfterStartup = false;
        state.demoResumeScheduled = false;
        byId('demoWizard').hidden = false;
        renderDemoInput();
      }
      async function startDemoPlanning() {
        var input = byId('demoCommand');
        var instruction = input ? input.value.trim() : state.demoInstruction;
        if (!instruction) {
          if (input) input.focus();
          return;
        }
        state.demoInstruction = instruction;
        state.demoLastProgress = 8;
        setDemoStage(1, 'STEP 2 OF 5 · PLAN', 'Building the workflow…', 'The instruction is being converted into reviewed high-level Skills.');
        byId('demoBody').innerHTML = demoProgressMarkup('Understanding the instruction', 8);
        setDemoActions('<button class="demo-action secondary" id="demoCancel" type="button">Cancel</button>');
        byId('demoCancel').addEventListener('click', closeDemoWizard);
        state.demoTimers.push(setTimeout(function () {
          renderDemoProgress(30, 'Selecting robot Skills');
        }, 300));
        state.demoTimers.push(setTimeout(function () {
          renderDemoProgress(62, 'Validating parameters and dependencies');
        }, 850));
        try {
          var result = await requestJson('/api/simulate/plan', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({ instruction: instruction })
          });
          state.demoPlan = result.data;
          renderDemoProgress(100, 'Workflow ready', 'success');
          state.demoTimers.push(setTimeout(renderDemoPlanReview, 450));
        } catch (error) {
          clearDemoTimers();
          renderDemoProgress(state.demoLastProgress, 'Planning failed', 'error');
          byId('demoBody').insertAdjacentHTML('beforeend', '<div class="demo-result" style="margin-top:20px">' + esc(error.message) + '</div>');
          setDemoActions('<button class="demo-action secondary" id="demoEditCommand" type="button">Edit command</button>');
          byId('demoEditCommand').addEventListener('click', renderDemoInput);
        }
      }
      function renderDemoPlanReview() {
        clearDemoTimers();
        var steps = state.demoPlan && Array.isArray(state.demoPlan.steps) ? state.demoPlan.steps : [];
        setDemoStage(1, 'STEP 2 OF 5 · REVIEW', 'This is the proposed workflow.', 'Review the complete Skill sequence before deciding whether to use real hardware.');
        byId('demoBody').innerHTML = renderDemoSteps(steps);
        setDemoActions('<button class="demo-action secondary" id="demoEditCommand" type="button">Edit command</button>' +
          '<button class="demo-action" id="demoPlanOk" type="button">OK</button>');
        byId('demoEditCommand').addEventListener('click', renderDemoInput);
        byId('demoPlanOk').addEventListener('click', renderDemoExecutionChoice);
      }
      function renderDemoExecutionChoice() {
        setDemoStage(2, 'STEP 3 OF 5 · EXECUTION MODE', 'Run this on the real robot?', 'Choose a no-motion walkthrough or continue to supervised hardware execution.');
        byId('demoBody').innerHTML = '<div class="demo-choice-grid">' +
          '<button class="demo-choice" id="demoSimulateChoice" type="button"><strong>No · Demonstrate only</strong><span>Animate every planned step without connecting to hardware.</span></button>' +
          '<button class="demo-choice" id="demoHardwareChoice" type="button"><strong>Yes · Real robot</strong><span>Verify hardware readiness and complete the on-site safety checks.</span></button></div>';
        setDemoActions('<button class="demo-action secondary" id="demoChoiceBack" type="button">Back</button>');
        byId('demoChoiceBack').addEventListener('click', renderDemoPlanReview);
        byId('demoSimulateChoice').addEventListener('click', startDemoSimulation);
        byId('demoHardwareChoice').addEventListener('click', prepareDemoHardware);
      }
      function startDemoSimulation() {
        clearDemoTimers();
        state.demoLastProgress = 3;
        var steps = state.demoPlan && Array.isArray(state.demoPlan.steps) ? state.demoPlan.steps : [];
        var states = steps.map(function () { return 'pending'; });
        setDemoStage(3, 'STEP 4 OF 5 · SIMULATION', 'Walking through the plan…', 'This demonstration does not connect to robot hardware.');
        byId('demoBody').innerHTML = demoProgressMarkup('Preparing no-motion demonstration', 3) + renderDemoSteps(steps, states);
        setDemoActions('<button class="demo-action secondary" id="demoCancel" type="button">Cancel</button>');
        byId('demoCancel').addEventListener('click', closeDemoWizard);
        if (!steps.length) {
          renderDemoDone(false, 'The generated Plan contains no executable steps.');
          return;
        }
        steps.forEach(function (step, index) {
          state.demoTimers.push(setTimeout(function () {
            if (index > 0) {
              states[index - 1] = 'completed';
              var previous = document.querySelector('[data-demo-step="' + CSS.escape(demoStepId(steps[index - 1], index - 1)) + '"]');
              if (previous) previous.className = 'demo-step completed';
            }
            states[index] = 'active';
            var current = document.querySelector('[data-demo-step="' + CSS.escape(demoStepId(step, index)) + '"]');
            if (current) current.className = 'demo-step active';
            renderDemoProgress(8 + (index / steps.length) * 82, 'Step ' + (index + 1) + ' of ' + steps.length + ' · ' + step.skill);
          }, 450 + index * 1050));
        });
        state.demoTimers.push(setTimeout(function () {
          var last = document.querySelector('[data-demo-step="' + CSS.escape(demoStepId(steps[steps.length - 1], steps.length - 1)) + '"]');
          if (last) last.className = 'demo-step completed';
          renderDemoProgress(100, 'Simulation complete', 'success');
          state.demoTimers.push(setTimeout(function () {
            renderDemoDone(true, 'The complete workflow was demonstrated without moving hardware.');
          }, 550));
        }, 450 + steps.length * 1050));
      }
      async function prepareDemoHardware() {
        var ready = productionAgentReady(state.opsSnapshot);
        var policy = state.opsSnapshot && state.opsSnapshot.policy ? state.opsSnapshot.policy : {};
        if (!ready || !policy.executeEnabled || activeHardwareTask(state.opsSnapshot)) {
          var stack = state.opsSnapshot && state.opsSnapshot.stack ? state.opsSnapshot.stack : {};
          setDemoStage(3, 'STEP 4 OF 5 · HARDWARE CHECK', 'The robot is not ready yet.', stack.detail || 'Start the hardware stack and wait for the supervised Agent to report ready.');
          byId('demoBody').innerHTML = '<div class="demo-result">Hardware readiness is required before a real Plan can be created.</div>';
          setDemoActions('<button class="demo-action secondary" id="demoUseSimulation" type="button">Use simulation</button>' +
            '<button class="demo-action" id="demoStartHardware" type="button">Start hardware</button>');
          byId('demoUseSimulation').addEventListener('click', startDemoSimulation);
          byId('demoStartHardware').addEventListener('click', pauseDemoForHardwareStartup);
          return;
        }
        state.demoLastProgress = 8;
        setDemoStage(3, 'STEP 4 OF 5 · HARDWARE PLAN', 'Preparing real execution…', 'The Python Agent is validating a short-lived hardware Plan. No motion has started.');
        byId('demoBody').innerHTML = demoProgressMarkup('Contacting the supervised Python Agent', 8);
        setDemoActions('<button class="demo-action secondary" id="demoCancel" type="button">Cancel</button>');
        byId('demoCancel').addEventListener('click', closeDemoWizard);
        state.demoTimers.push(setTimeout(function () {
          renderDemoProgress(38, 'Validating the hardware Skill allowlist');
        }, 350));
        try {
          var result = await requestJson('/api/hardware/plan', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({ instruction: state.demoInstruction })
          });
          state.demoHardwarePlan = result.data;
          renderDemoProgress(100, 'Hardware Plan ready', 'success');
          state.demoTimers.push(setTimeout(renderDemoSafetyChecks, 450));
        } catch (error) {
          clearDemoTimers();
          renderDemoProgress(state.demoLastProgress, 'Hardware planning failed', 'error');
          byId('demoBody').insertAdjacentHTML('beforeend', '<div class="demo-result" style="margin-top:20px">' + esc(error.message) + '</div>');
          setDemoActions('<button class="demo-action secondary" id="demoUseSimulation" type="button">Use simulation</button>' +
            '<button class="demo-action" id="demoCloseError" type="button">Close</button>');
          byId('demoUseSimulation').addEventListener('click', startDemoSimulation);
          byId('demoCloseError').addEventListener('click', closeDemoWizard);
        }
      }
      function renderDemoSafetyChecks() {
        clearDemoTimers();
        var steps = state.demoHardwarePlan && state.demoHardwarePlan.plan && Array.isArray(state.demoHardwarePlan.plan.steps)
          ? state.demoHardwarePlan.plan.steps
          : [];
        setDemoStage(3, 'STEP 4 OF 5 · ON-SITE CHECKS', 'Confirm the physical workspace.', 'Every item must be personally verified before a real hardware Task can be created.');
        var checks = [
          'I will remain beside the robot and observe the complete Task.',
          'The workspace is clear of people, cables, and unknown obstacles.',
          'The physical emergency stop is reachable and verified.',
          'The robot, grippers, cameras, and CAN state have been inspected.'
        ];
        byId('demoBody').innerHTML = renderDemoSteps(steps) + '<div class="demo-safety-list" style="margin-top:20px">' +
          checks.map(function (label) {
            return '<label class="demo-safety-item"><input type="checkbox" data-demo-safety><span>' + esc(label) + '</span></label>';
          }).join('') + '</div>';
        setDemoActions('<button class="demo-action secondary" id="demoSafetyBack" type="button">Back</button>' +
          '<button class="demo-action danger" id="demoExecuteHardware" type="button" disabled>Approve and execute</button>');
        var update = function () {
          byId('demoExecuteHardware').disabled = !Array.from(document.querySelectorAll('[data-demo-safety]')).every(function (node) {
            return node.checked;
          });
        };
        Array.from(document.querySelectorAll('[data-demo-safety]')).forEach(function (node) {
          node.addEventListener('change', update);
        });
        byId('demoSafetyBack').addEventListener('click', renderDemoExecutionChoice);
        byId('demoExecuteHardware').addEventListener('click', executeDemoHardware);
      }
      async function executeDemoHardware() {
        if (!state.demoHardwarePlan) return;
        var button = byId('demoExecuteHardware');
        button.disabled = true;
        button.textContent = 'Creating Task…';
        try {
          var result = await requestJson('/api/hardware/execute', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({
              planId: state.demoHardwarePlan.planId,
              confirmation: boot.confirmation,
              checklist: {
                operatorPresent: true,
                workspaceClear: true,
                emergencyStopReady: true,
                robotInspected: true
              }
            })
          });
          state.demoTaskId = result.data.taskId;
          state.demoCursor = 0;
          state.demoEvents = [];
          state.demoPollFailures = 0;
          state.hardwareTaskId = result.data.taskId;
          state.hardwareCursor = 0;
          state.hardwareEvents = [];
          state.demoLastProgress = 3;
          renderDemoHardwareExecution();
          pollDemoHardwareTask();
        } catch (error) {
          button.disabled = false;
          button.textContent = 'Approve and execute';
          byId('demoCopy').textContent = 'Execution was rejected: ' + error.message;
        }
      }
      function renderDemoHardwareExecution() {
        var steps = state.demoHardwarePlan && state.demoHardwarePlan.plan && Array.isArray(state.demoHardwarePlan.plan.steps)
          ? state.demoHardwarePlan.plan.steps
          : [];
        setDemoStage(3, 'STEP 4 OF 5 · REAL EXECUTION', 'The robot is executing the Plan.', 'Remain beside the robot. Web cancellation is not an emergency stop.');
        byId('demoBody').innerHTML = demoProgressMarkup('Waiting for the Python Agent', state.demoLastProgress || 3) +
          renderDemoSteps(steps, steps.map(function () { return 'pending'; })) +
          '<div class="demo-event-log" id="demoEventLog">Waiting for authoritative Task events…</div>';
        setDemoActions('<button class="demo-action secondary" id="demoCancelTask" type="button">Request cooperative stop</button>' +
          '<button class="demo-action danger" id="demoStopHardwareNow" type="button">Stop now &amp; shut down hardware</button>');
        byId('demoCancelTask').addEventListener('click', cancelDemoHardwareTask);
        byId('demoStopHardwareNow').addEventListener('click', stopDemoHardwareNow);
      }
      function demoEventCounters(event, currentKey, totalKey, pattern) {
        var current = Number(event && event[currentKey]);
        var total = Number(event && event[totalKey]);
        if ((!Number.isFinite(current) || !Number.isFinite(total)) && event) {
          var match = String(event.message || '').match(pattern);
          if (match) {
            current = Number(match[1]);
            total = Number(match[2]);
          }
        }
        return Number.isFinite(current) && Number.isFinite(total) && current > 0 && total > 0
          ? { current: Math.min(current, total), total: total }
          : null;
      }
      function demoActiveStepFraction(events) {
        var placement = null;
        var placementEventIndex = -1;
        var action = null;
        var actionEventIndex = -1;
        events.forEach(function (event, eventIndex) {
          if (event.type === 'tidy.placement.started' || event.type === 'tidy.placement.completed') {
            var placementCounters = demoEventCounters(
              event,
              'placementIndex',
              'placementTotal',
              /\u7b2c[ ]*([0-9]+)[ ]*\/[ ]*([0-9]+)[ ]*\u9879/
            );
            if (placementCounters) {
              placement = {
                current: placementCounters.current,
                total: placementCounters.total,
                completed: event.type === 'tidy.placement.completed'
              };
              placementEventIndex = eventIndex;
            }
          }
          if (event.type === 'operation.stage.started' || event.type === 'operation.stage.completed') {
            var actionCounters = demoEventCounters(
              event,
              'actionIndex',
              'actionTotal',
              /\u73af\u8282[ ]*([0-9]+)[ ]*\/[ ]*([0-9]+)/
            );
            if (actionCounters) {
              action = {
                current: actionCounters.current,
                total: actionCounters.total,
                completed: event.type === 'operation.stage.completed'
              };
              actionEventIndex = eventIndex;
            }
          }
        });
        var actionFraction = 0.08;
        if (action && (!placement || actionEventIndex > placementEventIndex || placement.completed)) {
          actionFraction = action.completed
            ? action.current / action.total
            : Math.max(0.04, (action.current - 0.5) / action.total);
        }
        var fraction = actionFraction;
        if (placement) {
          fraction = placement.completed && placementEventIndex > actionEventIndex
            ? placement.current / placement.total
            : ((placement.current - 1) + actionFraction) / placement.total;
        }
        return Math.max(0.04, Math.min(0.98, fraction));
      }
      function updateDemoHardwareExecution(status) {
        var steps = state.demoHardwarePlan && state.demoHardwarePlan.plan && Array.isArray(state.demoHardwarePlan.plan.steps)
          ? state.demoHardwarePlan.plan.steps
          : [];
        var states = steps.map(function () { return 'pending'; });
        var latestEvent = state.demoEvents[state.demoEvents.length - 1];
        state.demoEvents.forEach(function (event) {
          if (!event.stepId) return;
          var index = steps.findIndex(function (step, stepIndex) {
            return demoStepId(step, stepIndex) === String(event.stepId);
          });
          if (index < 0) return;
          if (event.type === 'step.completed') states[index] = 'completed';
          else if (event.type === 'step.failed') states[index] = 'failed';
          else if (event.type === 'step.started' && states[index] === 'pending') states[index] = 'active';
        });
        steps.forEach(function (step, index) {
          var node = document.querySelector('[data-demo-step="' + CSS.escape(demoStepId(step, index)) + '"]');
          if (!node) return;
          node.className = 'demo-step ' + states[index];
          var marker = node.querySelector('.demo-step-index');
          if (marker) marker.textContent = states[index] === 'completed' ? '✓' : states[index] === 'failed' ? '!' : String(index + 1);
          var statusNode = node.lastElementChild;
          if (statusNode) statusNode.textContent = states[index];
        });
        var completed = states.filter(function (value) { return value === 'completed'; }).length;
        var activeIndex = states.indexOf('active');
        var activeEvents = [];
        if (activeIndex >= 0) {
          var activeStepId = demoStepId(steps[activeIndex], activeIndex);
          var activeStartIndex = state.demoEvents.map(function (event) {
            return event.type === 'step.started' && String(event.stepId) === activeStepId;
          }).lastIndexOf(true);
          activeEvents = activeStartIndex >= 0 ? state.demoEvents.slice(activeStartIndex + 1) : [];
        }
        var activeFraction = activeIndex >= 0 ? demoActiveStepFraction(activeEvents) : 0;
        var progress = steps.length ? 5 + ((completed + activeFraction) / steps.length) * 90 : 5;
        var operation = latestEvent
          ? (latestEvent.message || latestEvent.type || latestEvent.status)
          : 'Task state · ' + String(status.status || status.state || 'queued');
        renderDemoProgress(progress, operation);
        byId('demoEventLog').innerHTML = state.demoEvents.slice(-6).map(function (event) {
          return '<div>' + esc(event.type || event.event || 'task.event') + ' · ' + esc(event.message || event.status || '') + '</div>';
        }).join('') || 'Waiting for authoritative Task events…';
      }
      async function pollDemoHardwareTask() {
        clearTimeout(state.demoPollTimer);
        if (!state.demoTaskId || byId('demoWizard').hidden) return;
        try {
          var query = '?taskId=' + encodeURIComponent(state.demoTaskId);
          var results = await Promise.all([
            requestJson('/api/hardware/task/status' + query),
            requestJson('/api/hardware/task/events' + query + '&cursor=' + encodeURIComponent(state.demoCursor))
          ]);
          var status = results[0].data;
          var eventData = results[1].data;
          if (Array.isArray(eventData.events)) state.demoEvents = state.demoEvents.concat(eventData.events);
          if (eventData.nextCursor !== undefined) state.demoCursor = eventData.nextCursor;
          state.demoPollFailures = 0;
          updateDemoHardwareExecution(status);
          if (hardwareTaskIsTerminal(status)) {
            var value = String(status.status || status.state || '').toLowerCase();
            var success = ['succeeded', 'success', 'completed'].includes(value);
            var failedEvent = state.demoEvents.slice().reverse().find(function (event) {
              return event.type === 'step.failed' || event.status === 'failed';
            });
            renderDemoDone(success, success
              ? 'The Python Agent reported that every hardware step completed.'
              : (status.error || status.message || (failedEvent && failedEvent.message) || 'The hardware Task did not complete.'));
          } else {
            state.demoPollTimer = setTimeout(pollDemoHardwareTask, 1000);
          }
        } catch (error) {
          state.demoPollFailures += 1;
          if (state.demoPollFailures >= 3) {
            renderDemoDone(false, 'Task status could not be read: ' + error.message);
          } else {
            renderDemoProgress(state.demoLastProgress, 'Reconnecting to Task status…');
            state.demoPollTimer = setTimeout(pollDemoHardwareTask, 1200);
          }
        }
      }
      async function cancelDemoHardwareTask() {
        if (!state.demoTaskId) return;
        byId('demoCancelTask').disabled = true;
        try {
          await requestJson('/api/hardware/task/cancel', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({ taskId: state.demoTaskId })
          });
          renderDemoProgress(state.demoLastProgress, 'Cooperative cancellation requested…');
        } catch (error) {
          byId('demoCancelTask').disabled = false;
          byId('demoCopy').textContent = 'Cancellation failed: ' + error.message;
        }
      }
      async function stopDemoHardwareNow() {
        var stopButton = byId('demoStopHardwareNow');
        var cancelButton = byId('demoCancelTask');
        if (stopButton) {
          stopButton.disabled = true;
          stopButton.textContent = 'Stopping hardware…';
        }
        if (cancelButton) cancelButton.disabled = true;
        renderDemoProgress(state.demoLastProgress, 'Stopping Task and shutting down the hardware stack…');
        var cancellation = state.demoTaskId
          ? requestJson('/api/hardware/task/cancel', {
              method: 'POST',
              headers: { 'content-type': 'application/json' },
              body: JSON.stringify({ taskId: state.demoTaskId })
            }).catch(function () { return null; })
          : Promise.resolve(null);
        var results = await Promise.all([cancellation, requestHardwareStop()]);
        if (results[1] === true) {
          renderDemoDone(false, 'The operator requested an immediate software stop. Task cancellation and hardware-stack shutdown were submitted. Use the physical emergency stop for unexpected motion.');
          return;
        }
        if (stopButton) {
          stopButton.disabled = false;
          stopButton.textContent = 'Stop now & shut down hardware';
        }
        if (cancelButton) cancelButton.disabled = false;
        byId('demoCopy').textContent = 'The hardware shutdown request failed. Use the physical emergency stop if motion is unsafe.';
      }
      function renderDemoDone(success, message) {
        clearDemoTimers();
        setDemoStage(4, 'STEP 5 OF 5 · COMPLETE', success ? 'The guided flow is complete.' : 'The guided flow stopped.', success
          ? 'Review the result below or start another demonstration.'
          : 'The reported reason is shown below.');
        state.demoLastProgress = success ? 100 : state.demoLastProgress;
        byId('demoBody').innerHTML = demoProgressMarkup(success ? 'Complete' : 'Stopped', state.demoLastProgress) +
          '<div class="demo-result" style="margin-top:22px">' + esc(message) + '</div>';
        renderDemoProgress(state.demoLastProgress, success ? 'Complete' : 'Stopped', success ? 'success' : 'error');
        setDemoActions('<button class="demo-action secondary" id="demoAgain" type="button">Run another demo</button>' +
          '<button class="demo-action" id="demoFinish" type="button">Done</button>');
        byId('demoAgain').addEventListener('click', renderDemoInput);
        byId('demoFinish').addEventListener('click', closeDemoWizard);
      }

      function setOpsState(id, text, connected) {
        var node = byId(id);
        node.textContent = text;
        node.classList.toggle('online', connected === true);
        node.classList.toggle('offline', connected === false);
        node.classList.toggle('pending', connected === null);
      }
      function setStackStatusUI(stack) {
        var panel = byId('stackFaultPanel');
        var button = byId('recoverHardwareStack');
        var stateName = stack && stack.state ? stack.state : 'faulted';
        var offline = stateName === 'offline';
        var healthy = stateName === 'healthy';
        var recovering = stateName === 'recovering';
        var starting = stateName === 'starting';
        var stopping = stateName === 'stopping';
        setOpsState(
          'opsStackState',
          offline
            ? 'Offline · Ready for local startup'
            : healthy
              ? 'Healthy · Available'
              : starting
                ? 'Starting · Initialization in progress…'
                : recovering
                  ? 'Operator recovery in progress…'
                  : stopping
                    ? 'Stopping · Hardware shutdown in progress…'
                  : 'Locked · Operator action required',
          healthy ? true : offline || starting || recovering || stopping ? null : false
        );
        panel.hidden = offline || healthy || starting || stopping;
        byId('stackFaultTitle').textContent = recovering
          ? 'Reinitializing the hardware stack'
          : 'Hardware execution is locked';
        byId('stackFaultDetail').textContent = stack && stack.detail
          ? stack.detail
          : recovering
            ? 'Only the control stack and Python Agent restart; this dashboard remains online.'
            : 'This dashboard remains online. Inspect the robot and emergency stop before recovery.';
        button.disabled = recovering || state.recoveryInFlight || !stack || !stack.recoverable;
        button.textContent = recovering || state.recoveryInFlight
          ? 'Recovering…'
          : stack && stack.recoverable
            ? 'Reinitialize hardware'
            : 'Recovery unavailable';
      }
      function setStartupUI(snapshot, productionReady) {
        var button = byId('startHardwareStack');
        var title = byId('stackStartupTitle');
        var detail = byId('stackStartupDetail');
        var policy = snapshot.policy || {};
        var stack = snapshot.stack || {};
        var available = snapshot.operations && snapshot.operations.startupAvailable === true;
        var lifecycleBusy = stack.state === 'starting' || stack.state === 'recovering';
        if (stack.state === 'stopping' || state.stopInFlight) {
          title.textContent = 'Stopping robot stack';
          detail.textContent = 'The Python Agent and hardware control stack are shutting down; this page and Harness remain online.';
          button.textContent = 'Stopping…';
          button.disabled = true;
        } else if (productionReady) {
          title.textContent = 'Robot stack ready';
          detail.textContent = 'The supervised Python Agent and control stack are online.';
          button.textContent = 'Ready';
          button.disabled = true;
          state.startupInFlight = false;
        } else if (!policy.executeEnabled) {
          title.textContent = 'One-click robot startup';
          detail.textContent = 'Disabled in this simulation preview. Open the local production console to initialize hardware.';
          button.textContent = 'Simulation mode';
          button.disabled = true;
        } else if (stack.state === 'faulted' && !state.startupInFlight && !state.recoveryInFlight) {
          title.textContent = stack.recoverable ? 'Hardware startup failed' : 'Cold start unavailable';
          detail.textContent = stack.detail || 'Inspect the robot and physical emergency stop before retrying.';
          button.textContent = stack.recoverable ? 'Retry startup' : 'Recovery unavailable';
          button.disabled = !stack.recoverable;
        } else if (lifecycleBusy || state.startupInFlight || state.recoveryInFlight) {
          title.textContent = stack.state === 'recovering' || state.recoveryInFlight
            ? 'Hardware recovery in progress'
            : 'Starting robot stack';
          detail.textContent = 'The control stack and supervised Python Agent are being initialized.';
          button.textContent = 'Starting…';
          button.disabled = true;
        } else if (!available) {
          title.textContent = 'One-click robot startup';
          detail.textContent = 'No reviewed local startup launcher is configured for this server.';
          button.textContent = 'Unavailable';
          button.disabled = true;
        } else {
          title.textContent = 'One-click robot startup';
          detail.textContent = 'On-site operator only: verify the robot and emergency stop before initializing the control stack and Python Agent.';
          button.textContent = 'Start robot stack';
          button.disabled = false;
        }
      }
      function setHardwarePowerToggle(snapshot, productionReady) {
        var button = byId('hardwarePowerToggle');
        var label = byId('hardwarePowerLabel');
        var stack = snapshot && snapshot.stack ? snapshot.stack : {};
        var policy = snapshot && snapshot.policy ? snapshot.policy : {};
        var operations = snapshot && snapshot.operations ? snapshot.operations : {};
        var stateName = stack.state || 'unknown';
        var text = 'Hardware unavailable';
        var visualState = 'unavailable';
        var action = 'start';
        var disabled = true;

        if (!snapshot) {
          text = 'Status unavailable';
        } else if (!policy.executeEnabled) {
          text = 'Simulation mode';
        } else if (state.stopInFlight || stateName === 'stopping') {
          text = 'Stopping hardware…';
          visualState = 'busy';
        } else if (state.startupInFlight || stateName === 'starting') {
          text = 'Starting hardware…';
          visualState = 'busy';
        } else if (state.recoveryInFlight || stateName === 'recovering') {
          text = 'Recovering hardware…';
          visualState = 'busy';
        } else if (stateName === 'faulted') {
          text = stack.recoverable ? 'Retry hardware' : 'Hardware fault';
          visualState = stack.recoverable ? 'offline' : 'unavailable';
          action = 'recover';
          disabled = !stack.recoverable;
        } else if (productionReady) {
          text = operations.stopAvailable === true ? 'Stop hardware' : 'Stop unavailable';
          visualState = 'online';
          action = 'stop';
          disabled = operations.stopAvailable !== true;
        } else if (operations.startupAvailable === true) {
          text = 'Start hardware';
          visualState = 'offline';
          disabled = false;
        }

        button.dataset.state = visualState;
        button.dataset.action = action;
        button.disabled = disabled;
        button.setAttribute('aria-label', text);
        button.title = action === 'stop' && !disabled
          ? 'Stop the Python Agent and hardware control stack; this is not an emergency stop.'
          : text;
        label.textContent = text;
      }
      function renderHardwareBootProgress(progress, operation) {
        var bounded = Math.max(0, Math.min(100, Math.round(progress)));
        state.bootProgress = Math.max(state.bootProgress, bounded);
        byId('hardwareBootProgress').style.width = state.bootProgress + '%';
        byId('hardwareBootPercent').textContent = state.bootProgress + '%';
        byId('hardwareBootOperation').textContent = operation;
        byId('hardwareBootTrack').setAttribute('aria-valuenow', String(state.bootProgress));
      }
      function openHardwareBootOverlay() {
        enterImmersiveFullscreen();
        clearTimeout(state.bootHideTimer);
        state.bootOverlayActive = true;
        state.bootProgress = 0;
        var overlay = byId('hardwareBootOverlay');
        overlay.hidden = false;
        overlay.dataset.state = 'loading';
        byId('hardwareBootTitle').textContent = 'Starting robot hardware';
        byId('hardwareBootSubtitle').textContent = 'Preparing the supervised hardware runtime…';
        byId('hardwareBootError').hidden = true;
        byId('hardwareBootClose').hidden = true;
        renderHardwareBootProgress(2, 'Submitting startup request');
      }
      function showHardwareBootFailure(reason) {
        clearTimeout(state.bootHideTimer);
        state.bootOverlayActive = true;
        var overlay = byId('hardwareBootOverlay');
        overlay.hidden = false;
        overlay.dataset.state = 'error';
        byId('hardwareBootTitle').textContent = 'Hardware startup failed';
        byId('hardwareBootSubtitle').textContent = 'The robot was not declared ready.';
        renderHardwareBootProgress(state.bootProgress || 5, 'Startup stopped');
        byId('hardwareBootError').textContent = reason || 'No failure reason was reported.';
        byId('hardwareBootError').hidden = false;
        byId('hardwareBootClose').hidden = false;
        byId('hardwareBootClose').focus();
        announce('Hardware startup failed: ' + (reason || 'unknown reason'));
      }
      function lifecycleStatusPredatesRequest(stack) {
        if (!state.lifecycleRequestStartedAt || !stack || !stack.updatedAt) return false;
        var updatedAt = Date.parse(stack.updatedAt);
        return Number.isFinite(updatedAt) && updatedAt < state.lifecycleRequestStartedAt;
      }
      function updateHardwareBootUI(snapshot, productionReady) {
        var stack = snapshot && snapshot.stack ? snapshot.stack : {};
        var bootstrapStatus = snapshot && snapshot.bootstrap ? snapshot.bootstrap : {};
        var staleLifecycleStatus = lifecycleStatusPredatesRequest(stack);
        if ((stack.state === 'starting' || stack.state === 'recovering') && !state.bootOverlayActive) {
          openHardwareBootOverlay();
          if (stack.state === 'recovering') {
            byId('hardwareBootTitle').textContent = 'Reinitializing robot hardware';
          }
        }
        if (!state.bootOverlayActive) return;

        if (productionReady) {
          var overlay = byId('hardwareBootOverlay');
          overlay.dataset.state = 'success';
          byId('hardwareBootTitle').textContent = 'Robot hardware ready';
          byId('hardwareBootSubtitle').textContent = 'Control stack, cameras, and perception models are online.';
          renderHardwareBootProgress(100, 'Startup complete');
          byId('hardwareBootError').hidden = true;
          byId('hardwareBootClose').hidden = true;
          if (state.demoResumeAfterStartup && !state.demoResumeScheduled) {
            clearTimeout(state.bootHideTimer);
            state.demoResumeScheduled = true;
            state.bootHideTimer = setTimeout(resumeDemoAfterHardwareStartup, 1100);
          } else if (!state.demoResumeAfterStartup) {
            clearTimeout(state.bootHideTimer);
            state.bootHideTimer = setTimeout(function () {
              overlay.hidden = true;
              state.bootOverlayActive = false;
              leaveImmersiveFullscreenIfIdle();
            }, 1400);
          }
          return;
        }

        if (stack.state === 'faulted' && !staleLifecycleStatus) {
          showHardwareBootFailure(bootstrapStatus.error || stack.detail || stack.reason);
          return;
        }
        if (bootstrapStatus.state === 'error' && stack.state === 'starting') {
          showHardwareBootFailure(bootstrapStatus.error || 'Python Agent initialization failed.');
          return;
        }

        var progress = typeof stack.progress === 'number' ? stack.progress : 5;
        var operation = stack.operation || stack.detail || 'Waiting for lifecycle status';
        if (staleLifecycleStatus) {
          operation = state.recoveryInFlight
            ? 'Waiting for the recovery launcher…'
            : 'Waiting for the hardware launcher…';
        }
        if (bootstrapStatus.state === 'loading') {
          var total = Math.max(1, Number(bootstrapStatus.total) || 1);
          var current = Math.max(0, Number(bootstrapStatus.current) || 0);
          progress = total === 1 ? 58 : 30 + (Math.min(current, total - 1) / (total - 1)) * 58;
          operation = bootstrapStatus.label || 'Loading Python Agent';
          if (bootstrapStatus.detail) operation += ' · ' + bootstrapStatus.detail;
          byId('hardwareBootSubtitle').textContent = 'Initializing cameras, robot interfaces, and perception models…';
        }
        renderHardwareBootProgress(progress, operation);
      }
      async function refreshOpsStatus() {
        clearTimeout(state.opsTimer);
        try {
          var snapshot = (await requestJson('/api/ops/status')).data;
          state.opsSnapshot = snapshot;
          var staleLifecycleStatus = lifecycleStatusPredatesRequest(snapshot.stack);
          var policy = snapshot.policy;
          var agentData = snapshot.agent && snapshot.agent.data ? snapshot.agent.data : {};
          var productionReady = snapshot.agent.connected && agentData.ready === true;
          setHardwarePolicyUI(policy);
          setOpsState(
            'opsAgentState',
            !snapshot.agent.connected
              ? 'Offline · Execution blocked'
              : productionReady
                ? 'Ready · Sole hardware owner'
                : policy.readonlyEnabled
                  ? 'Connected · Bootstrap not ready'
                  : 'Simulation provider · Hardware blocked',
            !snapshot.agent.connected ? false : productionReady ? true : null
          );
          setOpsState(
            'opsTaskState',
            !policy.readonlyEnabled
              ? 'None · Simulation only'
              : !snapshot.agent.connected
                ? 'Unknown · Agent offline'
                : !productionReady
                  ? 'Unavailable · Agent not ready'
                  : agentData.activeTask
                    ? 'Running · Hardware occupied'
                    : 'None · Robot idle',
            !policy.readonlyEnabled
              ? null
              : !snapshot.agent.connected || !productionReady
                ? false
                : agentData.activeTask ? null : true
          );
          setOpsState(
            'opsPolicyState',
            policy.executeEnabled ? 'Per-Plan operator approval' : policy.readonlyEnabled ? 'Read-only mode' : 'Simulation mode',
            policy.executeEnabled
          );
          setStackStatusUI(snapshot.stack);
          setStartupUI(snapshot, productionReady);
          if (
            snapshot.stack
            && snapshot.stack.state !== 'recovering'
            && !staleLifecycleStatus
          ) {
            state.recoveryInFlight = false;
          }
          if (
            snapshot.stack
            && snapshot.stack.state === 'faulted'
            && !staleLifecycleStatus
          ) {
            state.startupInFlight = false;
          }
          if (productionReady || (snapshot.stack && snapshot.stack.state === 'faulted' && !staleLifecycleStatus)) {
            state.lifecycleRequestStartedAt = 0;
          }
          if (snapshot.stack && (snapshot.stack.state === 'offline' || snapshot.stack.state === 'faulted')) {
            state.stopInFlight = false;
          }
          setHardwarePowerToggle(snapshot, productionReady);
          updateHardwareBootUI(snapshot, productionReady);
          updateWorkspaceModeUI();
        } catch (error) {
          state.opsSnapshot = null;
          setOpsState('opsAgentState', 'Status unavailable', false);
          setOpsState('opsTaskState', 'Status unavailable', false);
          setOpsState('opsPolicyState', 'Policy unavailable', false);
          setOpsState('opsStackState', 'Supervisor status unavailable', false);
          setHardwarePowerToggle(null, false);
          if (state.bootOverlayActive) {
            renderHardwareBootProgress(state.bootProgress, 'Waiting for startup status…');
          }
          updateWorkspaceModeUI();
        } finally {
          state.opsTimer = setTimeout(refreshOpsStatus, 3000);
        }
      }
      async function requestHardwareStartup() {
        if (state.startupInFlight) return;
        state.startupInFlight = true;
        state.lifecycleRequestStartedAt = Date.now();
        byId('startHardwareStack').disabled = true;
        byId('startHardwareStack').textContent = 'Starting…';
        byId('workspaceStartHardware').disabled = true;
        byId('workspaceStartHardware').textContent = 'Starting…';
        openHardwareBootOverlay();
        setHardwarePowerToggle(state.opsSnapshot, false);
        try {
          await requestJson('/api/ops/start', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: '{}'
          });
          toast('Robot stack startup accepted; monitoring readiness');
        } catch (error) {
          state.startupInFlight = false;
          state.lifecycleRequestStartedAt = 0;
          toast('Could not start robot stack: ' + error.message);
          showHardwareBootFailure(error.message);
        }
        updateWorkspaceModeUI();
        refreshOpsStatus();
      }
      async function requestHardwareRecovery() {
        if (state.recoveryInFlight) return;
        state.recoveryInFlight = true;
        state.lifecycleRequestStartedAt = Date.now();
        openHardwareBootOverlay();
        byId('hardwareBootTitle').textContent = 'Reinitializing robot hardware';
        byId('hardwareBootSubtitle').textContent = 'Cleaning the previous hardware state before startup…';
        renderHardwareBootProgress(3, 'Submitting recovery request');
        setStackStatusUI({
          state: 'recovering',
          recoverable: false,
          detail: 'Submitting an operator recovery request…'
        });
        setHardwarePowerToggle(state.opsSnapshot, false);
        updateWorkspaceModeUI();
        try {
          await requestJson('/api/ops/recover', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: '{}'
          });
          toast('Hardware recovery started; monitoring readiness');
        } catch (error) {
          state.recoveryInFlight = false;
          state.lifecycleRequestStartedAt = 0;
          toast('Could not start recovery: ' + error.message);
          showHardwareBootFailure(error.message);
        }
        setTimeout(refreshOpsStatus, 300);
      }
      function requestHardwareStartOrRecovery() {
        var stack = state.opsSnapshot && state.opsSnapshot.stack ? state.opsSnapshot.stack : {};
        if (stack.state === 'faulted' && stack.recoverable) {
          requestHardwareRecovery();
        } else {
          requestHardwareStartup();
        }
      }
      async function requestHardwareStop() {
        if (state.stopInFlight) return true;
        state.stopInFlight = true;
        setHardwarePowerToggle(state.opsSnapshot, true);
        setStackStatusUI({
          state: 'stopping',
          recoverable: false,
          detail: 'Submitting an operator hardware shutdown request…'
        });
        updateWorkspaceModeUI();
        try {
          await requestJson('/api/ops/stop', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: '{}'
          });
          toast('Hardware shutdown accepted; the page and Harness stay online');
          announce('Hardware shutdown started');
          refreshOpsStatus();
          return true;
        } catch (error) {
          state.stopInFlight = false;
          toast('Could not stop hardware: ' + error.message);
          refreshOpsStatus();
          return false;
        }
      }
      byId('startHardwareStack').addEventListener('click', requestHardwareStartOrRecovery);
      byId('workspaceStartHardware').addEventListener('click', requestHardwareStartOrRecovery);
      byId('memoryModeToggle').addEventListener('click', openMemoryOverlay);
      byId('memoryClose').addEventListener('click', closeMemoryOverlay);
      byId('memoryZoneList').addEventListener('click', function (event) {
        var button = event.target.closest('[data-memory-zone]');
        if (button) selectMemoryZone(button.dataset.memoryZone);
      });
      byId('memorySlotGrid').addEventListener('click', function (event) {
        var button = event.target.closest('[data-memory-remove]');
        if (button) unequipMemoryProduct(Number(button.dataset.memoryRemove));
      });
      byId('memoryAvailableList').addEventListener('click', function (event) {
        var button = event.target.closest('[data-memory-equip]');
        if (button && !button.disabled) equipMemoryProduct(button.dataset.memoryEquip);
      });
      byId('demoModeToggle').addEventListener('click', openDemoWizard);
      byId('demoClose').addEventListener('click', closeDemoWizard);
      byId('hardwareBootClose').addEventListener('click', function () {
        var returnToDemo = state.demoResumeAfterStartup;
        byId('hardwareBootOverlay').hidden = true;
        state.bootOverlayActive = false;
        state.demoResumeAfterStartup = false;
        state.demoResumeScheduled = false;
        if (returnToDemo) {
          byId('demoWizard').hidden = false;
          prepareDemoHardware();
        } else {
          leaveImmersiveFullscreenIfIdle();
        }
      });
      byId('hardwarePowerToggle').addEventListener('click', function () {
        var action = byId('hardwarePowerToggle').dataset.action;
        if (action === 'stop') {
          requestHardwareStop();
        } else if (action === 'recover') {
          requestHardwareRecovery();
        } else {
          requestHardwareStartup();
        }
      });
      byId('recoverHardwareStack').addEventListener('click', requestHardwareRecovery);

      async function refreshLive() {
        var button = byId('refreshLive');
        button.disabled = true; button.textContent = 'Connecting…';
        byId('liveDetails').textContent = 'Querying the loopback Agent in read-only mode…';
        try {
          var results = await Promise.all([requestJson('/api/hardware/policy'), requestJson('/api/live/status'), requestJson('/api/live/manifest')]);
          var policy = results[0].data;
          var snapshot = results[1].data;
          var connected = snapshot.connected === true;
          setHardwarePolicyUI(policy);
          byId('liveDot').classList.toggle('connected', connected);
          byId('liveLabel').textContent = connected
            ? (policy.executeEnabled ? 'Approval mode · Connected' : policy.readonlyEnabled ? 'Read-only · Connected' : 'Plan-only · Connected')
            : (policy.readonlyEnabled ? 'Read-only · Offline' : 'Local Agent · Offline');
          byId('liveDetails').textContent = connected
            ? pretty({ stage: policy.stage, allowedMethods: policy.allowedMethods, blockedMethods: policy.blockedMethods, executeEnabled: policy.executeEnabled, status: snapshot.data, manifest: results[2].data.connected ? 'available' : 'unavailable' })
            : 'Offline: ' + (snapshot.reason || 'Python Agent unavailable') + '\n\nSimulation remains fully available.';
          toast(connected ? 'Agent status updated' : 'Agent is offline; simulation remains available');
        } catch (error) {
          byId('liveDot').classList.remove('connected');
          byId('liveLabel').textContent = 'Local Agent · Offline';
          byId('liveDetails').textContent = 'The read-only API is unavailable.\n\n' + error.message + '\n\nSimulation remains fully available.';
        } finally {
          button.disabled = false; button.textContent = 'Refresh status';
        }
      }
      async function loadHardwarePolicy() {
        try {
          var policy = (await requestJson('/api/hardware/policy')).data;
          setHardwarePolicyUI(policy);
        } catch (_) {
          byId('stageOneState').textContent = 'Policy unavailable';
          byId('stageTwoState').textContent = 'Policy unavailable';
          byId('hardwarePolicyState').textContent = 'Server execution policy unavailable';
        }
      }
      byId('liveButton').addEventListener('click', function () {
        var panel = byId('livePanel');
        var open = !panel.classList.contains('open');
        panel.classList.toggle('open', open);
        byId('liveButton').setAttribute('aria-expanded', String(open));
        if (open) {
          loadHardwarePolicy();
          byId('refreshLive').focus();
        }
      });
      byId('refreshLive').addEventListener('click', refreshLive);
      document.addEventListener('keydown', function (event) {
        if (event.key === 'Escape' && !byId('memoryOverlay').hidden) {
          closeMemoryOverlay();
          byId('memoryModeToggle').focus();
          return;
        }
        if (event.key === 'Escape' && !byId('demoWizard').hidden) {
          closeDemoWizard();
          byId('demoModeToggle').focus();
          return;
        }
        if (event.key === 'Escape' && byId('livePanel').classList.contains('open')) {
          byId('livePanel').classList.remove('open');
          byId('liveButton').setAttribute('aria-expanded', 'false');
          byId('liveButton').focus();
        }
      });

      var sections = Array.from(document.querySelectorAll('main section[id]'));
      var navLinks = Array.from(document.querySelectorAll('.nav-links a'));
      if ('IntersectionObserver' in window) {
        var observer = new IntersectionObserver(function (entries) {
          entries.forEach(function (entry) {
            if (!entry.isIntersecting) return;
            navLinks.forEach(function (link) { link.setAttribute('aria-current', String(link.hash === '#' + entry.target.id)); });
          });
        }, { rootMargin: '-25% 0px -65% 0px' });
        sections.forEach(function (section) { observer.observe(section); });
      }

      restoreMemoryLoadout();
      renderMemoryLoadout();
      renderCatalog();
      renderArchitecture();
      renderProtocolEditor();
      renderSafety();
      renderExtensions();
      updateWorkspaceModeUI();
      refreshOpsStatus();
    })();
  </script>
</body>
</html>`
