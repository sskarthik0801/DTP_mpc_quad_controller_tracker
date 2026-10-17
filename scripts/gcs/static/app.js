'use strict';
/* Web GCS front end. Vanilla, no build step, no CDN -- there is no internet at
   a flying field. Polls /api/state at 2 Hz rather than streaming: a few hundred
   bytes, and it reconnects for free when field Wi-Fi drops. */

const POLL_MS = 500;
const TOKEN_KEY = 'super-gcs-token';

/* The token, from the URL if it is there and from this browser's memory if it
   is not.

   Why remember it at all: the dashboard is reached by a hostname now
   (`http://gypsy.local:8080`), and the entire point of a memorable name is
   that you type it -- at which moment the `?k=…` is gone and every API call
   401s while the page itself loads fine, which reads as a broken dashboard
   rather than a missing credential. Remembering it makes the bare name work
   from the second visit onwards.

   localStorage and NOT a cookie, deliberately: a cookie is attached
   automatically to cross-site requests, and this API can stop the stack. A
   value in localStorage is readable only by a page from this same origin, and
   is only ever sent because app.js puts it in an explicit header.

   Every access is wrapped: localStorage throws outright in some privacy modes,
   and a dashboard must not fail to render because of a storage preference. */
function readStored() {
  try {
    return localStorage.getItem(TOKEN_KEY) || '';
  } catch (e) {
    return '';
  }
}
function storeToken(tok) {
  try {
    if (tok) localStorage.setItem(TOKEN_KEY, tok);
    else localStorage.removeItem(TOKEN_KEY);
  } catch (e) { /* private mode: it still works for this tab, just not later */ }
}
let TOKEN = new URLSearchParams(location.search).get('k') || readStored();
if (TOKEN) storeToken(TOKEN);

/* The camera's admin session. sessionStorage, NOT localStorage: the dashboard
   token is something you want remembered on your own laptop for months, and a
   camera unlock is not. This dies when the tab closes, which is the behaviour
   you want from a gate on something that watches people. The server forgets it
   too -- on restart, and after 30 minutes of a request not carrying it. */
const ADMIN_KEY = 'super-gcs-admin';
function readAdmin() {
  try {
    return sessionStorage.getItem(ADMIN_KEY) || '';
  } catch (e) {
    return '';
  }
}
function storeAdmin(tok) {
  try {
    if (tok) sessionStorage.setItem(ADMIN_KEY, tok);
    else sessionStorage.removeItem(ADMIN_KEY);
  } catch (e) { /* private mode: works for this tab, just not on reload */ }
}
let ADMIN = readAdmin();

const $ = (id) => document.getElementById(id);
const el = (tag, cls, text) => {
  const n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text !== undefined) n.textContent = text;
  return n;
};

class AuthError extends Error {}

async function api(path, opts = {}) {
  const headers = {...(opts.headers || {}), 'X-GCS-Token': TOKEN};
  if (ADMIN) headers['X-GCS-Admin'] = ADMIN;
  const r = await fetch(path, {...opts, headers});
  const body = await r.json().catch(() => ({}));
  if (r.status === 401) {
    // A remembered token that has stopped working is worse than none: it would
    // fail silently forever. Drop it and ask.
    storeToken('');
    throw new AuthError(body.error || 'bad or missing token');
  }
  if (!r.ok) throw new Error(body.message || body.error || `HTTP ${r.status}`);
  return body;
}

/* --------------------------------------------------------------- token gate */

function showGate(why) {
  $('tokengate').hidden = false;
  $('gate-why').textContent = why;
  $('token-input').value = '';
}

$('token-save').onclick = () => {
  const tok = ($('token-input').value || '').trim();
  if (!tok) return;
  TOKEN = tok;
  storeToken(tok);
  $('tokengate').hidden = true;
  // Re-run the whole boot: /api/config never loaded, so nothing is wired up.
  boot()
    .then(() => { tick(); pollLog(); say('unlocked', 'ok'); })
    .catch((e) => showGate(e instanceof AuthError
      ? 'That token was rejected. Check it against ~/.cache/super-gcs/token.'
      : 'Still cannot reach the GCS: ' + e.message));
};

$('token-input').onkeydown = (e) => {
  if (e.key === 'Enter') $('token-save').onclick();
};

/* ------------------------------------------------------------- admin gate */
/* Two credentials, two failures, two messages. Getting this wrong is easy and
   confusing: a 401 from the dashboard token must not look like a rejected
   camera password, and vice versa -- which is why the server answers a wrong
   camera password with 200 + ok:false rather than a 401 that api() would treat
   as "the dashboard token has gone bad" and throw away. */

let ADMIN_ON = false;          // last known server-side answer

function showAdminGate(why) {
  $('admingate').hidden = false;
  // Deliberately says nothing about WHAT is behind it: this text is rendered
  // for a client that has not unlocked anything.
  $('admin-why').textContent = why || 'Locked. Enter the password to continue.';
  $('admin-input').value = '';
}

function hideAdminGate() {
  $('admingate').hidden = true;
  $('admin-input').value = '';
}

async function adminLogin() {
  const pw = $('admin-input').value || '';
  if (!pw) return;
  $('admin-save').disabled = true;
  try {
    const r = await api('/api/admin/login', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({password: pw}),
    });
    if (!r.ok) {
      // Wrong password, or the server's cooldown after too many tries. Say
      // exactly which -- "wrong password" five times in a row when the real
      // answer is "wait 30s" is how you conclude the password changed.
      showAdminGate(r.message);
      say(r.message, 'err');
      return;
    }
    ADMIN = r.session;
    storeAdmin(ADMIN);
    hideAdminGate();
    // CONFIG was fetched without the session, so it has no camera row and no
    // camera log tab. Re-boot rather than patching them in by hand.
    await boot();
    tick();
    say('unlocked', 'ok');
  } catch (e) {
    say(String(e.message || e), 'err');
  } finally {
    $('admin-save').disabled = false;
  }
}

async function adminLogout() {
  try {
    await api('/api/admin/logout', {method: 'POST'});
  } catch (e) { /* the session is going away locally either way */ }
  ADMIN = '';
  storeAdmin('');
  camDetach();
  hideAdminGate();
  await boot();
  tick();
  say('locked', 'ok');
}

function renderAdmin(a) {
  // The server is the authority, not this page: `unlocked` is its answer to
  // the session we sent, so an expired one collapses the panel by itself.
  const on = !!(a && a.unlocked);
  if (ADMIN_ON && !on && ADMIN) {
    ADMIN = '';
    storeAdmin('');
    camDetach();
    say('re-locked — the admin session timed out', 'err');
  }
  ADMIN_ON = on;
  $('cam-card').hidden = !on;
  $('rc-card').hidden = !on;
  if (on && !RC) { rcSteps(); rcTick(); }
  $('admin-btn').textContent = on ? 'Lock' : 'Admin control';
  $('admin-btn').className = 'ghost small' + (on ? ' on' : '');
  if (on) hideAdminGate();
}

$('admin-btn').onclick = () => {
  if (ADMIN_ON) adminLogout();
  else if ($('admingate').hidden) showAdminGate();
  else hideAdminGate();
};
$('admin-save').onclick = adminLogin;
$('admin-cancel').onclick = hideAdminGate;
$('admin-input').onkeydown = (e) => { if (e.key === 'Enter') adminLogin(); };

let banner_timer = null;
function say(msg, kind) {
  const b = $('banner');
  b.textContent = msg;
  b.className = 'banner' + (kind ? ' ' + kind : '');
  b.hidden = false;
  clearTimeout(banner_timer);
  banner_timer = setTimeout(() => { b.hidden = true; }, kind === 'err' ? 12000 : 6000);
}

const fmt = {
  n: (v, d = 2, unit = '') => (v === null || v === undefined) ? '—' : v.toFixed(d) + unit,
  dur: (s) => {
    if (s === null || s === undefined) return '—';
    const h = Math.floor(s / 3600), m = Math.floor(s % 3600 / 60);
    return h ? `${h}h ${m}m` : (m ? `${m}m ${s % 60}s` : `${s}s`);
  },
};

function kv(target, rows) {
  target.textContent = '';
  for (const [k, v, cls] of rows) {
    target.appendChild(el('dt', null, k));
    target.appendChild(el('dd', cls || null, v));
  }
}

function pills(target, items) {
  target.textContent = '';
  for (const [text, cls] of items) target.appendChild(el('span', 'pill ' + cls, text));
}

/* ---------------------------------------------------------------- render */

let CONFIG = null;
let last_ok = 0;
let last_error = null;

let AIRCRAFT = null;         // last seen, for the FC-reboot interlock

function renderAircraft(ros) {
  const a = ros && ros.available ? ros.aircraft : null;
  AIRCRAFT = a;
  if (!a || !a.fresh) {
    $('mode').textContent = '—';
    $('mode').className = 'big';
    $('mode-sub').textContent = (ros && ros.error)
      ? ros.error
      : 'no /fmu/out/vehicle_status_v1 — is dds-agent.service up?';
    pills($('ac-badges'), [['NO LINK', 'bad']]);
    return;
  }
  $('mode').textContent = a.mode;
  $('mode').className = 'big' + (a.mode.length > 12 ? ' small' : '');
  $('mode-sub').textContent = a.mode_intended !== a.mode
    ? `pilot selected ${a.mode_intended}` : ` `;

  const b = [[a.armed ? 'ARMED' : 'DISARMED', a.armed ? 'bad' : 'ok']];
  if (a.offboard) b.push(['OFFBOARD', 'info']);
  if (a.failsafe) b.push(['FAILSAFE', 'bad']);
  if (a.landed === false) b.push(['IN AIR', 'warn']);
  b.push([a.preflight_ok ? 'preflight ok' : 'preflight fail',
          a.preflight_ok ? 'ok' : 'warn']);
  pills($('ac-badges'), b);
}

function renderBattery(ros) {
  const b = ros && ros.available ? ros.battery : null;
  const bar = $('batt-bar');
  if (!b || !b.present) {
    $('volt').textContent = '—';
    $('batt-sub').textContent = b ? b.reason : 'no telemetry';
    bar.style.width = '0';
    kv($('batt-kv'), []);
    return;
  }
  const pct = b.remaining === null ? null : b.remaining * 100;
  const warn = b.warning !== 'NONE';
  $('volt').textContent = fmt.n(b.voltage, 2, ' V');
  $('batt-sub').textContent = `${b.cells}S · ${pct === null ? '—' : pct.toFixed(0) + '%'}`
    + (warn ? ` · ${b.warning}` : '');
  bar.style.width = (pct === null ? 0 : Math.max(0, Math.min(100, pct))) + '%';
  bar.style.background = pct === null ? 'var(--faint)'
    : pct > 40 ? 'var(--ok)' : pct > 20 ? 'var(--warn)' : 'var(--bad)';

  kv($('batt-kv'), [
    // The INA238 shunt is the datasheet value and was never checked against a
    // watt meter -- docs/ark-12s-payload-power-module.md. Say so on the tile
    // rather than letting a number imply an accuracy it does not have.
    [b.current_uncalibrated ? 'Current (uncal.)' : 'Current',
     fmt.n(b.current, 2, ' A')],
    ['Discharged', fmt.n(b.discharged_mah, 0, ' mAh')],
    ['Temperature', fmt.n(b.temperature, 1, ' °C')],
    ['Est. remaining', fmt.dur(b.time_remaining_s === null ? null
                               : Math.round(b.time_remaining_s))],
    ['Warning', b.warning, warn ? 'bad' : 'ok'],
  ]);
}

function renderEstimator(ros) {
  const e = ros && ros.available ? ros.estimator : null;
  if (!e || !e.fresh) {
    pills($('est-badges'), e && e.warming
      ? [['starting…', 'muted']]
      : [['no estimator data', 'muted']]);
    kv($('est-kv'), []);
    return;
  }
  const p = [
    ['xy ' + (e.xy_valid ? 'valid' : 'INVALID'), e.xy_valid ? 'ok' : 'bad'],
    ['z ' + (e.z_valid ? 'valid' : 'INVALID'), e.z_valid ? 'ok' : 'bad'],
  ];
  if (e.dead_reckoning) p.push(['DEAD RECKONING', 'bad']);
  for (const f of ['cs_ev_pos', 'cs_ev_yaw', 'cs_ev_hgt']) {
    p.push([f.replace('cs_', ''), e[f] ? 'ok' : 'muted']);
  }
  if (e.rejects && e.rejects.length) p.push([e.rejects.join(' '), 'bad']);
  pills($('est-badges'), p);
  kv($('est-kv'), [
    ['North', fmt.n(e.n, 2, ' m')],
    ['East', fmt.n(e.e, 2, ' m')],
    ['Down', fmt.n(e.d, 2, ' m')],
    ['Heading', fmt.n(e.heading_deg, 1, '°')],
  ]);
}

function renderHost(h) {
  const swap = h.swap_used_gb;
  kv($('host-kv'), [
    ['Load', `${h.load1} (${h.load_pct}% of ${h.cores})`,
     h.load_pct > 90 ? 'warn' : ''],
    ['RAM', `${h.mem_used_gb} / ${h.mem_total_gb} GB`],
    // swap != 0 is the signature of the FAST-LIO RAM leak (CLAUDE.md, M1).
    ['Swap', `${swap} / ${h.swap_total_gb} GB`, swap > 0.01 ? 'bad' : 'ok'],
    ['Disk free', `${h.disk_free_gb} GB`],
    ['Up', fmt.dur(h.uptime_s)],
  ]);
}

/* The two rows that take a choice before Start. Remembered for this page load
   only, and never across one: the follower in particular comes back to
   OBSERVE every time the page is opened, so "live" is always a fresh decision
   rather than something a reload kept. */
let STACK_CFG = 'click';
let DTP_MODE = 'observe';
let PROC_AIRCRAFT = null;      // for the follower's stop question

const DTP_MODE_TEXT = {
  observe: 'OBSERVE-ONLY — goals and yaw go to /dtp/shadow/*, the NAV_LAND '
         + 'abort is off. It commands nothing.',
  live: 'LIVE — command_mux publishes /planning/follow_goal and holds NAV_LAND '
      + 'armed. It commands the aircraft once armed in Offboard.',
  mixed: 'MIXED — one output reaches SUPER and the other is shadowed. Stop it: '
       + 'this is not a mode anyone asked for.',
};

function optChip(label, on, onclick, title) {
  const b = el('button', 'chip' + (on ? ' sel' : ''), label);
  b.title = title || '';
  b.onclick = onclick;
  return b;
}

/* The line under the Stack row: which config fsm_node is on when it runs, and
   the choice for Start when it does not. The config is read off the running
   process, so a stack started in a terminal is described just as truthfully. */
function stackExtras(p, busy) {
  const line = el('div', 'extra');
  // A GCS process older than this file sends no config at all, and ignores
  // ?config= -- offering a choice it would silently drop is worse than none.
  if (!('config' in p)) return line;
  if (p.state !== 'stopped') {
    line.appendChild(el('span', 'note', p.config
      ? `fsm_node on ${p.config}`
      : (p.config_error || 'fsm_node not up yet')));
    return line;
  }
  const cfgs = (CONFIG && CONFIG.stack_configs) || {click: 'click_smooth_ros2.yaml'};
  for (const [key, file] of Object.entries(cfgs)) {
    const c = optChip(key, STACK_CFG === key, () => { STACK_CFG = key; tick(); },
                      `Start fsm_node on ${file}`);
    c.disabled = !!busy;
    line.appendChild(c);
  }
  line.appendChild(el('span', 'note',
    `Start uses ${cfgs[STACK_CFG] || STACK_CFG}`
    + (STACK_CFG === 'follow' ? ' — the M5.2 first-flight envelope, needed '
       + 'for a live follower' : '')));
  return line;
}

/* The follower's line: the mode it is in when it runs, the mode chips and the
   reason Start is refused when it does not. The reason is VISIBLE, not a
   tooltip -- a phone has no hover, and this is the row where a silent grey
   button would matter most. */
/* The follower's lock state as a pill: the first field of /dtp/lock_state
   (IDLE | ACQUIRING | LOCKED | COASTING | LOST) and the reason after it. */
const LOCK_CLS = {LOCKED: 'ok', COASTING: 'warn', ACQUIRING: 'info', LOST: 'bad', IDLE: 'muted'};
function lockPill(follow) {
  if (!follow || !follow.lock) return el('span', 'pill muted', 'lock ?');
  const parts = follow.lock.split('|');
  const state = parts[0];
  const p = el('span', 'pill ' + (LOCK_CLS[state] || 'muted') + (follow.fresh ? '' : ' stale'),
               state + (parts[2] ? ' · ' + parts[2].slice(0, 48) : ''));
  p.title = follow.lock + (follow.mux ? '\nmux: ' + follow.mux : '');
  return p;
}

/* Engage / Disengage FOLLOW -- the one control on this page that designates a
   target. No confirm on purpose: a lock lost in flight is re-engaged in
   seconds or not at all (2026-09-14), and the button is behind Admin control
   and refused by the server without a follower, telemetry, or after an ABORT.
   ?set=on is always a false -> true cycle, so it re-designates from LOST. */
function followButtons(p, busy) {
  const box = el('span', 'acts');
  const why = p.follow_blocked || {};
  const on = el('button', 'small caution', p.follow && p.follow.lock && !p.follow.lock.startsWith('IDLE')
                                           ? 'Re-engage FOLLOW' : 'Engage FOLLOW');
  on.disabled = !!busy || !!why.on || !p.follow;
  on.title = why.on || 'target_manager follow_enabled: false -> true. LOCKED needs a person at 2.5-10 m who is moving.';
  on.onclick = () => act('/api/dtp/follow?set=on', on);
  const off = el('button', 'small ghost', 'Disengage');
  off.disabled = !!busy || !!why.off || !p.follow;
  off.title = why.off || 'target_manager follow_enabled: false. The mux falls back to HOLD.';
  off.onclick = () => act('/api/dtp/follow?set=off', off);
  box.append(on, off);
  return box;
}

function dtpExtras(p, busy) {
  const line = el('div', 'extra');
  if (p.state !== 'stopped') {
    const m = p.mode;
    const cls = m === 'live' ? 'warn' : m === 'observe' ? 'ok' : 'bad';
    line.appendChild(el('span', 'pill ' + cls,
      m ? m.toUpperCase() + (p.mode_pending ? ' · starting' : '') : 'mode unknown'));
    line.appendChild(el('span', 'note', m ? DTP_MODE_TEXT[m]
      : 'command_mux and follow_planner are not up, so the mode cannot be read yet.'));
    // The lock line: state for everyone, the buttons for an unlocked client.
    // `follow` is absent from a server older than this page -- then no button.
    if (p.follow !== undefined) {
      const lk = el('div', 'extra');
      lk.appendChild(lockPill(p.follow));
      if (p.follow && p.follow.mux) lk.appendChild(el('span', 'note', 'mux: ' + p.follow.mux.split('|').slice(0, 2).join(' · ')));
      if (!p.locked) lk.appendChild(followButtons(p, busy));
      line.appendChild(lk);
    }
    return line;
  }
  if (p.locked) {
    line.appendChild(el('span', 'note', 'Unlock Admin control to start it.'));
    return line;
  }
  for (const m of ['observe', 'live']) {
    const c = optChip(m === 'observe' ? 'Observe only' : 'Live', DTP_MODE === m,
                      () => { DTP_MODE = m; tick(); }, DTP_MODE_TEXT[m]);
    c.disabled = !!busy;
    if (m === 'live') c.className += ' warn';
    line.appendChild(c);
  }
  const why = (p.blocked || {})[DTP_MODE];
  line.appendChild(el('span', 'note' + (why ? ' why' : ''),
    why ? `unavailable: ${why}` : DTP_MODE_TEXT[DTP_MODE]));
  return line;
}

function dtpStart(btn) {
  if (DTP_MODE === 'live') {
    if (!confirm('Start the DTP follower LIVE?\n\n'
                 + 'command_mux will publish goals on /planning/follow_goal and '
                 + 'hold a NAV_LAND abort armed. Nothing flies until the pilot '
                 + 'arms and flips channel 11 to Offboard — and channel 11 OFF '
                 + 'is how you take it back.\n\n'
                 + 'After an ABORT, stop and restart the follower before '
                 + 're-engaging channel 11.')) return;
    act('/api/proc/dtp/start?mode=live&confirm=live', btn);
    return;
  }
  act('/api/proc/dtp/start?mode=observe', btn);
}

function dtpStop(p, btn) {
  const a = PROC_AIRCRAFT;
  const note = p.stop_note ? '\n\n' + p.stop_note : '';
  if (!confirm(`Stop the DTP follower${p.mode ? ` (${p.mode.toUpperCase()})` : ''}?`
               + '\n\nIt gets SIGINT, then SIGTERM, then SIGKILL, and its camera '
               + 'driver stops with it.' + note)) return;
  // A live follower on an armed aircraft is the one stop on this page that
  // changes what the aircraft does next, so it is asked twice.
  if (a && a.armed && p.mode !== 'observe'
      && !confirm('The aircraft is ARMED. Stop the follower anyway?')) return;
  act('/api/proc/dtp/stop', btn);
}

function renderProcs(procs, busy) {
  const box = $('procs');
  box.textContent = '';
  for (const p of procs) {
    const row = el('div', 'proc');
    const [label, sub] = p.label.includes('(')
      ? [p.label.slice(0, p.label.indexOf('(')).trim(),
         p.label.slice(p.label.indexOf('(') + 1, -1)]
      : [p.label, ''];

    const name = el('div', 'name', label);
    if (sub) name.appendChild(el('small', null, sub));
    row.appendChild(name);

    const cls = {running: 'ok', external: 'warn', stopped: 'muted'}[p.state];
    let txt = p.state === 'external' ? 'running · not ours' : p.state;
    if (p.key === 'dtp' && p.state !== 'stopped' && p.mode) txt += ` · ${p.mode}`;
    row.appendChild(el('span', 'pill ' + cls, txt));

    row.appendChild(el('div', 'stats', p.pid
      ? `pid ${p.pid} · ${p.procs} proc · ${p.rss_mb} MB · ${p.cpu_pct}%`
      : (p.exit_code ? `last exit ${p.exit_code}` : ' ')));

    const acts = el('div', 'acts');
    const start = el('button', 'small', 'Start');
    // start_blocked is set by the server for rows that own a single-user
    // device -- the camera, whose USB endpoint can be held by a process this
    // dashboard did not start. The follower has one reason per mode instead.
    // Pressing it would 409 with the same sentence; not offering it is better.
    const blockedWhy = p.key === 'dtp' ? (p.blocked || {})[DTP_MODE]
                                       : p.start_blocked;
    start.disabled = !!busy || p.state !== 'stopped' || !!blockedWhy || !!p.locked;
    start.title = p.locked ? 'Needs Admin control' : (blockedWhy || '');
    start.onclick = p.key === 'dtp' ? () => dtpStart(start)
      : p.key === 'stack'
        ? () => act(`/api/proc/stack/start?config=${encodeURIComponent(STACK_CFG)}`, start)
        : () => act(`/api/proc/${p.key}/start`, start);
    if (p.key === 'dtp' && DTP_MODE === 'live' && p.state === 'stopped') {
      start.textContent = 'Start LIVE';
      start.className = 'small caution';
    }
    const stop = el('button', 'small ghost', 'Stop');
    // Nothing is ours to touch while a restart is walking the same groups.
    // An externally started process IS stoppable now: the server adopts it,
    // signalling its process group only when that group's leader is
    // recognisably ours, and falling back to individual pids when it is not.
    stop.disabled = !!busy || p.state === 'stopped' || !!p.locked;
    stop.title = p.locked ? 'Needs Admin control — STOP ALL still stops it'
      : p.state === 'external'
        ? 'Started outside this GUI — will be adopted and stopped' : '';
    stop.onclick = p.key === 'dtp' ? () => dtpStop(p, stop)
                                   : () => act(`/api/proc/${p.key}/stop`, stop);
    acts.append(start, stop);
    row.appendChild(acts);
    // After acts, never before: children[3] is the buttons, and other code
    // (and test_render.js) finds them there.
    if (p.key === 'stack') row.appendChild(stackExtras(p, busy));
    if (p.key === 'dtp') row.appendChild(dtpExtras(p, busy));
    box.appendChild(row);
  }
}

function renderRates(ros) {
  const box = $('rates');
  box.textContent = '';
  const rates = (ros && ros.available && ros.rates) || {};
  for (const [topic, r] of Object.entries(rates)) {
    const frac = r.expect ? r.hz / r.expect : 0;
    const cls = r.hz === 0 ? '' : frac > 0.8 ? 'ok' : frac > 0.4 ? 'warn' : 'bad';
    const chip = el('div', 'chip ' + cls);
    chip.append(el('span', null, topic), el('b', null,
      r.hz === 0 ? 'silent' : `${r.hz} Hz`));
    chip.title = `expected ~${r.expect} Hz`;
    box.appendChild(chip);
  }

  const pbox = $('presence');
  pbox.textContent = '';
  const pres = (ros && ros.available && ros.presence) || {};
  for (const [topic, n] of Object.entries(pres)) {
    const chip = el('div', 'chip ' + (n > 0 ? 'ok' : ''));
    chip.append(el('span', null, topic), el('b', null, n > 0 ? `${n} pub` : 'none'));
    pbox.appendChild(chip);
  }
}

/* ------------------------------------------------------------------ acts */

async function act(path, btn) {
  if (btn) btn.disabled = true;
  try {
    const r = await api(path, {method: 'POST'});
    say(r.message, 'ok');
  } catch (e) {
    say(String(e.message || e), 'err');
  }
  tick();
}

$('stop-all').onclick = () => {
  if (confirm('Stop the SUPER stack, both PX4 bridges, the DTP follower and '
              + 'the camera?\n\n'
              + 'This also stops nodes started from a terminal, not just the '
              + 'ones this dashboard started.')) {
    act('/api/stop_all', $('stop-all'));
  }
};

$('stop-external').onclick = () => {
  if (confirm('Stop the SUPER nodes that were started outside this dashboard?'
              + '\n\nThey get the same SIGINT → SIGTERM → SIGKILL as ours. '
              + 'The terminal they were started in will show them exiting.')) {
    // The detail lands in the checks log, so put the reader there.
    log_key = 'check'; log_seq = 0; $('log').textContent = ''; buildTabs();
    act('/api/stop_external', $('stop-external'));
  }
};

/* ------------------------------------------------------------ FC reboot */
/* The only button here that resets the aircraft. The server refuses unless
   live telemetry says disarmed and on the ground, and fc_reboot.py asks the FC
   the same question again over MAVLink -- this only decides whether pressing
   it can possibly do anything, and says why when it cannot. */

function rebootBlockedReason(a, blocked) {
  if (blocked) return 'busy';
  if (!a || !a.fresh) return 'no link to the FC';
  if (a.armed) return 'ARMED — disarm first';
  if (a.landed === false) return 'FC reports IN AIR';
  return null;
}

function renderReboot(blocked) {
  const why = rebootBlockedReason(AIRCRAFT, blocked);
  $('fc-reboot').disabled = !!why;
  $('fc-reboot-note').textContent = why
    ? `unavailable: ${why}`
    : 'Resets the FC. /fmu/* drops and comes back.';
}

$('fc-reboot').onclick = () => {
  if (!confirm('Reboot the FLIGHT CONTROLLER?\n\n'
               + 'Every /fmu/* topic drops and comes back, EKF2 restarts with '
               + 'no position until the odometry bridge feeds it again, and '
               + 'the XRCE session is re-formed — which is the exact path '
               + 'behind the known NuttX hardfault (firmware/faults/).\n\n'
               + 'Only do this with the aircraft on the ground and disarmed.')) return;
  log_key = 'check'; log_seq = 0; $('log').textContent = ''; buildTabs();
  act('/api/fc/reboot', $('fc-reboot'));
};

/* ------------------------------------------------------- remote control */
/* Every button here publishes ONE /goal_pose -- the same message RViz's 2D
   Goal Pose tool sends -- and SUPER plans the path, so obstacle avoidance
   stays in the loop and a dropped packet means "no new goal" rather than "no
   setpoints". Tap = one step; nothing repeats while held, because a stuck
   touch event must never turn into distance.

   There is no STOP button. SUPER's FSM has three inputs and none of them is a
   stop, so a button here would imply an abort this page cannot perform: the
   abort is the RC's channel-11 switch. Saying so is the honest interface. */

const RC_AXES = ['forward', 'back', 'left', 'right', 'up', 'down',
                 'yaw_left', 'yaw_right'];
const RC_POLL_MS = 200;          // 5 Hz -- an attitude ball is unreadable at 2
let RC = null;                   // last /api/rc
let RC_STEP = 1.0;
let RC_YAW_STEP = 45;
let RC_ONSCREEN = false;
let RC_LADDER = false;

/* The bounds the typed boxes enforce come from the server, which is also the
   thing that refuses -- two copies of a limit is two limits. These defaults
   only cover the first render, before /api/rc has answered. */
const RC_LIMIT_DEFAULTS = {
  steps: [0.25, 0.5, 1, 2, 3], yaw_steps: [15, 45, 90],
  step_min: 0.25, step_max: 3, yaw_min: 1, yaw_max: 180,
  goto_max: 10, goto_confirm: 5,
};
const rcLimits = () => Object.assign({}, RC_LIMIT_DEFAULTS,
                                     (RC && RC.limits) || {});
const svg = (tag) => document.createElementNS
  ? document.createElementNS('http://www.w3.org/2000/svg', tag) : el(tag);

/* The 100 Hz attitude subscription costs 7.5% of a core, so it is not enough
   that a tab is open: this card has to be ON SCREEN. The server drops the
   subscription 15 s after the last /api/rc, so scrolling away is enough --
   nothing has to be switched off by hand. */
function rcWatch() {
  const card = $('rc-card');
  if (typeof IntersectionObserver !== 'function' || !card.getBoundingClientRect) {
    RC_ONSCREEN = true;          // old browser: fall back to "unlocked"
    return;
  }
  new IntersectionObserver((entries) => {
    RC_ONSCREEN = entries.some((e) => e.isIntersecting);
  }, {threshold: 0.05}).observe(card);
}

async function rcTick() {
  if (!ADMIN_ON || document.hidden || !RC_ONSCREEN) return;
  try {
    RC = await api('/api/rc');
    renderRC(RC);
  } catch (e) {
    if (!(e instanceof AuthError)) {
      $('rc-note').textContent = 'no RC data: ' + (e.message || e);
    }
  }
}

function rcLadder() {
  // Drawn once. Rebuilding twenty SVG nodes five times a second would cost
  // more than the subscription feeding them.
  if (RC_LADDER) return;
  RC_LADDER = true;
  const box = $('rc-ladder');
  for (const deg of [-30, -20, -10, 10, 20, 30]) {
    const y = deg * 2.2;
    const half = Math.abs(deg) === 10 ? 22 : 32;
    const ln = document.createElementNS
      ? document.createElementNS('http://www.w3.org/2000/svg', 'line')
      : el('line');
    ln.setAttribute && ln.setAttribute('x1', -half);
    ln.setAttribute && ln.setAttribute('x2', half);
    ln.setAttribute && ln.setAttribute('y1', y);
    ln.setAttribute && ln.setAttribute('y2', y);
    box.appendChild(ln);
  }
}

function rcRose() {
  // Same reasoning as the ladder: twelve static SVG nodes, drawn once, not
  // rebuilt five times a second.
  if (RC_ROSE) return;
  RC_ROSE = true;
  const box = $('rc-rose-ticks');
  if (!box) return;
  for (let deg = 0; deg < 360; deg += 30) {
    const r = deg * Math.PI / 180;
    const long = deg % 90 === 0;
    const ln = svg('line');
    for (const [k, v] of [['x1', Math.sin(r) * 44], ['y1', -Math.cos(r) * 44],
                          ['x2', Math.sin(r) * (long ? 36 : 39)],
                          ['y2', -Math.cos(r) * (long ? 36 : 39)]]) {
      if (ln.setAttribute) ln.setAttribute(k, v);
    }
    box.appendChild(ln);
  }
}
let RC_ROSE = false;

function renderRC(st) {
  if (!st) return;
  rcLadder();
  rcRose();
  const cap = st.capabilities || {};
  const cfg = st.config || {};
  const pose = st.pose;
  const fresh = !!(pose && pose.age !== null && pose.age <= 1.0);

  // -- readiness: four things, each of which fails silently on its own -----
  const box = $('rc-ready');
  box.textContent = '';
  for (const r of st.ready || []) {
    const chip = el('div', 'chip ' + (r.ok ? 'ok' : 'warn'));
    chip.append(el('span', null, r.label), el('b', null, r.ok ? 'yes' : 'no'));
    chip.title = r.note;
    box.appendChild(chip);
  }

  // -- which goal input, and what SUPER said about the last one -----------
  const gp = st.path || {};
  const gs = st.goal_status;
  let pathTxt = gp.why ? gp.why[0].toUpperCase() + gp.why.slice(1) : '—';
  if (gp.contended) pathTxt = 'Refusing to send: ' + gp.contended;
  else if (gs && gs.ours) {
    // A real answer from the planner, not our inference -- only the follow
    // path produces one, and only for a goal we can prove was ours.
    pathTxt += ` · SUPER said ${gs.name} at `
             + `[${fmt.n(gs.adopted.x, 2)}, ${fmt.n(gs.adopted.y, 2)}, `
             + `${fmt.n(gs.adopted.z, 2)}]`;
    if (gs.name === 'DROPPED') pathTxt += ' — that is the PREVIOUS goal, '
                                        + 'still active: the new one was refused';
  } else if (gp.reports && st.status_available === false) {
    pathTxt += ' — but dtp_msgs is not installed here, so the status cannot '
             + 'be read back';
  }
  $('rc-path').textContent = pathTxt;
  $('rc-path').className = 'note rc-path' + (gp.contended ? ' rc-abort' : '');

  // -- attitude ------------------------------------------------------------
  const a = st.attitude;
  if (a) {
    // Roll right tilts the horizon left; pitch up drops it. The pitch scale
    // matches the ladder (2.2 px per degree in viewBox units).
    $('rc-ball').setAttribute && $('rc-ball').setAttribute(
      'transform', `rotate(${-a.roll}) translate(0 ${a.pitch * 2.2})`);
    $('rc-roll-ptr').setAttribute && $('rc-roll-ptr').setAttribute(
      'transform', `rotate(${-a.roll})`);
  }
  // The rose turns under a fixed nose pointer, so the letter at the top is
  // the heading -- the same convention as every compass on an instrument panel.
  if (a && $('rc-rose') && $('rc-rose').setAttribute) {
    $('rc-rose').setAttribute('transform', `rotate(${-a.yaw})`);
  }
  kv($('rc-att-kv'), a ? [
    ['Roll', fmt.n(a.roll, 1, '°'), Math.abs(a.roll) > 25 ? 'warn' : ''],
    ['Pitch', fmt.n(a.pitch, 1, '°'), Math.abs(a.pitch) > 25 ? 'warn' : ''],
    ['Yaw', fmt.n(a.yaw, 1, '°')],
  ] : [['Attitude', st.attitude_live ? 'waiting for /fmu/out/vehicle_attitude'
                                     : 'not subscribed', 'muted']]);

  // -- altitude ------------------------------------------------------------
  const band = cfg.altitude;
  const z = pose ? pose.z : null;
  $('rc-alt').textContent = z === null || z === undefined ? '—' : fmt.n(z, 2, ' m');
  $('rc-alt-sub').textContent = st.altitude === null || st.altitude === undefined
    ? 'lidar_init z — the frame goals live in'
    : `lidar_init z · FC says ${fmt.n(st.altitude, 2, ' m')}`;
  if (band) {
    $('rc-tape-max').textContent = band.max + ' m';
    $('rc-tape-min').textContent = band.min + ' m';
    const frac = (v) => Math.max(0, Math.min(1, (v - band.min) / (band.max - band.min)));
    if (z !== null && z !== undefined) {
      $('rc-tape-now').style.bottom = (frac(z) * 100) + '%';
    }
    const g = (st.last || {}).goal;
    $('rc-tape-ghost').hidden = !g;
    if (g) $('rc-tape-ghost').style.bottom = (frac(g.z) * 100) + '%';
    const sl = $('rc-alt-slider');
    sl.min = band.min;
    sl.max = band.max;
    if (!RC_SLIDER_HELD && z !== null && z !== undefined) sl.value = z;
  }
  const altCap = cap.altitude || {ok: true};
  const altLive = !!altCap.ok && fresh;
  $('rc-alt-slider').disabled = !altLive;
  // The box and the slider are the same control; only the gesture differs.
  // Never overwrite what someone is in the middle of typing.
  const ai = $('rc-alt-input');
  ai.disabled = !altLive;
  if (band) { ai.min = band.min; ai.max = band.max; }
  if (document.activeElement !== ai && z !== null && z !== undefined) {
    ai.value = Number(z).toFixed(2);
  }
  $('rc-alt-go').disabled = !altLive;
  ai.title = altLive ? `absolute height, ${band ? band.min + ' to ' + band.max : ''} m`
                     : (altCap.why || 'no fresh odometry');
  $('rc-alt-note').textContent = altCap.ok
    ? 'Release the slider, or type a height and press Go. Position is held; '
      + 'only the height changes.'
    : 'inert: ' + altCap.why;

  // -- the pad -------------------------------------------------------------
  for (const axis of RC_AXES) {
    const btn = $('rc-b-' + axis);
    if (!btn) continue;
    const need = axis === 'up' || axis === 'down' ? cap.altitude
               : axis.startsWith('yaw') ? cap.yaw : {ok: true};
    const ok = fresh && st.available !== false && (!need || need.ok);
    btn.disabled = !ok;
    btn.title = !fresh ? 'no fresh /lidar_slam/odom — goals have no frame'
              : (need && !need.ok) ? need.why
              : `${axis.replace('_', ' ')} ${axis.startsWith('yaw')
                  ? RC_YAW_STEP + '°' : RC_STEP + ' m'}`;
  }
  $('rc-step-label').textContent = `${RC_STEP} m\n${RC_YAW_STEP}°`;

  const L = rcLimits();
  for (const [id, val, lo, hi] of [
      ['rc-step-input', RC_STEP, L.step_min, L.step_max],
      ['rc-yaw-input', RC_YAW_STEP, L.yaw_min, L.yaw_max]]) {
    const inp = $(id);
    inp.min = lo;
    inp.max = hi;
    inp.title = `${lo} to ${hi}`;
    if (document.activeElement !== inp) inp.value = val;
  }

  rcGoto(st);
  rcProgress(st);

  // -- what the planner will do with all this ------------------------------
  const vel = cfg.max_vel;
  const eta = vel ? ` — at max_vel ${vel} m/s a ${RC_STEP} m step takes about `
                    + `${Math.round(RC_STEP / vel)} s` : '';
  const last = st.last || {};
  $('rc-note').textContent = (last.what
    ? `last: ${last.what} → [${last.goal.x}, ${last.goal.y}, ${last.goal.z}]`
    // The LIVE topic, not cfg.goal_topic -- that one is the click goal even
    // when every button is going somewhere else.
    : `The pad is relative to where the aircraft is NOW; goals go to `
      + `${gp.topic || cfg.goal_topic}`)
    + eta
    + (cfg.error ? ` — config: ${cfg.error}` : '');
}

let RC_SLIDER_HELD = false;
$('rc-alt-slider').oninput = () => { RC_SLIDER_HELD = true; };
$('rc-alt-slider').onchange = () => {
  // On CHANGE, not input: dragging a slider must not emit forty goals.
  rcSendAltitude(Number($('rc-alt-slider').value));
};

for (const axis of RC_AXES) {
  const btn = $('rc-b-' + axis);
  if (!btn) continue;
  btn.onclick = async () => {
    const step = axis.startsWith('yaw') ? RC_YAW_STEP : RC_STEP;
    await act(`/api/rc/move?axis=${axis}&step=${step}`, btn);
    rcTick();
  };
}

/* --------------------------------------------------- did it get there? */
/* "Goal sent" is not the interesting confirmation -- every press produces one.
   The interesting ones are the two that a plain spinner reports as "moving":
   the aircraft is disarmed so nothing will ever happen, and SUPER has no free
   path so nothing is happening. Both get their own colour and their own
   sentence. Arrival is SUPER's own closeToGoal(0.1). */

function rcProgress(st) {
  const p = st.progress;
  const box = $('rc-prog');
  box.hidden = !p;
  if (!p) return;
  box.className = 'rc-prog ' + p.state;
  $('rc-prog-label').textContent =
    (p.state === 'arrived' ? '\u2713 ' : p.state === 'rejected' ? '\u2717 '
     : p.state === 'stalled' ? '\u26a0 ' : '') + p.label;
  // Distance remaining is the number an operator actually watches, so it goes
  // on the same line as the state rather than inside the sentence.
  $('rc-prog-dist').textContent =
    (p.what ? p.what + ' \u00b7 ' : '')
    + (p.remaining === null || p.remaining === undefined
       ? `${p.elapsed}s`
       : `${fmt.n(p.remaining, 2, ' m')} to go \u00b7 ${p.elapsed}s`);
  // A bar with no total would have to invent one. Full on arrival, empty when
  // there is nothing to measure against.
  const frac = p.state === 'arrived' ? 1
             : (p.frac === null || p.frac === undefined) ? 0 : p.frac;
  $('rc-prog-fill').style.width = (frac * 100) + '%';
  $('rc-prog-detail').textContent = p.detail;
}

/* ------------------------------------------------- the coordinate bar */
/* One absolute goal in lidar_init, not a nudge. Everything here mirrors
   rc.plan_goto() so the button can be dead before it is pressed and say why --
   but the SERVER is still the authority: this is a courtesy, not a gate. */

function rcGotoRead(st) {
  const L = rcLimits();
  const pose = st.pose;
  const gp = st.path || {};
  const num = (id) => {
    const raw = ($(id).value || '').trim();
    return raw === '' ? null : Number(raw);
  };
  const bad = (id, why) => ({ok: false, why: why, focus: id});
  const fresh = !!(pose && pose.age !== null && pose.age <= 1.0);
  if (!fresh) return bad(null, 'no fresh /lidar_slam/odom — a goal has no frame');
  if (gp.contended) return bad(null, gp.contended);
  const x = num('rc-gx'), y = num('rc-gy');
  if (x === null || y === null) return bad(null, 'enter an x and a y, in metres');
  if (!isFinite(x) || !isFinite(y)) return bad(null, 'x and y must be numbers');
  let z = num('rc-gz');
  if (z === null) z = pose.z;
  else if (!isFinite(z)) return bad('rc-gz', 'z must be a number');
  const yawRaw = num('rc-gyaw');
  if (yawRaw !== null && !isFinite(yawRaw)) {
    return bad('rc-gyaw', 'yaw must be a number of degrees, or blank to hold');
  }
  // The z SUPER will really use -- the distance has to be measured to the
  // point it flies to, not the one that was typed.
  const effZ = gp.keeps_z ? z : ((st.config || {}).click_height);
  const d = Math.sqrt((x - pose.x) ** 2 + (y - pose.y) ** 2
                      + ((effZ === null || effZ === undefined ? z : effZ) - pose.z) ** 2);
  if (d > L.goto_max) {
    return {ok: false, why: `${d.toFixed(1)} m away — this bar is capped at `
                            + `${L.goto_max} m. Send it in steps`, d: d};
  }
  return {ok: true, x: x, y: y, z: z, yaw: yawRaw, d: d, effZ: effZ};
}

function rcGoto(st) {
  const L = rcLimits();
  const pose = st.pose;
  const ned = st.pose_ned;
  const f = (v) => (v === null || v === undefined || !isFinite(v))
    ? '     —' : Number(v).toFixed(2).padStart(7);
  $('rc-goto-now').textContent = pose
    ? `now  lidar_init ${f(pose.x)}${f(pose.y)}${f(pose.z)}   `
      + `yaw ${(pose.yaw * 180 / Math.PI).toFixed(0)}°
`
      + `     PX4 NED   ${f(ned && ned.x)}${f(ned && ned.y)}${f(ned && ned.z)}`
    : 'no /lidar_slam/odom — there is no position to put a goal near';

  const g = rcGotoRead(st);
  $('rc-goto-send').disabled = !g.ok;
  $('rc-goto-send').title = g.ok ? `send one goal on ${(st.path || {}).topic}`
                                 : g.why;
  $('rc-goto-fill').disabled = !pose;
  if (!g.ok) {
    $('rc-goto-note').textContent = g.why;
    return;
  }
  const vel = (st.config || {}).max_vel;
  const eta = vel ? `, about ${Math.round(g.d / vel)} s at ${vel} m/s` : '';
  const over = (g.effZ !== null && g.effZ !== undefined && g.effZ !== g.z)
    ? ` — z will be forced to ${g.effZ} m by fsm/click_height` : '';
  $('rc-goto-note').textContent =
    `= PX4 NED ${g.x.toFixed(2)}, ${(-g.y).toFixed(2)}, ${(-g.z).toFixed(2)}`
    + ` · ${g.d.toFixed(2)} m away${eta}`
    + (g.yaw === null ? ' · heading held' : ` · turning to ${g.yaw}°`)
    + over;
}

$('rc-goto-fill').onclick = () => {
  const pose = RC && RC.pose;
  if (!pose) return;
  $('rc-gx').value = pose.x.toFixed(2);
  $('rc-gy').value = pose.y.toFixed(2);
  $('rc-gz').value = pose.z.toFixed(2);
  $('rc-gyaw').value = (pose.yaw * 180 / Math.PI).toFixed(0);
  if (RC) renderRC(RC);
};

$('rc-goto-send').onclick = async () => {
  if (!RC) return;
  const g = rcGotoRead(RC);
  if (!g.ok) { say(g.why, 'err'); return; }
  const L = rcLimits();
  // A nudge is one step; this is the whole distance in one press. Above the
  // confirm threshold it gets the same treatment as STOP ALL.
  if (g.d > L.goto_confirm
      && !confirm(`Fly ${g.d.toFixed(1)} m to [${g.x}, ${g.y}, ${g.z}] `
                  + `in lidar_init?

SUPER plans and avoids on the way, and `
                  + `the abort is the RC's channel-11 switch.`)) return;
  const q = `x=${g.x}&y=${g.y}&z=${g.z}`
          + (g.yaw === null ? '' : `&yaw=${g.yaw}`);
  await act('/api/rc/goto?' + q, $('rc-goto-send'));
  rcTick();
};

for (const id of ['rc-gx', 'rc-gy', 'rc-gz', 'rc-gyaw']) {
  // Recompute the echo locally on every keystroke: the distance, the ETA and
  // the PX4 equivalent must be visible BEFORE the goal is sent, not after.
  $(id).oninput = () => { if (RC) rcGoto(RC); };
  $(id).onkeydown = (e) => { if (e.key === 'Enter') $('rc-goto-send').onclick(); };
}

/* ------------------------------------------------------- typed entry */
/* Out of range is clamped and SAID, never silently ignored: these are metres
   the aircraft flies, and a box that quietly disagrees with what it shows is
   the same class of bug as a button that does nothing. */

function rcCommitTyped(id, lo, hi, round, apply) {
  const inp = $(id);
  const raw = Number((inp.value || '').trim());
  if (!isFinite(raw)) {
    inp.classList.add('bad');
    say(`${id === 'rc-yaw-input' ? 'Turn' : 'Step'} must be a number`, 'err');
    return;
  }
  inp.classList.remove('bad');
  const v = Math.round(Math.min(hi, Math.max(lo, raw)) * round) / round;
  if (Math.abs(v - raw) > 1e-9) say(`clamped to ${v} (allowed ${lo}–${hi})`, 'warn');
  apply(v);
  inp.value = v;
  rcSteps();
  if (RC) renderRC(RC);
}

$('rc-step-input').onchange = () => {
  const L = rcLimits();
  rcCommitTyped('rc-step-input', L.step_min, L.step_max, 100,
                (v) => { RC_STEP = v; });
};
$('rc-yaw-input').onchange = () => {
  const L = rcLimits();
  rcCommitTyped('rc-yaw-input', L.yaw_min, L.yaw_max, 1,
                (v) => { RC_YAW_STEP = v; });
};
for (const id of ['rc-step-input', 'rc-yaw-input']) {
  $(id).onkeydown = (e) => { if (e.key === 'Enter') $(id).blur(); };
}

async function rcSendAltitude(z) {
  // One call site for the slider and the box, so they cannot drift apart.
  RC_SLIDER_HELD = false;
  await act(`/api/rc/altitude?z=${encodeURIComponent(Number(z).toFixed(2))}`, null);
  rcTick();
}

$('rc-alt-go').onclick = () => {
  const v = Number(($('rc-alt-input').value || '').trim());
  if (!isFinite(v)) { say('Altitude must be a number, in metres', 'err'); return; }
  rcSendAltitude(v);
};
$('rc-alt-input').onkeydown = (e) => {
  if (e.key === 'Enter') $('rc-alt-go').onclick();
};

function rcSteps() {
  const lin = $('rc-steps');
  lin.textContent = '';
  for (const v of (RC && RC.limits ? RC.limits.steps : [0.25, 0.5, 1, 2, 3])) {
    const b = el('button', 'chip' + (v === RC_STEP ? ' pick' : ''), v + ' m');
    b.onclick = () => { RC_STEP = v; rcSteps(); if (RC) renderRC(RC); };
    lin.appendChild(b);
  }
  const yaw = $('rc-yaw-steps');
  yaw.textContent = '';
  for (const v of (RC && RC.limits ? RC.limits.yaw_steps : [15, 45, 90])) {
    const b = el('button', 'chip' + (v === RC_YAW_STEP ? ' pick' : ''), v + '°');
    b.onclick = () => { RC_YAW_STEP = v; rcSteps(); if (RC) renderRC(RC); };
    yaw.appendChild(b);
  }
}

/* -------------------------------------------------------- Jetson power */
/* The only buttons on this page that stop the COMPUTER rather than a program.
   On this project that computer is bolted to an aircraft, so there are three
   gates and none of them is the others' fallback: these two questions, the
   ?confirm=<mode> the server insists on, and host_power_blocked() refusing
   while the FC reports ARMED or IN AIR. */

let GOING_DOWN = null;       // 'poweroff' | 'reboot', once one is accepted
let BAG_RECORDING = false;   // so the first question can mention it

function powerQuestions(mode, host, recording) {
  // Worth naming: rosbag2 writes metadata.yaml on SHUTDOWN only, so a
  // recording that is killed instead of stopped is unplayable. The GCS's own
  // teardown SIGINTs it first -- say so, rather than leaving the pilot to
  // guess whether the last ten minutes survived.
  const bag = recording
    ? '\n\nA bag recording is running. It gets SIGINT first and is finalised, '
      + 'so it stays playable.'
    : '';
  if (mode === 'reboot') {
    return [
      `Are you sure you want to restart (reboot) the Jetson (${host})?\n\n`
      + 'Everything running on it stops — the SUPER stack, both PX4 bridges, '
      + 'this dashboard. The box comes back on its own and the dashboard with '
      + 'it, but the stack does NOT restart itself: super-gcs.service is '
      + 'dashboard-only, deliberately.' + bag,
      `Second confirmation.\n\nReally reboot ${host} now? This page goes dead `
      + 'until it is back.',
    ];
  }
  return [
    `Are you sure you want to turn OFF the Jetson (${host})?\n\n`
    + 'This shuts down the computer this dashboard runs on — the SUPER stack, '
    + 'both PX4 bridges and this page all stop with it. Nothing here can '
    + 'switch it back on: someone has to reach the aircraft and power-cycle '
    + 'it.' + bag,
    `Second confirmation.\n\nReally power off ${host} now?`,
  ];
}

async function hostPower(mode) {
  const host = (CONFIG && CONFIG.host) || 'this Jetson';
  const qs = powerQuestions(mode, host, BAG_RECORDING);
  // Asked TWICE on purpose. Once is one misplaced tap on a phone, and the
  // second question names the host so it has to be read rather than clicked
  // through -- this dashboard is often open next to another machine's.
  if (!confirm(qs[0])) return;
  if (!confirm(qs[1])) return;
  const btn = $(mode === 'reboot' ? 'host-reboot' : 'host-poweroff');
  btn.disabled = true;
  try {
    const r = await api(`/api/host/${mode}?confirm=${mode}`, {method: 'POST'});
    GOING_DOWN = mode;
    say(r.message, 'ok');
  } catch (e) {
    say(String(e.message || e), 'err');
  }
  tick();
}

$('host-reboot').onclick = () => hostPower('reboot');
$('host-poweroff').onclick = () => hostPower('poweroff');

function renderPower(p) {
  const note = $('host-power-note');
  const btns = [$('host-reboot'), $('host-poweroff')];
  if (!p) {
    // Same trap as the camera and recording panels: the static files are
    // re-read per request but server.py is not, so a GCS started before this
    // panel existed serves the new page and a state with no `power` key.
    for (const b of btns) b.disabled = true;
    note.textContent = STALE_SERVER.replace('camera data', 'power data')
      .replace('the camera panel', 'this panel');
    return;
  }
  if (p.pending) {
    GOING_DOWN = p.pending;
    for (const b of btns) b.disabled = true;
    note.textContent = (p.pending === 'reboot' ? 'Rebooting' : 'Powering off')
      + (p.in_s ? ` in ${p.in_s}s` : ' now') + ' — this dashboard is about to '
      + 'go away.';
    return;
  }
  if (GOING_DOWN) {
    // A poll succeeded with nothing pending: either the Jetson came back from
    // a reboot, or systemd never took the job and hostpower said why.
    const was = GOING_DOWN;
    GOING_DOWN = null;
    if (p.error) say(p.error, 'err');
    else if (was === 'reboot') say('the Jetson is back', 'ok');
  }
  const why = p.blocked;
  for (const b of btns) b.disabled = !!why;
  // Say it out loud. A dashboard whose power buttons quietly do nothing
  // because someone left GCS_POWER_DRY_RUN set is worse than one without them.
  const dry = p.dry_run ? 'DRY RUN (GCS_POWER_DRY_RUN is set): these do not '
                        + 'actually power anything down. ' : '';
  note.textContent = dry + (p.error ? `last attempt failed: ${p.error}`
    : why ? `unavailable: ${why}`
    : 'Stops this computer, not the FC. Each button asks twice.');
}

$('restart').onclick = () => {
  if (confirm('Restart everything that is currently running?\n\n'
              + 'It goes down, DDS settles, then the same set comes back.')) {
    act('/api/restart', $('restart'));
  }
};

/* -------------------------------------------------------------- log pane */

let log_key = 'stack';
let log_seq = 0;

function buildTabs() {
  const box = $('log-tabs');
  box.textContent = '';
  const keys = [...Object.keys(CONFIG.procs), 'bag', 'check'];
  const label = {check: 'checks', bag: 'recording'};
  for (const k of keys) {
    const t = el('div', 'tab' + (k === log_key ? ' sel' : ''), label[k] || k);
    t.onclick = () => { log_key = k; log_seq = 0; $('log').textContent = '';
                        buildTabs(); pollLog(); };
    box.appendChild(t);
  }
}

async function pollLog() {
  if (document.hidden) return;
  try {
    const r = await api(`/api/logs/${log_key}?since=${log_seq}`);
    if (r.lines && r.lines.length) {
      const pre = $('log');
      const stick = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 40;
      pre.textContent += r.lines.join('\n') + '\n';
      if (stick) pre.scrollTop = pre.scrollHeight;
      log_seq = r.seq;
    }
  } catch (e) { /* the state poll already reports the link */ }
}

/* ---------------------------------------------------------- RViz (laptop) */
/* All same-origin: the page asks the Jetson, and the laptop helper picks the
   command up on its next poll. Nothing here talks to 127.0.0.1, because a page
   on a private address is not allowed to (and a phone has no laptop loopback). */

function renderRviz(r) {
  const state = $('rviz-state');
  const note = $('rviz-note');
  const cmdrow = $('cmdrow');
  if (!r || !r.helper) {
    $('rviz-start').disabled = true;
    $('rviz-stop').disabled = true;
    state.textContent = 'helper not running';
    state.className = 'pill warn';
    note.textContent = 'RViz is a native app, so this page cannot launch it by '
      + 'itself. Run this on the laptop you want RViz to open on, then these '
      + 'buttons light up:';
    cmdrow.hidden = false;
    return;
  }
  $('rviz-start').disabled = !!r.running || r.pending === 'start';
  $('rviz-stop').disabled = !r.running || r.pending === 'stop';
  state.textContent = r.pending ? `${r.pending}ing…`
    : r.running ? `running · pid ${r.pid}` : 'not running';
  state.className = 'pill ' + (r.running ? 'ok' : r.pending ? 'info' : 'muted');
  note.textContent = `Helper connected from ${r.host || 'your laptop'}`
    + (r.note ? ` — last action: ${r.note}` : '')
    + '. It runs scripts/laptop_viewer.sh there.';
  cmdrow.hidden = true;
}

$('rviz-start').onclick = () => act('/api/rviz/start', $('rviz-start'));
$('rviz-stop').onclick = () => act('/api/rviz/stop', $('rviz-stop'));
$('copy-cmd').onclick = () => copy($('viewer-cmd').textContent, 'command');

/* ----------------------------------------------------------------- camera */
/* The feed is an <img> pointed at /api/camera/stream, which is MJPEG -- the
   camera's own JPEG frames forwarded untouched. No player, no websocket, no
   decode anywhere; browsers have rendered this shape since 2004 and it is the
   only kind of video a stdlib http.server with no build step could serve.

   Attaching the <img> is what OPENS the stream and detaching is what closes
   it, so this file is careful about when it does either: a hidden tab, a
   stopped camera and a "Hide feed" click all have to actually free the
   socket, or the Jetson keeps pushing 40 Mbit/s at nobody. */

let CAM_WANT = false;        // the user has asked to see the feed
let CAM_ON = false;          // the <img> is attached, i.e. a stream is open
let CAM_RETRY = 0;           // do not reconnect a failing stream in a loop

function camAttach() {
  if (CAM_ON) return;
  // The token goes in the URL because an <img> cannot send a header. Date.now()
  // defeats the cache: without it a reconnect can be served the dead response.
  // Both credentials in the URL, for the same reason: an <img> cannot send a
  // header. It is the one place the camera password's *session* is exposed
  // that way -- never the password itself, which only ever goes in a POST body.
  $('cam-img').src = `/api/camera/stream?k=${encodeURIComponent(TOKEN)}`
                   + `&a=${encodeURIComponent(ADMIN)}&t=${Date.now()}`;
  $('cam-wrap').hidden = false;
  CAM_ON = true;
}

function camDetach() {
  if (!CAM_ON) { $('cam-wrap').hidden = true; return; }
  // Clearing src is what closes the connection. Hiding the element does not:
  // the browser keeps reading the multipart body, and the server keeps a
  // viewer registered, which keeps the ROS subscription alive.
  $('cam-img').src = '';
  $('cam-wrap').hidden = true;
  CAM_ON = false;
}

$('cam-img').onerror = () => {
  // A stream ends when the camera stops, when three viewers are already
  // watching, or when the Wi-Fi drops. Back off and let the poll decide
  // whether there is anything to reconnect to.
  CAM_ON = false;
  CAM_RETRY = Date.now() + 4000;
  $('cam-wrap').hidden = true;
};

/* The source picker. Starting the driver and watching an existing publisher
   are two different actions, and this is the second one: no process is
   touched, the server just subscribes somewhere else. The open stream follows
   the switch without reconnecting -- the next frame simply comes from the new
   topic. */
function renderSources(c) {
  const box = $('cam-sources');
  const rows = (c && c.sources) || [];
  box.hidden = rows.length === 0;
  box.textContent = '';
  if (!rows.length) return;

  // The selected chip is marked, not disabled: a greyed-out chip is how this
  // page shows "you cannot have this", and the one you are already watching is
  // the opposite of that. Re-clicking it is a harmless no-op.
  const chip = (label, on, title, topic) => {
    const b = el('button', 'chip' + (on ? ' sel' : ''), label);
    b.title = title;
    b.onclick = () => act(`/api/camera/source?topic=${encodeURIComponent(topic)}`,
                          null);
    return b;
  };
  box.appendChild(chip('Auto', !c.pinned,
                       'Ours first, then whatever else is publishing', ''));
  for (const r of rows) {
    box.appendChild(chip(r.topic + (r.ours ? ' · ours' : ''),
                         c.pinned === r.topic,
                         r.ours ? 'Published by the camera this dashboard started'
                                : 'Published by another process — watching it '
                                  + 'starts nothing and stops nothing',
                         r.topic));
  }
  // A pin whose publisher went away is not an error, but the panel would
  // otherwise silently show a different camera than the one you asked for.
  if (c.pinned && !c.pinned_live) {
    box.appendChild(el('span', 'pill warn', `${c.pinned} is not publishing`));
  }
}

/* The static files are read from disk on every request but server.py is not:
   a GCS left running from before this panel existed serves the NEW page and an
   /api/state with no `camera` key at all. renderCamera is only ever called from
   a SUCCESSFUL poll, so a missing `camera` means exactly that -- and saying
   "checking…" for ever is the one thing this page must not do. */
const STALE_SERVER =
  'This page is newer than the GCS process serving it: /api/state carries no '
  + 'camera data, so the server was started before the camera panel existed. '
  + 'Restart it — sudo systemctl restart super-gcs (or Ctrl-C and re-run '
  + 'bash scripts/gcs.sh) — then reload. It stops nothing it did not start.';

function camReason(c, row) {
  if (!c) return STALE_SERVER;
  if (c.error) return c.error;
  if (!(c.devices || []).length) {
    return 'No Orbbec camera on USB. Check the cable and the USB 3 port, then '
         + 'run: python3 scripts/gcs/camera.py';
  }
  const d = c.devices[0];
  const dev = `${d.name} (${d.id}, serial ${d.serial || 'unknown'})`;
  if (c.source && row && row.state !== 'stopped') {
    return `${dev} — started from here, publishing ${c.source}.`;
  }
  if (c.source) {
    // The case this panel exists for: somebody else's launch already owns the
    // camera. Watching it is fine -- and is a first-class action, not a
    // consolation prize; starting a second driver is not.
    return `${dev} — already owned by another process, publishing ${c.source}. `
         + 'Show feed watches it (subscribing starts and stops nothing). Start '
         + 'is refused: a Gemini can only be opened once, and a second reader '
         + 'dies with uvc_open -6.';
  }
  const h = (c.holders || [])[0];
  if (h) return `${dev} — USB device held by pid ${h.pid}: ${h.cmd}`;
  return `${dev} — free. Start publishes /gemini/color/image_raw/compressed `
       + '(640×480 at 15 fps, MJPEG straight off the camera).';
}

function renderCamera(c, procs, busy) {
  // Locked: the server sent no camera data at all, so there is nothing to
  // render and the card is hidden by renderAdmin().
  if (!ADMIN_ON) return;
  const row = (procs || []).find((p) => p.key === 'camera') || null;
  const live = !!(c && c.source);
  const pill = $('cam-state');

  $('cam-start').disabled = !!busy || !c || !!(row && row.start_blocked)
                            || (row ? row.state !== 'stopped' : true);
  $('cam-stop').disabled = !!busy || !row || row.state === 'stopped';
  $('cam-view').disabled = !live;
  $('cam-view').textContent = CAM_WANT ? 'Hide feed' : 'Show feed';

  pill.textContent = !c ? 'server too old'
    : c.error ? 'no ROS'
    : live ? (row && row.state !== 'stopped' ? 'running · ours' : 'running · not ours')
    : (c.devices || []).length ? 'stopped' : 'no camera';
  pill.className = 'pill ' + (live ? 'ok'
    : (c && (c.devices || []).length) ? 'muted' : 'warn');
  $('cam-note').textContent = camReason(c, row);
  renderSources(c);

  if (live && CAM_WANT && !CAM_ON && !document.hidden && Date.now() > CAM_RETRY) {
    camAttach();
  } else if (!live && CAM_ON) {
    camDetach();          // the source went away; do not sit on a dead socket
  }

  if (CAM_ON && c) {
    const size = c.width ? `${c.width}×${c.height}` : 'unknown size';
    $('cam-meta').textContent =
      `${size} · ${c.fps || 0} fps at the source · ${c.kb || '?'} KB/frame · `
      + `${c.viewers} of ${c.max_streams} viewers · browser capped to `
      + `${c.default_fps} fps · ${c.source}`;
  }
}

$('cam-start').onclick = async () => {
  CAM_WANT = true;          // pressing Start means "and show me the picture"
  CAM_RETRY = 0;
  await act('/api/proc/camera/start', $('cam-start'));
};
$('cam-stop').onclick = () => {
  camDetach();
  act('/api/proc/camera/stop', $('cam-stop'));
};
$('cam-view').onclick = () => {
  CAM_WANT = !CAM_WANT;
  CAM_RETRY = 0;
  if (!CAM_WANT) camDetach();
  tick();
};

/* ---------------------------------------------------------------- recording */
/* `ros2 bag record` into bags/, either everything or a chosen set of topics.

   Two rules shape this panel:

   1. The topic list is fetched ON DEMAND, never from the 2 Hz poll. Listing
      topics means one graph query per topic -- ~100 on this aircraft, 65 of
      them /fmu/* -- and paying that twice a second to render chips nobody is
      looking at is exactly the mistake telemetry.py's header warns about.

   2. Nothing here re-renders the chips from tick(). A picker that rebuilt
      itself twice a second would fight the scroll position and lose the
      selection mid-click; the poll only ever updates the state line. */

let BAG_MODE = 'all';           // 'all' | 'pick'
const BAG_PICK = new Set();     // topics the user chose
let BAG_TOPICS = [];            // last /api/bag/topics answer
let BAG_TOPICS_ERR = null;
let BAG_FILTER = '';
let BAG_LIST = null;            // last /api/bag/list answer
let BAG_WAS_RECORDING = false;  // to refresh the list on the falling edge

function bagShown() {
  const f = BAG_FILTER.trim().toLowerCase();
  return f ? BAG_TOPICS.filter((t) => t.topic.toLowerCase().includes(f))
           : BAG_TOPICS;
}

async function bagLoadTopics() {
  try {
    const r = await api('/api/bag/topics');
    BAG_TOPICS = r.topics || [];
    BAG_TOPICS_ERR = r.ok ? null : (r.error || 'cannot list topics');
  } catch (e) {
    BAG_TOPICS = [];
    BAG_TOPICS_ERR = String(e.message || e);
  }
  // A topic that stopped publishing cannot be recorded, so drop it from the
  // selection rather than sending a name the server would refuse.
  const live = new Set(BAG_TOPICS.map((t) => t.topic));
  for (const t of [...BAG_PICK]) if (!live.has(t)) BAG_PICK.delete(t);
  renderBagTopics();
}

async function bagLoadList() {
  try {
    BAG_LIST = await api('/api/bag/list');
  } catch (e) {
    BAG_LIST = null;
  }
  renderBagList();
}

function renderBagTopics() {
  const box = $('bag-topics');
  box.textContent = '';
  if (BAG_TOPICS_ERR) {
    box.appendChild(el('span', 'pill warn', BAG_TOPICS_ERR));
    return;
  }
  const rows = bagShown();
  if (!rows.length) {
    box.appendChild(el('span', 'pill muted',
      BAG_TOPICS.length ? 'nothing matches that filter'
                        : 'no topic has a publisher — is the stack running?'));
    return;
  }
  for (const t of rows) {
    const on = BAG_PICK.has(t.topic);
    // 'heavy' is a warning, not a refusal: the point cloud is often exactly
    // what you came to record. It just costs a few hundred MB a minute.
    const b = el('button', 'chip' + (t.kind === 'heavy' || t.kind === 'image'
      ? ' heavy' : '') + (on ? ' pick' : ''));
    b.append(el('span', null, t.topic),
             el('span', 'ty', t.type.split('/').pop()
                              + (t.publishers > 1 ? ` ×${t.publishers}` : '')));
    b.title = t.kind ? `${t.type} — large; a few hundred MB a minute`
                     : t.type;
    b.onclick = () => {
      if (BAG_PICK.has(t.topic)) BAG_PICK.delete(t.topic);
      else BAG_PICK.add(t.topic);
      renderBagTopics();
      bagSelNote();
    };
    box.appendChild(b);
  }
  bagSelNote();
}

function bagSelNote() {
  $('bag-sel').textContent = BAG_MODE === 'all'
    ? (ADMIN_ON ? 'every topic with a publisher, including new ones'
       // Says that something is held back without naming it -- the same rule
       // the admin gate's own text follows.
       : 'every topic with a publisher, except restricted ones')
    : `${BAG_PICK.size} of ${BAG_TOPICS.length} selected`;
}

function bagMode(mode) {
  BAG_MODE = mode;
  $('bag-mode-all').className = 'chip' + (mode === 'all' ? ' sel' : '');
  $('bag-mode-pick').className = 'chip' + (mode === 'pick' ? ' sel' : '');
  $('bag-picker').hidden = mode !== 'pick';
  if (mode === 'pick' && !BAG_TOPICS.length && !BAG_TOPICS_ERR) bagLoadTopics();
  else bagSelNote();
}

function renderBagList() {
  const box = $('bag-list');
  box.textContent = '';
  const rows = (BAG_LIST && BAG_LIST.bags) || [];
  if (!rows.length) {
    box.appendChild(el('div', 'note',
      BAG_LIST ? `Nothing recorded yet. They land in ${BAG_LIST.dir}/ on the `
                 + 'Jetson, which is gitignored.'
               : 'Could not read the recordings directory.'));
    return;
  }
  for (const b of rows) {
    const row = el('div', 'bagrow'
      + (b.recording ? ' live' : b.finalized ? '' : ' broken'));
    row.appendChild(el('div', 'n', b.dir));
    const bits = [`${b.size_mb} MB`];
    if (b.topics) bits.push(`${b.topics} topics`);
    if (b.duration_s) bits.push(fmt.dur(Math.round(b.duration_s)));
    if (b.messages) bits.push(`${b.messages} msgs`);
    row.appendChild(el('div', 'm', bits.join(' · ')));
    // The distinction that matters when you get home: metadata.yaml is written
    // at shutdown, and without it `ros2 bag play` cannot read a single message.
    row.appendChild(el('span', 'pill ' + (b.recording ? 'ok'
      : b.finalized ? 'muted' : 'bad'),
      b.recording ? 'recording' : b.finalized ? 'playable' : 'NO metadata.yaml'));
    box.appendChild(row);
  }
}

function renderBag(b, busy) {
  if (!b) {
    // Same trap as the camera panel: the static files are re-read from disk on
    // every request but server.py is not, so a GCS left running from before
    // this panel existed serves the NEW page and a state with no `bag` key.
    // Saying "checking…" for ever is the one thing this page must not do.
    $('bag-state').textContent = 'server too old';
    $('bag-state').className = 'pill warn';
    $('bag-note').textContent = STALE_SERVER.replace('camera data', 'bag data')
      .replace('the camera panel', 'this panel');
    $('bag-start').disabled = true;
    $('bag-stop').disabled = true;
    return;
  }
  const rec = BAG_RECORDING = !!b.recording;
  $('bag-state').textContent = rec ? 'recording' : 'idle';
  $('bag-state').className = 'pill ' + (rec ? 'ok' : 'muted');
  $('bag-start').disabled = !!busy || rec || !!b.start_blocked;
  $('bag-start').title = b.start_blocked || '';
  $('bag-stop').disabled = !rec;

  if (rec) {
    $('bag-note').textContent =
      `${b.dir}/${b.name} — ${b.size_mb} MB in ${b.files} file(s), `
      + `${b.rate_mbs} MB/s, ${fmt.dur(b.elapsed_s)} elapsed. `
      + `${b.free_gb} GB free. Stop finalises the bag; it is `
      + 'not playable until then.';
  } else if (b.start_blocked) {
    $('bag-note').textContent = `unavailable: ${b.start_blocked}`;
  } else {
    $('bag-note').textContent =
      // Measured, not guessed: 87 MB in 5.4 s of `record all` on this
      // aircraft, and that run had the LiDAR down -- the number a pilot needs
      // before leaving a bag running through a flight.
      `${b.free_gb} GB free. "All topics" runs close to a gigabyte a minute `
      + 'here — /livox/lidar, /cloud_registered and the depth cloud carry '
      + `whole point clouds. Below ${b.min_free_gb} GB a running recording `
      + 'stops itself, so the bag stays playable.';
  }
  // The list is only worth re-reading when it can have changed: at boot, and
  // when a recording ends. Sizing every bag in bags/ on every poll is a
  // directory walk per bag, twice a second, for a panel that rarely changes.
  if (BAG_WAS_RECORDING && !rec) bagLoadList();
  BAG_WAS_RECORDING = rec;
}

$('bag-mode-all').onclick = () => bagMode('all');
$('bag-mode-pick').onclick = () => bagMode('pick');
$('bag-refresh').onclick = () => bagLoadTopics();
$('bag-filter').oninput = () => {
  BAG_FILTER = $('bag-filter').value || '';
  renderBagTopics();
};
$('bag-all-shown').onclick = () => {
  for (const t of bagShown()) BAG_PICK.add(t.topic);
  renderBagTopics();
};
$('bag-none').onclick = () => { BAG_PICK.clear(); renderBagTopics(); };

$('bag-start').onclick = async () => {
  const all = BAG_MODE === 'all';
  if (!all && !BAG_PICK.size) {
    say('pick at least one topic, or choose "All topics"', 'err');
    return;
  }
  $('bag-start').disabled = true;
  try {
    const r = await api('/api/bag/start', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({all, topics: [...BAG_PICK],
                            label: $('bag-label').value || ''}),
    });
    say(r.message, 'ok');
    // The recorder prints the topics it actually subscribed to, which is the
    // only place a typo'd selection or a QoS mismatch shows up.
    log_key = 'bag'; log_seq = 0; $('log').textContent = ''; buildTabs();
    bagLoadList();
  } catch (e) {
    say(String(e.message || e), 'err');
  }
  tick();
};

$('bag-stop').onclick = async () => {
  $('bag-stop').disabled = true;
  try {
    const r = await api('/api/bag/stop', {method: 'POST'});
    say(r.message, 'ok');
  } catch (e) {
    say(String(e.message || e), 'err');
  }
  bagLoadList();
  tick();
};

/* ----------------------------------------------------------- bookmark URLs */
/* The Wi-Fi IP is DHCP and changes between sessions, so the name-based URLs
   are the ones worth keeping. Rendered once at boot from /api/config: the
   hostname does not change while the server is up (renaming it restarts this
   service -- scripts/set_hostname.sh). */

function copy(text, what) {
  // navigator.clipboard is a SECURE-CONTEXT api: on plain http:// it is simply
  // undefined everywhere except localhost. This dashboard is served over plain
  // http on a LAN name, so that is the normal case here, not the exception --
  // without the execCommand fallback every copy button on the page is dead for
  // exactly the people who need it (a phone at a field). Keep both: the old
  // path is deprecated but universally implemented, the new one is the only
  // one that will still exist in a few years.
  const done = () => say(`${what} copied`, 'ok');
  const failed = () => say('copy failed — select it by hand', 'err');
  if (navigator.clipboard && globalThis.isSecureContext) {
    navigator.clipboard.writeText(text).then(done).catch(failed);
    return;
  }
  try {
    const ta = document.createElement('textarea');
    ta.value = text;
    // Off-screen rather than hidden: a display:none element cannot be selected.
    ta.style.position = 'fixed';
    ta.style.top = '-1000px';
    document.body.appendChild(ta);
    ta.focus();
    ta.select();
    const ok = document.execCommand('copy');
    document.body.removeChild(ta);
    ok ? done() : failed();
  } catch (e) {
    failed();
  }
}

function renderUrls(urls) {
  const box = $('urls');
  box.textContent = '';
  if (!urls || !urls.length) return;
  for (const u of urls) {
    const row = el('div', 'urlrow');
    const code = el('code', null, u.url);
    const btn = el('button', 'ghost small', 'copy');
    btn.onclick = () => copy(u.url, u.name);
    row.append(code, btn, el('span', 'why', u.note));
    box.appendChild(row);
  }
}

/* ------------------------------------------------------------------ loop */

async function boot() {
  CONFIG = await api('/api/config');
  $('host').textContent = `${CONFIG.host} · ${CONFIG.ips.join(' ')}`;
  $('viewer-cmd').textContent = CONFIG.helper_cmd;
  renderUrls(CONFIG.urls);
  buildTabs();
  bagMode(BAG_MODE);
  // boot() runs again after an admin unlock, and what a locked client was
  // given is not the same list -- refetch rather than leave the picker showing
  // the redacted one.
  if (BAG_MODE === 'pick' && BAG_TOPICS.length) bagLoadTopics();
  bagLoadList();

  buildFlight();
  const box = $('checks');
  for (const [key, spec] of Object.entries(CONFIG.checks)) {
    const b = el('button', 'chip' + (spec.destructive ? ' warn' : ''), spec.label);
    b.onclick = async () => {
      // The only check that changes anything on the aircraft. It archives to
      // firmware/faults/ and verifies the copy before deleting, but a hardfault
      // log is the only evidence the FC reset itself -- so ask.
      if (spec.destructive && !confirm(
            `${spec.label}\n\nEach dump is copied into firmware/faults/ and `
            + 'verified before it is removed from the SD card. Continue?')) return;
      log_key = 'check'; log_seq = 0; $('log').textContent = ''; buildTabs();
      await act(`/api/check/${key}`, null);
    };
    box.appendChild(b);
  }
}

/* ------------------------------------------------------------ flight day */
// Built once from CONFIG.flight; the verdict pills and the buttons' enabled
// state follow every poll. A step's button is the SAME action as elsewhere on
// the page (a row's Start with its options, a check chip, the recorder), so
// every refusal and interlock the server has applies unchanged.
const FLIGHT_EL = {};          // step id -> {pill, btn, box}

function flightStepButton(st) {
  if (st.kind === 'check') {
    const b = el('button', 'small', 'Run');
    b.onclick = async () => {
      log_key = 'check'; log_seq = 0; $('log').textContent = ''; buildTabs();
      await act(`/api/check/${st.key}`, null);
    };
    return b;
  }
  if (st.kind === 'proc') {
    const stop = st.action === 'stop';
    const b = el('button', 'small' + (stop ? ' ghost' : ''), stop ? 'Stop' : 'Start');
    if (st.key === 'dtp' && !stop && st.opts && st.opts.mode === 'live') {
      b.textContent = 'Start LIVE'; b.className = 'small caution';
    }
    b.onclick = () => {
      const row = (FLIGHT_PROCS || []).find((p) => p.key === st.key) || {};
      if (st.key === 'dtp') {
        if (stop) return dtpStop(row, b);
        DTP_MODE = (st.opts || {}).mode || 'observe';
        return dtpStart(b);
      }
      if (stop) return act(`/api/proc/${st.key}/stop`, b);
      if (st.key === 'stack') {
        STACK_CFG = (st.opts || {}).config || STACK_CFG;
        return act(`/api/proc/stack/start?config=${encodeURIComponent(STACK_CFG)}`, b);
      }
      return act(`/api/proc/${st.key}/start`, b);
    };
    return b;
  }
  if (st.kind === 'follow') {
    const b = el('button', 'small caution', 'Engage FOLLOW');
    b.onclick = () => act('/api/dtp/follow?set=on', b);
    return b;
  }
  if (st.kind === 'bag' && st.action === 'stop') {
    // The Recording card's Stop, nothing else: SIGINT, so metadata.yaml is written.
    const b = el('button', 'small ghost', 'Stop');
    b.onclick = async () => {
      b.disabled = true;
      try {
        const r = await api('/api/bag/stop', {method: 'POST'});
        say(r.message || 'recording stopped', 'ok');
        bagLoadList();
      } catch (e) { say(e.message, 'err'); }
    };
    return b;
  }
  if (st.kind === 'bag') {
    const b = el('button', 'small', 'Record');
    b.onclick = async () => {
      b.disabled = true;
      try {
        const r = await api('/api/bag/start', {
          method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({preset: st.preset})});
        say(r.message || 'recording', 'ok');
      } catch (e) { say(e.message, 'err'); }
    };
    return b;
  }
  return null;
}

function buildFlight() {
  const box = $('flight-steps');
  box.textContent = '';
  const f = CONFIG.flight;
  if (!f || !f.steps) { $('flight-card').hidden = true; return; }
  for (const ph of f.phases) {
    const h = el('div', 'phase', ph.title);
    if (ph.note) h.appendChild(el('small', null, ph.note));
    box.appendChild(h);
    for (const st of f.steps.filter((x) => x.phase === ph.id)) {
      const row = el('div', 'step');
      // Numbered in renderFlight(): the count depends on the mode.
      const num = el('span', 'n', '');
      row.appendChild(num);
      const t = el('div', 'title', st.title);
      if (st.detail) t.appendChild(el('small', null, st.detail));
      row.appendChild(t);
      const pill = el('span', 'pill muted', '—');
      row.appendChild(pill);
      const acts = el('div', 'acts');
      const btn = flightStepButton(st);
      if (btn) acts.appendChild(btn);
      row.appendChild(acts);
      box.appendChild(row);
      FLIGHT_EL[st.id] = {pill, btn, box: row, num, st};
    }
  }
}

/* Observe only / Live, for the whole card. It is the SAME choice as the DTP
   row's (DTP_MODE), so the two can never disagree, and like it it comes back to
   observe on every page load. While a follower runs the card shows the mode it
   is really in, read off its processes, and the chips are fixed. */
function flightMode(rows) {
  const p = rows.dtp || {};
  if (p.state && p.state !== 'stopped' && (p.mode === 'observe' || p.mode === 'live')) {
    return {mode: p.mode, fixed: true};
  }
  return {mode: DTP_MODE, fixed: false};
}

/* The chips are built ONCE and updated in place. Rebuilt on every 500 ms poll,
   the chip under the pointer was often replaced between mousedown and mouseup,
   and the browser then drops the click: "Live" would not switch (2026-09-21). */
function renderFlightMode(fm, busy) {
  const box = $('flight-mode');
  if (!box) return;
  const modes = ['observe', 'live'];
  if (box.children.length !== modes.length + 1) {
    box.textContent = '';
    for (const m of modes) {
      box.appendChild(optChip(m === 'observe' ? 'Observe only' : 'Live', false,
                              () => { DTP_MODE = m; tick(); }, DTP_MODE_TEXT[m]));
    }
    box.appendChild(el('span', 'note', ''));
  }
  modes.forEach((m, i) => {
    const c = box.children[i];
    c.className = 'chip' + (fm.mode === m ? ' sel' : '') + (m === 'live' ? ' warn' : '');
    c.disabled = !!busy || fm.fixed;
  });
  box.children[modes.length].textContent = fm.fixed
    ? `a follower is running ${fm.mode.toUpperCase()} — stop it to change mode`
    : DTP_MODE_TEXT[fm.mode];
}

let FLIGHT_PROCS = null;
function renderFlight(s) {
  if (!CONFIG || !CONFIG.flight) return;
  FLIGHT_PROCS = s.procs;
  const busy = !!s.check_running || !!s.busy;
  const rows = {};
  for (const p of s.procs || []) rows[p.key] = p;
  const presets = CONFIG.bag_presets || {};
  const hhmm = (t) => new Date(t * 1000).toTimeString().slice(0, 5);
  const fm = flightMode(rows);
  renderFlightMode(fm, busy);
  let n = 0;
  for (const [id, e] of Object.entries(FLIGHT_EL)) {
    const st = e.st;
    e.box.className = 'step';
    e.box.hidden = !!st.modes && !st.modes.includes(fm.mode);
    if (e.box.hidden) continue;
    n += 1;
    e.num.textContent = String(n);
    if (st.kind === 'check') {
      const r = (s.checks || {})[st.key];
      const spec = (CONFIG.checks || {})[st.key] || {};
      const running = s.check_running === spec.label;
      e.pill.className = 'pill ' + (running ? 'info' : !r ? 'muted' : r.ok ? 'ok' : 'bad');
      e.pill.textContent = running ? 'running…' : !r ? 'not run'
        : (r.ok ? 'PASS' : 'FAIL') + ` ${hhmm(r.t)} · ${r.detail}`;
      if (r && !running) e.box.className += r.ok ? ' done' : ' failed';
      if (e.btn) e.btn.disabled = busy;
    } else if (st.kind === 'proc') {
      const p = rows[st.key] || {state: 'stopped'};
      const stop = st.action === 'stop';
      const cls = {running: 'ok', external: 'warn', stopped: 'muted'}[p.state] || 'muted';
      let txt = p.state === 'external' ? 'running · not ours' : p.state;
      if (st.key === 'dtp' && p.state !== 'stopped' && p.mode) txt += ` · ${p.mode}`;
      e.pill.className = 'pill ' + cls;
      e.pill.textContent = txt;
      const want = (st.opts || {}).mode;
      const up = p.state !== 'stopped';
      if (!stop && up && (!want || p.mode === want)) e.box.className += ' done';
      if (e.btn) {
        if (stop) {
          e.btn.disabled = busy || !up || !!p.locked;
          e.btn.title = p.locked ? 'Needs Admin control' : '';
        } else {
          const why = st.key === 'dtp' ? (p.blocked || {})[want || 'observe'] : p.start_blocked;
          e.btn.disabled = busy || up || !!why || !!p.locked;
          e.btn.title = p.locked ? 'Needs Admin control' : (why || '');
        }
      }
    } else if (st.kind === 'follow') {
      const p = rows.dtp || {};
      const f = p.follow;
      const state = f && f.lock ? f.lock.split('|')[0] : null;
      e.pill.className = 'pill ' + (state ? (LOCK_CLS[state] || 'muted') : 'muted');
      e.pill.textContent = state ? f.lock.split('|').slice(0, 1).concat(f.lock.split('|').slice(2, 3)).join(' · ').slice(0, 60)
                                 : (p.state === 'stopped' ? 'no follower' : 'lock ?');
      if (state === 'LOCKED') e.box.className += ' done';
      if (e.btn) {
        const why = (p.follow_blocked || {}).on;
        e.btn.disabled = busy || !!why || !!p.locked || f === undefined;
        e.btn.title = p.locked ? 'Needs Admin control' : (why || '');
        e.btn.textContent = state && state !== 'IDLE' ? 'Re-engage FOLLOW' : 'Engage FOLLOW';
      }
    } else if (st.kind === 'bag' && st.action === 'stop') {
      const rec = s.bag && s.bag.recording;
      e.pill.className = 'pill ' + (rec ? 'warn' : 'muted');
      e.pill.textContent = rec ? 'recording' : 'idle';
      if (e.btn) e.btn.disabled = busy || !rec;
    } else if (st.kind === 'bag') {
      const rec = s.bag && s.bag.recording;
      e.pill.className = 'pill ' + (rec ? 'ok' : 'muted');
      e.pill.textContent = rec ? 'recording' : 'idle';
      const pre = presets[st.preset] || {};
      const needAdmin = pre.images && !ADMIN_ON;
      if (e.btn) {
        e.btn.disabled = busy || !!rec || needAdmin;
        e.btn.title = needAdmin ? 'This preset records camera frames — unlock Admin control' : '';
      }
      if (rec) e.box.className += ' done';
    }
  }
}

async function tick() {
  // A tab you switched away from should cost nothing. Polling is the only
  // thing that keeps the Jetson's 50 Hz vehicle_local_position subscription
  // alive (telemetry.py tears it down after IDLE_AFTER), so skipping the poll
  // takes the GCS from ~21% of a core back to ~1.5%. A forgotten background
  // tab used to cost exactly as much as the one you were looking at.
  if (document.hidden) return;
  try {
    const s = await api('/api/state');
    last_ok = Date.now();
    $('link-dot').className = 'dot on';
    renderAircraft(s.ros);
    renderBattery(s.ros);
    renderEstimator(s.ros);
    renderHost(s.host);
    PROC_AIRCRAFT = s.ros && s.ros.available ? s.ros.aircraft : null;
    renderProcs(s.procs, s.busy);
    renderRates(s.ros);
    renderRviz(s.rviz);
    renderAdmin(s.admin);
    renderCamera(s.camera, s.procs, s.busy);
    renderBag(s.bag, s.busy);
    renderFlight(s);
    // After renderBag: the first shutdown question mentions a live recording.
    renderPower(s.power);
    const blocked = !!s.check_running || !!s.busy;
    for (const b of $('checks').children) b.disabled = blocked;
    // The server refuses a restart while a follower runs (supervisor.restart),
    // so the page does not offer one either.
    const follower = s.procs.find((p) => p.key === 'dtp' && p.state !== 'stopped');
    $('restart').disabled = blocked || !!follower;
    $('restart').title = follower
      ? 'Stop the DTP follower first — Restart never brings it back on its own'
      : '';
    $('stop-all').disabled = !!s.busy;
    // Only offer the sweep when there is actually something to sweep -- an
    // enabled button whose only possible answer is "nothing is running outside
    // this GUI" is just a way to make the page look broken.
    const strays = s.procs.filter((p) => p.state === 'external');
    $('stop-external').disabled = blocked || !strays.length;
    $('stop-external').title = strays.length
      ? `${strays.map((p) => p.key).join(', ')} started outside this dashboard`
      : 'nothing is running outside this dashboard';
    renderReboot(blocked);
    $('busy-hint').textContent = s.busy ? `— ${s.busy} in progress…` : '';
    // A restart that stopped things and could not bring them back is the worst
    // outcome this page can produce; it must not be silent.
    if (s.error && s.error !== last_error) { say(s.error, 'err'); }
    last_error = s.error || null;
  } catch (e) {
    $('link-dot').className = 'dot off';
    if (e instanceof AuthError) {
      // Not a lost link -- the server is answering, it just will not talk to
      // us. Saying "lost the Jetson" here would send you debugging Wi-Fi.
      $('host').textContent = 'locked — token needed';
      if ($('tokengate').hidden) {
        showGate('The token stopped being accepted. Paste the current one '
                 + 'from ~/.cache/super-gcs/token.');
      }
      return;
    }
    if (GOING_DOWN) {
      // Not a lost link: we asked for this. "lost the Jetson" here would send
      // someone hunting field Wi-Fi for a shutdown they ordered themselves.
      $('host').textContent = GOING_DOWN === 'reboot'
        ? 'rebooting — waiting for the Jetson to come back…'
        : 'powered off — switch the Jetson on again to use this dashboard';
      return;
    }
    if (Date.now() - last_ok > 4000) {
      $('host').textContent = 'lost the Jetson — retrying…';
    }
  }
}

// The timers start regardless of whether boot() succeeded: with a bad token
// they simply keep 401ing behind the gate, and the moment a good one is pasted
// the page fills in without a reload.
setInterval(tick, POLL_MS);
setInterval(pollLog, 1000);
// rcTick returns immediately unless the card is unlocked AND on screen, so
// this costs one comparison a fifth of a second while it is not.
setInterval(rcTick, RC_POLL_MS);
rcWatch();
document.addEventListener('visibilitychange', () => {
  // Come back instantly rather than waiting out the next interval.
  if (document.hidden) {
    // The video is the expensive one: a background tab left streaming keeps
    // the Jetson's image subscription alive and 40 Mbit/s on the Wi-Fi. The
    // poll skipping itself is not enough -- the socket has to go.
    camDetach();
    return;
  }
  tick();
  pollLog();
});

boot()
  .then(() => { tick(); pollLog(); })
  .catch((e) => {
    if (e instanceof AuthError) {
      showGate(TOKEN
        ? 'The token this browser remembered was rejected — it has been '
          + 'forgotten. Paste the current one.'
        : 'This URL has no ?k=… token, and this browser has not seen one yet.');
    } else {
      say('cannot reach the GCS API: ' + e.message, 'err');
    }
  });
