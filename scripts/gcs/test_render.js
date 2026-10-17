// Render test for the GCS front end -- run it with gjs, no browser needed.
//
//   python3 scripts/gcs/test_render.py          # convenience wrapper
//   gjs scripts/gcs/test_render.js <state.json> <config.json>
//
// It stubs just enough DOM and fetch to execute app.js's REAL render path
// against a REAL /api/state snapshot, then asserts on what came out. A parser
// check ("does it parse") cannot catch a renamed field or a typo'd property;
// this can, and there is no browser on the Jetson to catch it any other way.
const GLib = imports.gi.GLib;

const argv = ARGV;
const read = (f) => {
  const [ok, bytes] = GLib.file_get_contents(f);
  if (!ok) throw new Error('cannot read ' + f);
  return new TextDecoder().decode(bytes);
};

const HERE = GLib.path_get_dirname(
  GLib.find_program_in_path('gjs') && argv[2] ? argv[2] : '.');
const STATIC = argv[2] || 'scripts/gcs/static';
const STATE = JSON.parse(read(argv[0]));
const CFG = JSON.parse(read(argv[1]));
// The RC card renders from its own endpoint, so it is a fourth input -- passed
// only when test_render.py had an admin session to fetch it with.
const RCS = argv[3] ? JSON.parse(read(argv[3])) : null;

/* --- minimal DOM ------------------------------------------------------- */
function mkNode(tag) {
  const classes = [];
  return {
    tagName: tag, children: [], _text: '', className: '', style: {},
    hidden: false, disabled: false, title: '', scrollTop: 0,
    clientHeight: 0, scrollHeight: 0, onclick: null,
    // The typed boxes: a number input has a value and bounds, and app.js
    // marks a bad one with a class. Without these the RC code would throw
    // here and every assertion after it would never run.
    value: '', min: '', max: '', placeholder: '',
    classList: {
      add: (c) => { if (!classes.includes(c)) classes.push(c); },
      remove: (c) => { const i = classes.indexOf(c); if (i >= 0) classes.splice(i, 1); },
      contains: (c) => classes.includes(c),
    },
    blur() {},
    appendChild(c) { this.children.push(c); return c; },
    append(...cs) { cs.forEach((c) => this.children.push(c)); },
    after() {},
    get textContent() { return this._text; },
    set textContent(v) { this._text = String(v); if (v === '') this.children = []; },
  };
}
/* Every element that carries an id, with the `hidden` ATTRIBUTE honoured.
   The stub used to default them all to visible, so any assertion about
   something that starts hidden in the markup silently tested nothing -- and
   the fix must not be "have render() hide it", because a panel the user just
   opened would then be slammed shut by the next 2 Hz poll. */
const IDS = {};
for (const tag of read(STATIC + '/index.html')
                    .match(/<[a-zA-Z][^>]*id="[^"]+"[^>]*>/g) || []) {
  const node = mkNode('div');
  node.hidden = /\shidden(\s|>|=)/.test(tag);
  IDS[tag.match(/id="([^"]+)"/)[1]] = node;
}
globalThis.document = {
  getElementById: (id) => IDS[id] || (IDS[id] = mkNode('div')),
  createElement: mkNode,
  hidden: false,                      // app.js skips polling when hidden
  addEventListener() {},
  activeElement: null,                // nothing focused: values are refreshed
};
globalThis.location = { search: '?k=test' };
globalThis.confirm = () => false;
globalThis.navigator = {};
globalThis.setInterval = () => 0;
globalThis.setTimeout = () => 0;
globalThis.clearTimeout = () => {};
globalThis.URLSearchParams = function () { this.get = () => 'test'; };
globalThis.fetch = (url) => Promise.resolve({
  ok: true,
  json: () => Promise.resolve(
    url.includes('/api/rc') ? (RCS || {})
    : url.includes('/api/config') ? CFG
    : url.includes('/api/state') ? STATE
    : url.includes('/api/logs') ? { lines: ['log line'], seq: 1 }
    : {}),
});

/* --- run app.js -------------------------------------------------------- */
let fail = 0;
const check = (name, cond, detail) => {
  print(`  ${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  -- ' + detail : ''}`);
  if (!cond) fail++;
};

try {
  new Function(read(STATIC + '/app.js'))();
} catch (e) {
  print('  FAIL  app.js threw: ' + e.message + '\n' + (e.stack || ''));
  fail++;
}
const loop = GLib.MainLoop.new(null, false);
GLib.timeout_add(GLib.PRIORITY_DEFAULT, 400, () => { loop.quit(); return false; });
loop.run();

/* --- assertions -------------------------------------------------------- */
const txt = (id) => IDS[id]._text;
const kids = (id) => IDS[id].children.map((c) => c._text);
const ros = STATE.ros || {};

print('render test');
// Both states are legitimate: /fmu/out/vehicle_status_v1 is 2 Hz, so the first
// poll after the server starts genuinely has nothing yet. Assert the right
// thing for whichever state the snapshot is in, and never accept a blank.
const ac = ros.aircraft || {};
if (ac.fresh) {
  check('flight mode rendered', txt('mode') === ac.mode,
        `got ${JSON.stringify(txt('mode'))}`);
  check('mode is not a raw enum number', !/^\d+$/.test(txt('mode')));
} else {
  check('stale telemetry shows a dash, not a blank or a stale mode',
        txt('mode') === '\u2014', JSON.stringify(txt('mode')));
  check('stale telemetry is explained', txt('mode-sub').length > 0,
        txt('mode-sub'));
}
check('host line rendered', txt('host').includes(CFG.host));
// Rows come from /api/state and tabs from /api/config, and they are no longer
// the same list: the follower's row is shown to a locked client while its log
// tab is not.
check('one row per process in the state',
      IDS['procs'].children.length === (STATE.procs || []).length);
check('a rate chip per rate topic',
      IDS['rates'].children.length === Object.keys(ros.rates || {}).length);
check('a button per check',
      IDS['checks'].children.length === Object.keys(CFG.checks).length);
check('the destructive check is marked in the UI',
      IDS['checks'].children.some((b) => b.className.includes('warn')),
      'clearing crash dumps must not look like every other button');
check('restart button present and gated on state',
      typeof IDS['restart'].onclick === 'function');
// The Flight day card: one header per phase, one row per step, a button on
// every step, and only the picked mode's steps showing, numbered 1..n.
if (CFG.flight) {
  const F = CFG.flight;
  check('flight day: a header per phase and a row per step',
        IDS['flight-steps'].children.length === F.phases.length + F.steps.length,
        `${IDS['flight-steps'].children.length} vs ${F.phases.length} + ${F.steps.length}`);
  const rows = IDS['flight-steps'].children.filter((c) => c.className.includes('step'));
  const withBtn = rows.filter((r) => r.children[3] && r.children[3].children.length);
  check('flight day: every step is a button -- the card carries no terminal or pilot rows',
        withBtn.length === F.steps.length
        && F.steps.every((s) => ['check', 'proc', 'bag', 'follow'].includes(s.kind)),
        `${withBtn.length} vs ${F.steps.length}`);
  check('flight day: every check step names a real check',
        F.steps.filter((s) => s.kind === 'check').every((s) => s.key in CFG.checks));
  check('flight day: every recording step names a real preset, or is the stop',
        F.steps.filter((s) => s.kind === 'bag')
               .every((s) => s.action === 'stop' || s.preset in (CFG.bag_presets || {})));
  check('flight day: a verdict pill on every row',
        rows.every((r) => r.children[2] && r.children[2].className.includes('pill')));
  const live = rows.find((r) => r.children[3].children[0]
                                && r.children[3].children[0].textContent === 'Start LIVE');
  check('flight day: the LIVE follower start is amber, like the row\'s',
        !!live && live.children[3].children[0].className.includes('caution'));

  // The mode. A running follower fixes it; otherwise a fresh page is observe.
  const dtpRow = (STATE.procs || []).find((p) => p.key === 'dtp') || {};
  const running = dtpRow.state && dtpRow.state !== 'stopped'
                  && ['observe', 'live'].includes(dtpRow.mode);
  const mode = running ? dtpRow.mode : 'observe';
  const chips = IDS['flight-mode'].children.filter((c) => c.className.includes('chip'));
  check('flight day: two mode chips, the right one selected',
        chips.length === 2
        && chips[0].className.includes('sel') === (mode === 'observe')
        && chips[1].className.includes('sel') === (mode === 'live'),
        chips.map((c) => c.className).join(' | '));
  check('flight day: the mode chips are fixed while a follower runs',
        chips.every((c) => c.disabled === (!!running || !!STATE.busy || !!STATE.check_running)));
  const shown = (st) => !st.modes || st.modes.includes(mode);
  check('flight day: only the picked mode\'s steps are showing',
        F.steps.every((st, i) => rows[i].hidden === !shown(st)),
        rows.map((r) => (r.hidden ? 'h' : 'v')).join(''));
  const nums = rows.filter((r) => !r.hidden).map((r) => r.children[0].textContent);
  check('flight day: the showing steps are numbered 1..n with no gap',
        nums.every((t, i) => t === String(i + 1)), nums.join(','));
  for (const m of ['observe', 'live']) {
    const kinds = F.steps.filter((st) => !st.modes || st.modes.includes(m))
                         .map((st) => st.kind + (st.action ? ':' + st.action : ''));
    check(`flight day: the ${m} path starts a follower, records, preflights and engages`,
          ['proc', 'bag', 'check', 'follow', 'bag:stop', 'proc:stop'].every((k) => kinds.includes(k))
          && F.steps.some((st) => st.key === 'dtp' && (st.opts || {}).mode === m
                                 && (st.modes || []).includes(m)),
          kinds.join(' '));
  }
}
check('busy hint reflects state.busy',
      txt('busy-hint') === (STATE.busy ? `— ${STATE.busy} in progress…` : ''),
      JSON.stringify(txt('busy-hint')));
check('log tabs = processes + recording + checks',
      IDS['log-tabs'].children.length === Object.keys(CFG.procs).length + 2);

/* The two buttons that reach past "software only". Both are interlocked in the
   server as well; these assert the page never OFFERS an action the server would
   refuse, because a button that fails when pressed reads as a broken page. */
const blocked = !!STATE.check_running || !!STATE.busy;
const strays = (STATE.procs || []).filter((p) => p.state === 'external');
const canReboot = !blocked && !!ac.fresh && !ac.armed && ac.landed !== false;
check('FC reboot offered only when disarmed, landed and linked',
      IDS['fc-reboot'].disabled === !canReboot,
      `armed=${ac.armed} landed=${ac.landed} fresh=${ac.fresh} busy=${blocked}`);
check('FC reboot says WHY it is unavailable, rather than just greying out',
      canReboot || txt('fc-reboot-note').startsWith('unavailable'),
      JSON.stringify(txt('fc-reboot-note')));
check('stop-outside-nodes offered only when something is outside',
      IDS['stop-external'].disabled === (blocked || strays.length === 0),
      `${strays.length} external`);
check('stop-outside-nodes explains itself either way',
      IDS['stop-external'].title.length > 0, IDS['stop-external'].title);
// A row started from a terminal used to be unstoppable from here; the server
// now adopts it, so the button must not still be greyed out.
// Not a locked row: those are greyed out for a different reason, asserted below.
const extRow = STATE.procs.findIndex((p) => p.state === 'external' && !p.locked);
if (extRow >= 0 && !blocked) {
  const acts = IDS['procs'].children[extRow].children[3];
  check('Stop is live for a process started outside the GUI',
        acts.children[1].disabled === false);
}

/* The Stack row's config and the DTP follower. The follower is the one row that
   can end with something commanding the aircraft, so these assert the page
   never offers a start the server would refuse, that a live start is asked
   about and carries confirm=live, and that its reason is on screen rather than
   in a tooltip a phone cannot show. */
const spin = (ms) => {
  const l = GLib.MainLoop.new(null, false);
  GLib.timeout_add(GLib.PRIORITY_DEFAULT, ms, () => { l.quit(); return false; });
  l.run();
};
const rowOf = (key) => {
  const i = (STATE.procs || []).findIndex((p) => p.key === key);
  return i < 0 ? null : IDS['procs'].children[i];
};
const capturePosts = (fn) => {
  const seen = [];
  const realFetch = globalThis.fetch;
  globalThis.fetch = (url, opts) => {
    if ((opts || {}).method === 'POST') seen.push(url);
    return realFetch(url, opts);
  };
  try { fn(); } finally { globalThis.fetch = realFetch; }
  return seen;
};
const busyNow = !!STATE.busy;

const stackRow = (STATE.procs || []).find((p) => p.key === 'stack') || null;
if (stackRow && 'config' in stackRow) {
  const extra = rowOf('stack').children[4];
  if (stackRow.state === 'stopped') {
    const chips = extra.children.filter((c) => c.tagName === 'button');
    check('a stopped stack offers one chip per config',
          chips.length === Object.keys(CFG.stack_configs || {}).length,
          `${chips.length} chips`);
    check('...with click selected by default',
          chips.length > 0 && chips[0]._text === 'click' && chips[0].className.includes('sel'));
    const posts = capturePosts(() => rowOf('stack').children[3].children[0].onclick());
    check('Start carries the chosen config',
          posts[0] === '/api/proc/stack/start?config=click', JSON.stringify(posts));
    spin(150);
  } else {
    check('a running stack says which config fsm_node is on',
          extra.children.length > 0
          && extra.children[0]._text.includes(stackRow.config || stackRow.config_error
                                              || 'not up yet'),
          JSON.stringify(extra.children.map((c) => c._text)));
  }
}

const dtpRow = (STATE.procs || []).find((p) => p.key === 'dtp') || null;
if (dtpRow) {
  let node = rowOf('dtp');
  check('the follower row has its mode line',
        !!node.children[4] && node.children[4].className === 'extra');
  check('Restart is not offered while a follower runs',
        dtpRow.state === 'stopped' || IDS['restart'].disabled === true,
        `state=${dtpRow.state}`);
  if (dtpRow.locked) {
    check('a locked client cannot press the follower\'s Start or Stop',
          node.children[3].children[0].disabled === true
          && node.children[3].children[1].disabled === true);
    check('...and is told it needs Admin control',
          /Admin control/.test(node.children[3].children[0].title));
    check('...and was sent no reasons, which name the camera',
          !dtpRow.blocked && !dtpRow.stop_note);
  } else if (dtpRow.state === 'stopped') {
    const chips = node.children[4].children.filter((c) => c.tagName === 'button');
    check('two mode chips, OBSERVE selected on page load',
          chips.length === 2 && chips[0].className.includes('sel')
          && !chips[1].className.includes('sel'));
    const obs = (dtpRow.blocked || {}).observe;
    check('Start offered exactly when the server would start observe',
          node.children[3].children[0].disabled === !!(obs || busyNow),
          `blocked=${JSON.stringify(obs)}`);
    check('...and a refusal is on screen, not only in a tooltip',
          !obs || node.children[4].children.some((c) => c._text.includes(obs)));
    let posts = capturePosts(() => node.children[3].children[0].onclick());
    check('an observe Start says observe, and asks for no confirm token',
          posts[0] === '/api/proc/dtp/start?mode=observe', JSON.stringify(posts));
    spin(150);

    rowOf('dtp').children[4].children.filter((c) => c.tagName === 'button')[1].onclick();
    spin(250);
    node = rowOf('dtp');
    const live = (dtpRow.blocked || {}).live;
    check('choosing Live relabels Start, so it cannot be mistaken',
          node.children[3].children[0]._text === 'Start LIVE',
          JSON.stringify(node.children[3].children[0]._text));
    check('Start LIVE offered exactly when the server would start it',
          node.children[3].children[0].disabled === !!(live || busyNow),
          `blocked=${JSON.stringify(live)}`);
    globalThis.confirm = () => false;
    posts = capturePosts(() => node.children[3].children[0].onclick());
    check('declining the live question sends nothing', posts.length === 0,
          JSON.stringify(posts));
    globalThis.confirm = () => true;
    posts = capturePosts(() => node.children[3].children[0].onclick());
    check('...and accepting it POSTs with confirm=live',
          posts[0] === '/api/proc/dtp/start?mode=live&confirm=live', JSON.stringify(posts));
    globalThis.confirm = () => false;
    spin(150);
    // Back to observe, so nothing after this renders against a changed page.
    rowOf('dtp').children[4].children.filter((c) => c.tagName === 'button')[0].onclick();
    spin(250);
  } else {
    check('a running follower names its mode in its pill',
          !dtpRow.mode || node.children[1]._text.includes(dtpRow.mode),
          JSON.stringify(node.children[1]._text));
    globalThis.confirm = () => false;
    const posts = capturePosts(() => node.children[3].children[1].onclick());
    check('stopping the follower asks first', posts.length === 0, JSON.stringify(posts));
  }
}

/* The remote control. Locked is the default shape and the one that matters:
   the assertion is not "the card is hidden" but that no RC data reached this
   client at all, and that the page did not wire up a control it cannot use. */
if (!(STATE.admin || {}).unlocked) {
  check('RC card hidden while locked', IDS['rc-card'].hidden === true);
  check('NO rc data reaches a locked client', !STATE.rc,
        '/api/rc is a separate endpoint and must 403, not redact');
}
check('every pad button exists and is wired',
      ['forward', 'back', 'left', 'right', 'up', 'down', 'yaw_left', 'yaw_right']
        .every((a) => typeof IDS['rc-b-' + a].onclick === 'function'));
check('the typed boxes are wired, not decoration',
      ['rc-step-input', 'rc-yaw-input', 'rc-alt-input']
        .every((i) => typeof IDS[i].onkeydown === 'function')
      && typeof IDS['rc-step-input'].onchange === 'function'
      && typeof IDS['rc-alt-go'].onclick === 'function');
check('the coordinate bar is wired on every field and both buttons',
      ['rc-gx', 'rc-gy', 'rc-gz', 'rc-gyaw']
        .every((i) => typeof IDS[i].oninput === 'function'
                      && typeof IDS[i].onkeydown === 'function')
      && typeof IDS['rc-goto-send'].onclick === 'function'
      && typeof IDS['rc-goto-fill'].onclick === 'function');
check('the abort instruction names the RC, not a button on this page',
      /channel-11/i.test(read(STATIC + '/index.html')),
      'SUPER has no stop input; implying one here would be the dangerous bit');
check('there is no STOP button pretending otherwise',
      !/id="rc-stop"/.test(read(STATIC + '/index.html')));

/* With a live /api/rc (an admin session was available), render it for real:
   this is the only check that renderRC() survives actual data rather than the
   empty object the fetch stub hands it. */
if (RCS) {
  check('RC card shown when unlocked', IDS['rc-card'].hidden === false);
  check('the readiness row has one chip per failure mode',
        IDS['rc-ready'].children.length === (RCS.ready || []).length,
        `${(RCS.ready || []).length} rows`);
  const capAlt = (RCS.capabilities || {}).altitude || {ok: true};
  check('up/down offered exactly when the config would honour the z',
        IDS['rc-b-up'].disabled === !(capAlt.ok && RCS.pose),
        `click_height honoured=${capAlt.ok} pose=${!!RCS.pose}`);
  check('...and a disabled one says why in its tooltip',
        IDS['rc-b-up'].disabled === false || IDS['rc-b-up'].title.length > 10,
        JSON.stringify(IDS['rc-b-up'].title));
  check('the altitude slider is gated with the up/down buttons',
        IDS['rc-alt-slider'].disabled === IDS['rc-b-up'].disabled);
  if ((RCS.config || {}).altitude) {
    check('the tape is labelled with the EFFECTIVE band, not the configured one',
          txt('rc-tape-max') === RCS.config.altitude.max + ' m'
          && Number(RCS.config.altitude.max) < 3.5,
          `${txt('rc-tape-min')} .. ${txt('rc-tape-max')}`);
  }
  check('the note always says something', txt('rc-note').length > 0,
        txt('rc-note'));

  /* The goal path. Which of SUPER's two inputs is live decides whether the
     altitude controls do anything at all, so it is rendered, not implied. */
  const gp = RCS.path || {};
  check('the card says which goal input it is using',
        txt('rc-path').length > 10 && /follow|click|refusing/i.test(txt('rc-path')),
        txt('rc-path'));
  check('altitude is offered exactly when the live path keeps the z',
        IDS['rc-alt-input'].disabled === !(gp.keeps_z && RCS.pose),
        `keeps_z=${gp.keeps_z} pose=${!!RCS.pose}`);
  check('the altitude box and the slider are gated together',
        IDS['rc-alt-input'].disabled === IDS['rc-alt-slider'].disabled);

  /* The typed boxes take their bounds from the server, so a limit cannot be
     tightened in rc.py and stay loose in the browser. */
  const L = RCS.limits || {};
  check('the step box advertises the server bounds',
        Number(IDS['rc-step-input'].min) === L.step_min
        && Number(IDS['rc-step-input'].max) === L.step_max,
        `${IDS['rc-step-input'].min}..${IDS['rc-step-input'].max}`);
  check('the turn box advertises the server bounds',
        Number(IDS['rc-yaw-input'].min) === L.yaw_min
        && Number(IDS['rc-yaw-input'].max) === L.yaw_max,
        `${IDS['rc-yaw-input'].min}..${IDS['rc-yaw-input'].max}`);

  /* Did the last goal happen? The block is hidden until one has been sent,
     and its class carries the outcome -- the two states that must not read as
     "moving" are coloured, not just worded. */
  check('the progress block is hidden until a goal has been sent',
        IDS['rc-prog'].hidden === !RCS.progress,
        `progress=${!!RCS.progress}`);
  if (RCS.progress) {
    check('...and shows the state SUPER/PX4 imply',
          IDS['rc-prog'].className.indexOf(RCS.progress.state) >= 0,
          IDS['rc-prog'].className);
    check('...with a sentence, not just a word',
          txt('rc-prog-detail').length > 15, txt('rc-prog-detail'));
    check('...and never claims to be moving while nothing is flying',
          RCS.progress.state !== 'moving'
          || ((RCS.aircraft || {}).armed && (RCS.aircraft || {}).offboard),
          JSON.stringify(RCS.aircraft));
  }

  /* The coordinate bar. Send must be dead before it is pressed whenever the
     server would refuse -- and with the fields empty it always would. */
  check('Send goal is dead with the coordinate fields empty',
        IDS['rc-goto-send'].disabled === true
        && IDS['rc-goto-send'].title.length > 10,
        IDS['rc-goto-send'].title);
  if (RCS.pose) {
    check('the current position is shown in BOTH frames',
          /lidar_init/.test(txt('rc-goto-now')) && /PX4 NED/.test(txt('rc-goto-now')),
          JSON.stringify(txt('rc-goto-now')));
    // The whole point of the echo: PX4 reports the same place with y and z
    // negated, and a number copied off QGC must look wrong BEFORE it is sent.
    const ned = RCS.pose_ned || {};
    check('...and the PX4 echo negates y and z',
          Math.abs(ned.y + RCS.pose.y) < 1e-6 && Math.abs(ned.z + RCS.pose.z) < 1e-6
          && Math.abs(ned.x - RCS.pose.x) < 1e-6,
          JSON.stringify(ned));
    // Fill, then re-render: the bar must now be able to describe a real goal.
    IDS['rc-goto-fill'].onclick();
    check('Use current fills all four fields',
          ['rc-gx', 'rc-gy', 'rc-gz', 'rc-gyaw']
            .every((i) => String(IDS[i].value).length > 0),
          ['rc-gx', 'rc-gy', 'rc-gz', 'rc-gyaw'].map((i) => IDS[i].value).join(' '));
    // Filled with the CURRENT position, the goal is 0 m away -- which on the
    // click path is exactly the silent-drop case, so the note has to warn.
    check('a filled bar describes the goal before it is sent',
          txt('rc-goto-note').length > 10, txt('rc-goto-note'));
  }
}

/* The Jetson's OWN power buttons. Same rule as Reboot FC -- never offer what
   the server would refuse -- plus the thing that makes them different from
   every other button on this page: they ask TWICE. */
const pw = STATE.power;
check('the state carries a power block', !!pw,
      'a GCS older than this panel serves the new page with no power data');
if (pw) {
  for (const id of ['host-reboot', 'host-poweroff']) {
    check(`${id} is offered exactly when the server would allow it`,
          IDS[id].disabled === !!pw.blocked, `blocked=${pw.blocked}`);
  }
  check('the power note says WHY, rather than just greying out',
        !pw.blocked || txt('host-power-note').includes(pw.blocked),
        JSON.stringify(txt('host-power-note')));
  check('a dry run is admitted on the page, not just in the API',
        !pw.dry_run || txt('host-power-note').includes('DRY RUN'),
        JSON.stringify(txt('host-power-note')));
}

/* One confirm() is one misplaced tap on a phone, and an endpoint reachable
   without the token is one stale tab away from a shutdown. Both halves are
   asserted here because neither is visible from the other side: the server
   test cannot see how many times the page asked. */
let asked = 0;
let posted = null;
const realFetch = globalThis.fetch;
globalThis.fetch = (url, opts) => {
  if ((opts || {}).method === 'POST') posted = url;
  return realFetch(url, opts);
};
globalThis.confirm = () => { asked++; return true; };
IDS['host-poweroff'].onclick();
check('powering the Jetson off asks twice', asked === 2, `asked ${asked} time(s)`);
check('...and POSTs only with an explicit confirm token',
      posted === '/api/host/poweroff?confirm=poweroff', String(posted));
asked = 0; posted = null;
globalThis.confirm = () => { asked++; return asked > 1; };   // decline the first
IDS['host-reboot'].onclick();
check('declining the first question stops it there, with no POST',
      asked === 1 && posted === null, `asked ${asked}, posted ${posted}`);
globalThis.fetch = realFetch;
globalThis.confirm = () => false;

if ((ros.battery || {}).present) {
  check('battery voltage rendered', /^\d+\.\d+ V$/.test(txt('volt')),
        JSON.stringify(txt('volt')));
  check('current labelled uncalibrated',
        kids('batt-kv').some((t) => t.includes('uncal')));
} else {
  check('absent battery explained, not shown as 0 V',
        txt('volt') === '—' && txt('batt-sub').length > 0, txt('batt-sub'));
}

check('swap is on the host panel', kids('host-kv').includes('Swap'));

/* The bookmark links. An IP-only dashboard is one DHCP lease away from being
   unreachable, so assert these are name-based and carry the token -- a URL
   without ?k= only ever says "cannot reach the GCS API". */
check('a bookmark row per stable URL',
      IDS['urls'].children.length === (CFG.urls || []).length,
      `${(CFG.urls || []).length} offered`);
check('at least an mDNS .local URL is offered',
      (CFG.urls || []).some((u) => u.name.endsWith('.local')),
      JSON.stringify((CFG.urls || []).map((u) => u.name)));
check('bookmark URLs are name-based, never a bare IP',
      (CFG.urls || []).every((u) => !/^https?:\/\/\d+\.\d+\.\d+\.\d+/.test(u.url)),
      JSON.stringify((CFG.urls || []).map((u) => u.url)));
check('bookmark URLs carry the token',
      (CFG.urls || []).every((u) => u.url.includes('?k=')));

/* The camera panel, behind Admin control.

   Locked is the DEFAULT shape and the one that matters: the assertion is not
   "the panel is hidden" -- a hidden panel whose data arrived anyway is
   theatre -- but that no camera data reached this client at all. */
const adm = STATE.admin || {};
if (!adm.unlocked) {
  check('camera panel hidden while locked', IDS['cam-card'].hidden === true);
  check('NO camera data reaches a locked client', !STATE.camera,
        'hiding a panel whose contents the page already has is not a gate');
  check('no camera row in the Stack card while locked',
        !(STATE.procs || []).some((p) => p.key === 'camera'));
  check('no camera log tab while locked',
        !Object.keys(CFG.procs).includes('camera'));
  check('the header offers the unlock', txt('admin-btn') === 'Admin control',
        JSON.stringify(txt('admin-btn')));
  check('the gate is not in your face until you ask for it',
        IDS['admingate'].hidden === true);
  /* The gate is rendered for a client that has unlocked nothing, so its own
     text must not give away what is behind it. Both halves are checked: the
     static markup as served, and the message app.js writes when the button is
     pressed -- the second only appears if you actually press it, so asserting
     on the untouched element would pass on an empty string and prove nothing. */
  const gateHtml = (read(STATIC + '/index.html')
    .match(/<section id="admingate"[\s\S]*?<\/section>/) || [''])[0];
  check('the gate markup names no feature',
        gateHtml.length > 0 && !/camera|gemini/i.test(gateHtml),
        (gateHtml.match(/camera|gemini/ig) || []).join(','));
  IDS['admin-btn'].onclick();
  check('pressing Admin control opens the gate',
        IDS['admingate'].hidden === false);
  check('...and what it then says names no feature',
        txt('admin-why').length > 0
          && !/camera|gemini/i.test(txt('admin-why')),
        JSON.stringify(txt('admin-why')));
} else {
  check('camera panel shown when unlocked', IDS['cam-card'].hidden === false);
  check('the header offers the re-lock', txt('admin-btn') === 'Lock',
        JSON.stringify(txt('admin-btn')));
  cameraChecks();
}

/* The failure these exist to prevent: a Gemini can be opened by exactly ONE
   process, and on this Jetson it is regularly already owned by another launch
   under /camera. Offering Start there would kill somebody else's perception
   stack with a `uvc_open -6` in the middle. */
function cameraChecks() {
const cam = STATE.camera || {};
const camRow = (STATE.procs || []).find((p) => p.key === 'camera') || null;
const camFree = !!camRow && camRow.state === 'stopped' && !camRow.start_blocked;
check('camera panel always says something, never a blank',
      txt('cam-note').length > 0, JSON.stringify(txt('cam-note')));
check('Start offered only when the camera is actually free',
      IDS['cam-start'].disabled === !(camFree && !blocked),
      `state=${camRow && camRow.state} blocked=${JSON.stringify(
        camRow && camRow.start_blocked)}`);
check('Show feed offered only when something is publishing',
      IDS['cam-view'].disabled === !cam.source, `source=${cam.source}`);
if (camRow && camRow.start_blocked) {
  // The row in the Stack card must agree with the panel -- it is the same
  // single-user device, and one of the two saying "Start" is a trap.
  const acts = IDS['procs'].children[STATE.procs.indexOf(camRow)].children[3];
  check('the Stack row for the camera agrees that Start is refused',
        acts.children[0].disabled === true);
  check('the row explains why, on hover',
        acts.children[0].title === camRow.start_blocked);
}
if (cam.source && !(camRow && camRow.state !== 'stopped')) {
  check('a camera owned by another process names it, and says why not to start one',
        txt('cam-note').includes(cam.source) && /uvc_open/.test(txt('cam-note')),
        txt('cam-note'));
}
/* Watching an already-published topic is a separate action from starting the
   driver -- on this Jetson the camera is usually already owned by something
   else, so it has to be reachable, not a fallback. */
check('a source chip per publisher, plus Auto',
      IDS['cam-sources'].children.length >= (cam.sources || []).length
        + ((cam.sources || []).length ? 1 : 0),
      `${(cam.sources || []).length} sources`);
check('the source chips appear exactly when something is publishing',
      IDS['cam-sources'].hidden === !(cam.sources || []).length);
if ((cam.sources || []).length) {
  const sel = IDS['cam-sources'].children.filter(
    (c) => c.className && c.className.includes('sel'));
  check('exactly one source chip is marked as the one being watched',
        sel.length === 1, `${sel.length} marked`);
  check('...and it is not greyed out, which on this page means "unavailable"',
        sel[0].disabled === false);
  check('every source chip can be clicked to watch that topic',
        IDS['cam-sources'].children.every(
          (c) => c.tagName !== 'button' || typeof c.onclick === 'function'));
}
if ((cam.devices || []).length) {
  check('the camera panel names the hardware it found',
        txt('cam-note').includes(cam.devices[0].id), txt('cam-note'));
}
}

/* The recording panel. Its two failure modes are both silent, so both are
   asserted: offering Start when the server would refuse it (a button that
   fails when pressed reads as a broken page), and sitting on "checking…"
   against a GCS process older than this file. */
const bag = STATE.bag;
if (!bag) {
  check('a missing bag block is explained, not left as "checking…"',
        txt('bag-note').length > 0 && txt('bag-state') !== 'checking…',
        JSON.stringify(txt('bag-state')));
} else {
  check('recording state rendered',
        txt('bag-state') === (bag.recording ? 'recording' : 'idle'),
        JSON.stringify(txt('bag-state')));
  check('the recording panel always says something',
        txt('bag-note').length > 0, txt('bag-note'));
  check('Start offered only when idle, not busy and above the disk floor',
        IDS['bag-start'].disabled
          === !!(blocked || bag.recording || bag.start_blocked),
        `recording=${bag.recording} blocked=${JSON.stringify(bag.start_blocked)}`);
  check('Stop offered only while recording',
        IDS['bag-stop'].disabled === !bag.recording);
  check('the free-space figure reaches the operator',
        txt('bag-note').includes(String(bag.free_gb)), txt('bag-note'));
  if (bag.start_blocked) {
    check('...and it says why, rather than just greying out',
          txt('bag-note').startsWith('unavailable'), txt('bag-note'));
  }
  // "All topics" is the default because it is the one that cannot be got
  // wrong; the picker is a deliberate second step.
  check('the mode chips are exclusive, and All is the default',
        IDS['bag-mode-all'].className.includes('sel')
          && !IDS['bag-mode-pick'].className.includes('sel'));
  check('the topic picker is closed until asked for',
        IDS['bag-picker'].hidden === true);
  IDS['bag-mode-pick'].onclick();
  check('choosing "Choose topics" opens the picker',
        IDS['bag-picker'].hidden === false);
  check('...and switching mode is exclusive',
        IDS['bag-mode-pick'].className.includes('sel')
          && !IDS['bag-mode-all'].className.includes('sel'));
  IDS['bag-mode-all'].onclick();
  check('...and switching back closes it again',
        IDS['bag-picker'].hidden === true);
}

const rv = STATE.rviz || {};
if (rv.helper) {
  check('RViz buttons enabled when a helper is connected',
        !(IDS['rviz-start'].disabled && IDS['rviz-stop'].disabled));
  check('paste-the-command row hidden when helper is present',
        IDS['cmdrow'].hidden === true);
  check('helper host named', txt('rviz-note').includes(rv.host || ''));
} else {
  check('RViz buttons disabled with no helper',
        IDS['rviz-start'].disabled === true && IDS['rviz-stop'].disabled === true);
  check('paste-the-command row shown instead', IDS['cmdrow'].hidden === false);
  check('the command includes the token',
        txt('viewer-cmd').includes('?k='), txt('viewer-cmd'));
  // 192.168.1.50 is eno1, the MID360's point-to-point link. A helper command
  // pointing there can never work from a laptop -- and it used to.
  check('the command uses a routable name, not the LiDAR link IP',
        !/\/\/\d+\.\d+\.\d+\.\d+/.test(txt('viewer-cmd')), txt('viewer-cmd'));
}

print(fail ? `\n${fail} FAILED` : '\nall passed');
imports.system.exit(fail ? 1 : 0);
