// Auth-path test for the GCS front end -- run it with gjs, no browser needed.
//
//   python3 scripts/gcs/test_render.py     # runs this too
//   gjs scripts/gcs/test_gate.js <state.json> <config.json> [static/]
//
// The failure it exists to prevent: the dashboard is reached by a hostname
// (`http://gypsy.local:8080`), and the whole point of a memorable name is that
// you TYPE it -- at which moment the `?k=…` is gone, the page still loads, and
// every API call 401s. That looks like a broken dashboard rather than a
// missing credential, and it is what happened the first time the name was
// used. So: no token in the URL here, 401 from every call, then a paste.
//
// gjs has no `localStorage` at all, which is the same shape as a browser in a
// privacy mode that throws on it -- so this doubles as proof that the page
// still renders when storage is unavailable.
const GLib = imports.gi.GLib;
const read = (f) => {
  const [ok, bytes] = GLib.file_get_contents(f);
  if (!ok) throw new Error('cannot read ' + f);
  return new TextDecoder().decode(bytes);
};
const STATE = JSON.parse(read(ARGV[0]));
const CFG = JSON.parse(read(ARGV[1]));
const STATIC = ARGV[2] || 'scripts/gcs/static';

function mkNode(tag) {
  return {
    tagName: tag, children: [], _text: '', className: '', style: {},
    hidden: false, disabled: false, title: '', value: '', scrollTop: 0,
    clientHeight: 0, scrollHeight: 0, onclick: null, onkeydown: null,
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
  createElement: mkNode, hidden: false, addEventListener() {},
};
globalThis.location = { search: '' };          // <-- the whole point: no ?k=
globalThis.confirm = () => false;
globalThis.navigator = {};
globalThis.setInterval = () => 0;
globalThis.setTimeout = () => 0;
globalThis.clearTimeout = () => {};
globalThis.URLSearchParams = function () { this.get = () => null; };
// localStorage is deliberately NOT stubbed: every access must throw.

let UNLOCKED = false;
globalThis.fetch = (url, opts) => {
  const tok = (opts && opts.headers && opts.headers['X-GCS-Token']) || '';
  if (!UNLOCKED || tok !== 'realtoken') {
    return Promise.resolve({
      ok: false, status: 401,
      json: () => Promise.resolve({ error: 'bad or missing token' }) });
  }
  return Promise.resolve({ ok: true, status: 200, json: () => Promise.resolve(
    url.includes('/api/config') ? CFG
    : url.includes('/api/state') ? STATE
    : url.includes('/api/logs') ? { lines: ['log line'], seq: 1 } : {}) });
};

let fail = 0;
const check = (name, cond, detail) => {
  print(`  ${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  -- ' + detail : ''}`);
  if (!cond) fail++;
};
const settle = (ms) => {
  const loop = GLib.MainLoop.new(null, false);
  GLib.timeout_add(GLib.PRIORITY_DEFAULT, ms, () => { loop.quit(); return false; });
  loop.run();
};

try {
  new Function(read(STATIC + '/app.js'))();
} catch (e) {
  print('  FAIL  app.js threw: ' + e.message + '\n' + (e.stack || ''));
  fail++;
}
settle(400);

print('token gate');
check('app.js runs with localStorage unavailable', fail === 0,
      'a storage preference must not stop the page rendering');
check('the gate is shown when the URL carries no token',
      IDS['tokengate'].hidden === false);
check('the gate says the URL had no token', IDS['gate-why']._text.includes('?k='),
      IDS['gate-why']._text);
check('a 401 is NOT reported as a lost link',
      !IDS['host']._text.includes('lost the Jetson'),
      'saying "lost the Jetson" here sends you debugging Wi-Fi instead of pasting a token');

UNLOCKED = true;
IDS['token-input'].value = '  realtoken  ';     // leading/trailing space from a paste
IDS['token-save'].onclick();
settle(600);

check('a valid pasted token hides the gate', IDS['tokengate'].hidden === true);
check('the pasted token is trimmed', IDS['host']._text.includes(CFG.host),
      IDS['host']._text);
// Rows come from the state, not the config: a locked client is shown the
// follower's row but not its log tab.
check('the dashboard fills in without a reload',
      IDS['procs'].children.length === (STATE.procs || []).length);

print(fail ? `\n${fail} FAILED` : '\nall passed');
imports.system.exit(fail ? 1 : 0);
