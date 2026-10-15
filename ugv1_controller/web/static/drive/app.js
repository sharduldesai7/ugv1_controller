// UGV1 drive page.
//
// Keys, speeds and Drive Enable follow teleop_twist_keyboard (without the
// Shift sideways keys, which a differential-drive rover cannot do):
//   u i o / j k l / m , .   move and turn      k   stop
//   q z   both speeds +/- 10%   w x   linear only   e c   turn only
// A held key sends its command about 10 times a second; releasing it sends
// zero. The server also treats a command older than 0.3 s as zero.
(function () {
  'use strict';

  const API = '/drive/api';
  const SEND_MS = 100;
  const POLL_MS = 200;
  const LINK_FAIL_LIMIT = 4;           // failed polls in a row before "link lost"

  const MOVES = {
    u: [1, 1], i: [1, 0], o: [1, -1],
    j: [0, 1], k: [0, 0], l: [0, -1],
    m: [-1, -1], ',': [-1, 0], '.': [-1, 1],
  };
  const SPEED_KEYS = {
    q: ['both', 1], z: ['both', -1],
    w: ['lin', 1],  x: ['lin', -1],
    e: ['turn', 1], c: ['turn', -1],
  };
  const REASONS = {
    locked: 'Another device is driving.',
    moving: 'Stop the rover before switching off.',
    settling: 'Wait for the rover to come to rest.',
    imu_missing: 'No IMU data, cannot confirm the rover is still.',
    imu_moving: 'The IMU still shows motion. Wait for it to settle.',
    not_driver: 'This device does not have control.',
  };

  const $ = (id) => document.getElementById(id);
  const el = {
    body: document.body, feed: $('feed'), noFeed: $('no-feed'),
    enable: $('enable'), hint: $('hint'),
    statusText: $('status-text'), statusDot: $('status-dot'),
    linkText: $('link-text'), linkDot: $('link-dot'),
    cmdLin: $('cmd-lin'), cmdAng: $('cmd-ang'),
    maxLin: $('max-lin'), maxTurn: $('max-turn'),
    stopBtn: $('stop-btn'), joy: $('joy'), knob: $('joy-knob'),
  };
  const keyButtons = Array.from(document.querySelectorAll('.key'));

  const client = (function () {
    const a = new Uint8Array(8);
    if (window.crypto && crypto.getRandomValues) crypto.getRandomValues(a);
    else for (let i = 0; i < 8; i++) a[i] = Math.floor(Math.random() * 256);
    return Array.from(a, (b) => b.toString(16).padStart(2, '0')).join('');
  })();

  let state = { enabled: false, you_drive: false, locked: false, can_disable: false };
  let linkOk = false;
  let failedPolls = 0;
  let held = [];                        // move keys currently down, newest last
  let joy = { x: 0, th: 0 };
  let zerosToSend = 0;
  let busy = false;                     // a command request is in flight
  let toastTimer = null;

  // ── Requests ──────────────────────────────────────────────────────────────

  function post(path, body) {
    return fetch(API + path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(Object.assign({ client: client }, body || {})),
      cache: 'no-store',
    }).then((r) => r.json().then((j) => ({ status: r.status, data: j })));
  }

  function apply(data) {
    if (data && typeof data.enabled === 'boolean') {
      state = data;
      render();
    }
  }

  function poll() {
    fetch(API + '/state?client=' + client, { cache: 'no-store' })
      .then((r) => r.json())
      .then((j) => { failedPolls = 0; linkOk = true; apply(j); })
      .catch(() => {
        failedPolls += 1;
        if (failedPolls >= LINK_FAIL_LIMIT) { linkOk = false; held = []; joyReset(); render(); }
      });
  }

  // ── Commands ──────────────────────────────────────────────────────────────

  function currentCommand() {
    if (!state.you_drive || !linkOk) return { x: 0, th: 0 };
    if (joy.x !== 0 || joy.th !== 0) return joy;
    if (held.length) {
      const m = MOVES[held[held.length - 1]];
      return { x: m[0], th: m[1] };
    }
    return { x: 0, th: 0 };
  }

  function sendLoop() {
    const c = currentCommand();
    const moving = c.x !== 0 || c.th !== 0;
    if (moving) zerosToSend = 2;
    if ((moving || zerosToSend > 0) && state.you_drive && !busy) {
      if (!moving) zerosToSend -= 1;
      busy = true;
      post('/cmd', { x: c.x, th: c.th })
        .then((r) => apply(r.data))
        .catch(() => {})
        .then(() => { busy = false; });
    }
  }

  function stopAll() {
    held = [];
    joyReset();
    zerosToSend = 2;
    render();
  }

  // ── Drive Enable ──────────────────────────────────────────────────────────

  function toast(msg) {
    el.hint.textContent = msg;
    el.body.classList.add('toasting');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { el.body.classList.remove('toasting'); render(); }, 3500);
  }

  function toggleEnable() {
    if (!linkOk) { toast('No connection to the rover.'); return; }
    if (state.locked) { toast(REASONS.locked); return; }
    const path = state.you_drive ? '/disable' : '/enable';
    if (state.you_drive) stopAll();
    post(path).then((r) => {
      apply(r.data);
      if (!r.data.ok && r.data.error) toast(REASONS[r.data.error] || 'Refused: ' + r.data.error);
    }).catch(() => toast('No connection to the rover.'));
  }

  // ── Rendering ─────────────────────────────────────────────────────────────

  const fmt = (v) => (Math.abs(v) < 0.005 ? '+0.00' : (v < 0 ? '-' : '+') + Math.abs(v).toFixed(2));

  function render() {
    const you = state.you_drive && linkOk;
    const driving = you && (state.cmd_lin !== 0 || state.cmd_ang !== 0);
    el.body.classList.toggle('is-on', you);
    el.body.classList.toggle('is-locked', linkOk && state.locked);
    el.body.classList.toggle('is-down', !linkOk);

    el.enable.setAttribute('aria-checked', you ? 'true' : 'false');
    el.enable.classList.toggle('on', you);
    el.enable.classList.toggle('blocked', you && !state.can_disable);

    let text = 'DISABLED', color = 'idle';
    if (!linkOk) { text = 'LINK LOST'; color = 'bad'; }
    else if (state.locked) { text = 'IN USE'; color = 'idle'; }
    else if (driving) { text = 'DRIVING'; color = 'active'; }
    else if (you) { text = 'STOPPED'; color = 'ready'; }
    el.statusText.textContent = text;
    el.statusDot.className = 'dot ' + color;
    el.statusText.className = color;

    el.linkText.textContent = linkOk ? 'LINK OK' : 'NO LINK';
    el.linkDot.className = 'dot ' + (linkOk ? 'ok' : 'bad');

    el.cmdLin.textContent = fmt(you ? state.cmd_lin : 0);
    el.cmdAng.textContent = fmt(you ? state.cmd_ang : 0);
    if (typeof state.lin_max === 'number') {
      el.maxLin.textContent = state.lin_max.toFixed(2);
      el.maxTurn.textContent = state.turn_max.toFixed(2);
    }

    keyButtons.forEach((b) => b.classList.toggle(
      'active', you && b.dataset.key !== 'k' && held[held.length - 1] === b.dataset.key));

    if (!el.body.classList.contains('toasting')) {
      el.hint.textContent = !linkOk ? 'No connection to the rover.'
        : state.locked ? 'Another device is driving.'
        : !you ? 'Switch on to take control.'
        : driving ? 'Stop the rover to switch off.'
        : 'Hold to drive. Release to stop.';
    }
  }

  // ── Keyboard ──────────────────────────────────────────────────────────────

  function speed(target, dir) {
    if (!state.you_drive) return;
    post('/speed', { target: target, direction: dir }).then((r) => apply(r.data)).catch(() => {});
  }

  function pressMove(key) {
    if (!state.you_drive) return;
    if (key === 'k') { stopAll(); return; }
    held = held.filter((k) => k !== key);
    held.push(key);
    render();
  }

  function releaseMove(key) {
    held = held.filter((k) => k !== key);
    zerosToSend = 2;
    render();
  }

  window.addEventListener('keydown', (e) => {
    if (e.ctrlKey || e.metaKey || e.altKey || e.shiftKey) return;
    const key = e.key.toLowerCase();
    if (key in MOVES) {
      e.preventDefault();
      if (!e.repeat) pressMove(key);
    } else if (key in SPEED_KEYS) {
      e.preventDefault();
      if (!e.repeat) speed(SPEED_KEYS[key][0], SPEED_KEYS[key][1]);
    }
  });
  window.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    if (key in MOVES && key !== 'k') releaseMove(key);
  });
  window.addEventListener('blur', stopAll);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopAll(); });
  window.addEventListener('pagehide', stopAll);

  // ── On-screen keys, speed buttons, STOP ───────────────────────────────────

  keyButtons.forEach((b) => {
    const key = b.dataset.key;
    b.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      b.setPointerCapture(e.pointerId);
      pressMove(key);
    });
    const up = () => { if (key !== 'k') releaseMove(key); };
    b.addEventListener('pointerup', up);
    b.addEventListener('pointercancel', up);
    b.addEventListener('lostpointercapture', up);
  });

  document.querySelectorAll('.step').forEach((b) => {
    b.addEventListener('click', () => speed(b.dataset.target, Number(b.dataset.dir)));
  });
  el.stopBtn.addEventListener('pointerdown', (e) => { e.preventDefault(); stopAll(); });
  el.enable.addEventListener('click', toggleEnable);

  // ── Joystick ──────────────────────────────────────────────────────────────

  const JOY_TRAVEL = 52;               // px the knob can move from the centre
  const JOY_DEADZONE = 0.12;
  let joyId = null;

  function joyReset() {
    joyId = null;
    joy = { x: 0, th: 0 };
    el.knob.style.transform = 'translate(-50%, -50%)';
    zerosToSend = 2;
  }

  function joyMove(e) {
    const r = el.joy.getBoundingClientRect();
    let dx = (e.clientX - (r.left + r.width / 2)) / JOY_TRAVEL;
    let dy = (e.clientY - (r.top + r.height / 2)) / JOY_TRAVEL;
    const m = Math.hypot(dx, dy);
    if (m > 1) { dx /= m; dy /= m; }
    el.knob.style.transform =
      'translate(calc(-50% + ' + dx * JOY_TRAVEL + 'px), calc(-50% + ' + dy * JOY_TRAVEL + 'px))';
    const mag = Math.min(m, 1);
    if (mag < JOY_DEADZONE || !state.you_drive) { joy = { x: 0, th: 0 }; return; }
    const s = ((mag - JOY_DEADZONE) / (1 - JOY_DEADZONE)) / mag;
    joy = { x: -dy * s, th: -dx * s };            // up = forward, left = turn left
  }

  el.joy.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    joyId = e.pointerId;
    el.joy.setPointerCapture(joyId);
    joyMove(e);
  });
  el.joy.addEventListener('pointermove', (e) => { if (e.pointerId === joyId) joyMove(e); });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((n) =>
    el.joy.addEventListener(n, (e) => { if (e.pointerId === joyId) joyReset(); }));

  // ── Camera feed ───────────────────────────────────────────────────────────

  function checkFeed() {
    el.noFeed.style.display = el.feed.naturalWidth > 0 ? 'none' : 'flex';
  }
  el.feed.addEventListener('error', () => {
    setTimeout(() => { el.feed.src = '/drive/feed/raw?' + Date.now(); }, 1000);
  });

  // ── Start ─────────────────────────────────────────────────────────────────

  render();
  poll();
  setInterval(poll, POLL_MS);
  setInterval(sendLoop, SEND_MS);
  setInterval(checkFeed, 1000);
})();
