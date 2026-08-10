(() => {
  'use strict';

  const elements = Object.fromEntries(
    [...document.querySelectorAll('[id]')].map((element) => [element.id, element]),
  );
  let token = sessionStorage.getItem('dogzillaGatewayToken') || '';
  let currentMap = 'test1';
  let pollTimer = null;
  let eventAbort = null;
  let toastTimer = null;

  async function api(path, options = {}) {
    const headers = new Headers(options.headers || {});
    headers.set('Authorization', `Bearer ${token}`);
    if (options.body) headers.set('Content-Type', 'application/json');
    const response = await fetch(path, { ...options, headers, cache: 'no-store' });
    if (response.status === 401) {
      disconnect('The gateway rejected that token.');
      throw new Error('Authentication failed');
    }
    let payload = {};
    try { payload = await response.json(); } catch (_) { /* response may be empty */ }
    if (!response.ok) throw new Error(payload.error || `Request failed (${response.status})`);
    return payload;
  }

  function setConnection(online) {
    elements['connection-pill'].className = `pill ${online ? 'online' : 'offline'}`;
    elements['connection-pill'].innerHTML = `<span></span>${online ? 'Live' : 'Offline'}`;
  }

  function disconnect(message = '') {
    token = '';
    sessionStorage.removeItem('dogzillaGatewayToken');
    clearInterval(pollTimer);
    if (eventAbort) eventAbort.abort();
    elements.app.classList.add('hidden');
    elements.login.classList.remove('hidden');
    elements['login-error'].textContent = message;
    elements.token.value = '';
    setConnection(false);
  }

  function ageLabel(item, empty) {
    if (!item) return empty;
    const age = Number(item.age_seconds || 0);
    return `${item.stale ? 'Stale · ' : 'Updated '}${age.toFixed(1)}s ago`;
  }

  function sourceTimeLabel(timestamp) {
    const value = Date.parse(timestamp || '');
    if (!Number.isFinite(value)) return 'No ROS graph';
    const age = Math.max(0, (Date.now() - value) / 1000);
    return `Graph ${age.toFixed(1)}s ago`;
  }

  function renderState(state) {
    setConnection(true);
    const telemetry = state.telemetry || {};
    const safety = state.safety || {};
    const robot = state.robot || {};
    currentMap = state.configuration?.map || telemetry.map?.value?.name || currentMap;
    elements['map-chip'].textContent = `map: ${currentMap}`;
    elements['robot-mode'].textContent = String(robot.mode || 'stopped').replaceAll('_', ' ');
    elements['gate-reason'].textContent = safety.task_ready
      ? 'Navigation checks passed. Ready to dispatch.'
      : safety.task_gate_reason || 'Robot is not ready for autonomous tasks.';

    const battery = telemetry.battery;
    const percentage = battery?.value?.percentage;
    elements['battery-value'].textContent = Number.isFinite(percentage) ? percentage.toFixed(0) : '—';
    const width = Number.isFinite(percentage) ? Math.max(0, Math.min(100, percentage)) : 0;
    elements['battery-meter'].style.width = `${width}%`;
    elements['battery-meter'].style.background = width < 28
      ? 'var(--red)'
      : 'linear-gradient(90deg, var(--green-soft), var(--green))';
    elements['battery-age'].textContent = ageLabel(battery, 'No reading');

    const pose = telemetry.pose;
    const poseValue = pose?.value;
    elements['pose-x'].textContent = Number.isFinite(poseValue?.x) ? `${poseValue.x.toFixed(2)} m` : '—';
    elements['pose-y'].textContent = Number.isFinite(poseValue?.y) ? `${poseValue.y.toFixed(2)} m` : '—';
    elements['pose-yaw'].textContent = Number.isFinite(poseValue?.yaw) ? `${poseValue.yaw.toFixed(2)} rad` : '—';
    elements['pose-age'].textContent = ageLabel(pose, 'No localization');

    elements['nav-state'].textContent = robot.nav_available ? 'Ready' : 'Unavailable';
    elements['nav-state'].className = robot.nav_available ? 'ready' : 'unavailable';
    elements['ros-node-count'].textContent = Array.isArray(robot.nodes)
      ? String(robot.nodes.length)
      : '—';
    elements['graph-age'].textContent = sourceTimeLabel(robot.updated_at);
    const joints = telemetry.joints;
    const jointCount = joints?.value?.count;
    elements['joint-count'].textContent = Number.isFinite(jointCount)
      ? `${jointCount}${joints.stale ? ' (stale)' : ''}`
      : 'No reading';
    const linear = poseValue?.linear_speed;
    const angular = poseValue?.angular_speed;
    elements['velocity-detail'].textContent = Number.isFinite(linear) && Number.isFinite(angular)
      ? `${linear.toFixed(2)} m/s · ${angular.toFixed(2)} rad/s`
      : 'No reading';
    const map = telemetry.map;
    const mapValue = map?.value;
    elements['map-detail'].textContent = Number.isFinite(mapValue?.width)
      ? `${mapValue.name} · ${mapValue.width}×${mapValue.height} · ${mapValue.resolution} m/cell`
      : 'No map received';

    const active = state.active_task;
    if (active) {
      const count = active.payload?.waypoints?.length || 1;
      const step = Math.min(count, Number(active.current_step || 0));
      elements['active-task'].textContent = active.name;
      elements['active-task-detail'].textContent = `${active.state} · stop ${Math.min(step + 1, count)} of ${count}`;
      elements['task-progress'].style.width = `${Math.round((step / count) * 100)}%`;
    } else {
      elements['active-task'].textContent = 'None';
      elements['active-task-detail'].textContent = 'Queue is idle';
      elements['task-progress'].style.width = '0%';
    }

    const latched = Boolean(safety.estop_latched);
    elements['safety-panel'].classList.toggle('latched', latched);
    elements['safety-title'].textContent = latched ? 'Emergency stop latched' : 'Emergency stop ready';
    elements['safety-detail'].textContent = latched
      ? 'Autonomous movement is blocked until an operator resets it.'
      : 'Stops navigation and continuously commands zero velocity.';
    elements.estop.disabled = latched;
    elements['reset-estop'].classList.toggle('hidden', !latched);
    elements['submit-mission'].disabled = latched;
  }

  function createTaskElement(task) {
    const item = document.createElement('div');
    item.className = 'task-item';
    const name = document.createElement('strong');
    name.textContent = task.name;
    name.title = task.name;
    const state = document.createElement('span');
    state.className = `task-state ${task.state}`;
    state.textContent = task.state;
    const meta = document.createElement('span');
    meta.className = 'task-meta';
    const total = task.payload?.waypoints?.length || 0;
    meta.textContent = `${task.kind} · ${task.current_step}/${total} stops · ${new Date(task.created_at).toLocaleString()}`;
    item.append(name, state, meta);
    if (['queued', 'running', 'cancelling'].includes(task.state)) {
      const cancel = document.createElement('button');
      cancel.className = 'cancel-task';
      cancel.type = 'button';
      cancel.textContent = task.state === 'cancelling' ? 'Cancellation requested' : 'Cancel mission';
      cancel.disabled = task.state === 'cancelling';
      cancel.addEventListener('click', () => cancelTask(task.id));
      item.append(cancel);
    }
    if (task.error) {
      const error = document.createElement('span');
      error.className = 'task-meta';
      error.textContent = task.error;
      error.title = task.error;
      item.append(error);
    }
    return item;
  }

  async function refreshTasks() {
    const { tasks } = await api('/api/v1/tasks?limit=50');
    elements['task-list'].replaceChildren();
    if (!tasks.length) {
      const empty = document.createElement('p');
      empty.className = 'empty-state';
      empty.textContent = 'No missions yet.';
      elements['task-list'].append(empty);
      return;
    }
    tasks.forEach((task) => elements['task-list'].append(createTaskElement(task)));
  }

  async function refreshAll() {
    try {
      const [state] = await Promise.all([api('/api/v1/state'), refreshTasks()]);
      renderState(state);
    } catch (error) {
      if (token) {
        setConnection(false);
        elements['gate-reason'].textContent = error.message;
      }
    }
  }

  function showToast(message) {
    elements.toast.textContent = message;
    elements.toast.classList.add('visible');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => elements.toast.classList.remove('visible'), 3500);
  }

  async function post(path, body = {}) {
    return api(path, { method: 'POST', body: JSON.stringify(body) });
  }

  async function cancelTask(taskId) {
    try {
      await post(`/api/v1/tasks/${encodeURIComponent(taskId)}/cancel`);
      showToast('Mission cancellation requested.');
      await refreshAll();
    } catch (error) { showToast(error.message); }
  }

  function waypoint(form, prefix, label) {
    return {
      label,
      x: Number(form.get(`${prefix}-x`)),
      y: Number(form.get(`${prefix}-y`)),
      yaw: Number(form.get(`${prefix}-yaw`)),
      dwell_seconds: Number(form.get(`${prefix}-dwell`)),
    };
  }

  async function connectEvents() {
    if (eventAbort) eventAbort.abort();
    eventAbort = new AbortController();
    try {
      const response = await fetch('/api/v1/events', {
        headers: { Authorization: `Bearer ${token}` },
        cache: 'no-store',
        signal: eventAbort.signal,
      });
      if (!response.ok || !response.body) return;
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';
      while (token) {
        const { value, done } = await reader.read();
        if (done) break;
        buffer += decoder.decode(value, { stream: true });
        const records = buffer.split('\n\n');
        buffer = records.pop() || '';
        if (records.some((record) => record.includes('\ndata: '))) await refreshAll();
      }
      if (token) setTimeout(connectEvents, 1000);
    } catch (error) {
      if (error.name !== 'AbortError' && token) setTimeout(connectEvents, 3000);
    }
  }

  elements['login-form'].addEventListener('submit', async (event) => {
    event.preventDefault();
    token = elements.token.value.trim();
    elements['login-error'].textContent = '';
    try {
      const state = await api('/api/v1/state');
      sessionStorage.setItem('dogzillaGatewayToken', token);
      elements.login.classList.add('hidden');
      elements.app.classList.remove('hidden');
      renderState(state);
      await refreshTasks();
      clearInterval(pollTimer);
      pollTimer = setInterval(refreshAll, 3000);
      connectEvents();
    } catch (error) {
      elements['login-error'].textContent = error.message;
    }
  });

  elements['delivery-form'].addEventListener('submit', async (event) => {
    event.preventDefault();
    const form = new FormData(event.currentTarget);
    elements['mission-message'].className = 'form-message';
    elements['mission-message'].textContent = '';
    try {
      const task = await post('/api/v1/tasks/delivery', {
        name: form.get('name'),
        map: currentMap,
        pickup: waypoint(form, 'pickup', 'Pickup'),
        dropoff: waypoint(form, 'dropoff', 'Drop-off'),
      });
      elements['mission-message'].classList.add('success');
      elements['mission-message'].textContent = `Queued ${task.name}.`;
      showToast('Delivery added to the mission queue.');
      await refreshAll();
    } catch (error) { elements['mission-message'].textContent = error.message; }
  });

  elements.estop.addEventListener('click', async () => {
    if (!window.confirm('Latch the emergency stop and cancel active navigation?')) return;
    try { renderState({ ...(await api('/api/v1/state')), safety: await post('/api/v1/estop') }); await refreshAll(); }
    catch (error) { showToast(error.message); }
  });
  elements['reset-estop'].addEventListener('click', async () => {
    try { await post('/api/v1/estop/reset'); showToast('Emergency stop reset.'); await refreshAll(); }
    catch (error) { showToast(error.message); }
  });
  elements.refresh.addEventListener('click', refreshAll);
  elements.logout.addEventListener('click', () => disconnect());

  if (token) {
    elements.token.value = token;
    elements['login-form'].requestSubmit();
  }
})();
