(() => {
  'use strict';

  const elements = Object.fromEntries(
    [...document.querySelectorAll('[id]')].map((element) => [element.id, element]),
  );
  const targetButtons = [...document.querySelectorAll('[data-map-target]')];
  let token = sessionStorage.getItem('dogzillaGatewayToken') || '';
  let currentMap = 'test1';
  let pollTimer = null;
  let eventAbort = null;
  let toastTimer = null;
  let mapSnapshot = null;
  let mapCells = null;
  let mapImage = null;
  let mapView = null;
  let robotPose = null;
  let activeTarget = 'pickup';
  let plannedPath = [];
  let previewTimer = null;
  let previewGeneration = 0;
  let visionFrameTimer = null;
  let visionFrameUrl = '';
  const waypoints = { pickup: null, dropoff: null };

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

  async function post(path, body = {}) {
    return api(path, { method: 'POST', body: JSON.stringify(body) });
  }

  function setConnection(online) {
    elements['connection-pill'].className = `pill ${online ? 'online' : 'offline'}`;
    elements['connection-pill'].innerHTML = `<span></span>${online ? 'Live' : 'Offline'}`;
  }

  function disconnect(message = '') {
    token = '';
    sessionStorage.removeItem('dogzillaGatewayToken');
    clearInterval(pollTimer);
    clearTimeout(previewTimer);
    clearTimeout(visionFrameTimer);
    if (visionFrameUrl) URL.revokeObjectURL(visionFrameUrl);
    visionFrameUrl = '';
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

  async function refreshVisionFrame() {
    if (!token) return;
    if (document.hidden) {
      visionFrameTimer = setTimeout(refreshVisionFrame, 1000);
      return;
    }
    try {
      const response = await fetch('/api/v1/vision/frame.jpg', {
        headers: { Authorization: `Bearer ${token}` },
        cache: 'no-store',
      });
      if (response.status === 401) {
        disconnect('The gateway rejected that token.');
        return;
      }
      if (!response.ok) throw new Error('Vision frame unavailable');
      const nextUrl = URL.createObjectURL(await response.blob());
      const previousUrl = visionFrameUrl;
      visionFrameUrl = nextUrl;
      elements['vision-frame'].src = nextUrl;
      elements['vision-frame'].classList.add('live');
      elements['vision-placeholder'].classList.add('hidden');
      if (previousUrl) URL.revokeObjectURL(previousUrl);
    } catch (_) {
      elements['vision-frame'].classList.remove('live');
      elements['vision-placeholder'].classList.remove('hidden');
    } finally {
      if (token) visionFrameTimer = setTimeout(refreshVisionFrame, 250);
    }
  }

  function renderVision(telemetry) {
    const status = telemetry.vision_status;
    const statusValue = status?.value;
    const live = Boolean(statusValue) && !status.stale;
    elements['vision-status'].className = `pill ${live ? 'online' : 'offline'}`;
    elements['vision-status'].innerHTML = `<span></span>${live ? 'Live' : 'Unavailable'}`;
    if (statusValue?.mode) elements['vision-mode'].value = statusValue.mode;
    if (statusValue?.color) elements['vision-color'].value = statusValue.color;

    const vision = telemetry.vision;
    const result = vision?.value;
    const detections = Array.isArray(result?.detections) ? result.detections : [];
    if (!detections.length) {
      elements['vision-result'].textContent = result?.mode
        ? `${String(result.mode).replaceAll('-', ' ')} · no target`
        : 'No detections';
    } else {
      const first = detections[0];
      if (first.kind === 'qr') {
        elements['vision-result'].textContent = `QR · ${first.text || '(empty)'}`;
      } else {
        const offset = Number.isFinite(first.error_x)
          ? ` · horizontal ${first.error_x.toFixed(2)}`
          : '';
        elements['vision-result'].textContent = `${first.kind}${offset}`;
      }
    }
    elements['vision-age'].textContent = ageLabel(vision, 'No vision telemetry');
    elements['vision-apply'].disabled = !live;
  }

  function setMapMessage(message, success = false) {
    elements['map-message'].textContent = message;
    elements['map-message'].className = `form-message${success ? ' success' : ''}`;
  }

  function normalizeAngle(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
  }

  function decodeMapRuns(snapshot) {
    if (snapshot.encoding !== 'rle-value-count' || !Array.isArray(snapshot.runs)) {
      throw new Error('Gateway returned an unsupported occupancy-map encoding.');
    }
    if (snapshot.runs.length % 2 !== 0) throw new Error('Occupancy-map run data is malformed.');
    const expected = snapshot.width * snapshot.height;
    const cells = new Int8Array(expected);
    let offset = 0;
    for (let index = 0; index < snapshot.runs.length; index += 2) {
      const value = Number(snapshot.runs[index]);
      const count = Number(snapshot.runs[index + 1]);
      if (!Number.isInteger(value) || value < -1 || value > 100 || !Number.isInteger(count) || count < 1) {
        throw new Error('Occupancy-map run data contains an invalid value.');
      }
      if (offset + count > expected) throw new Error('Occupancy-map run data is too long.');
      cells.fill(value, offset, offset + count);
      offset += count;
    }
    if (offset !== expected) throw new Error('Occupancy-map run data is incomplete.');
    return cells;
  }

  function createMapImage(snapshot, cells) {
    const imageCanvas = document.createElement('canvas');
    imageCanvas.width = snapshot.width;
    imageCanvas.height = snapshot.height;
    const context = imageCanvas.getContext('2d');
    const image = context.createImageData(snapshot.width, snapshot.height);
    for (let row = 0; row < snapshot.height; row += 1) {
      const displayRow = snapshot.height - row - 1;
      for (let column = 0; column < snapshot.width; column += 1) {
        const value = cells[row * snapshot.width + column];
        const pixel = (displayRow * snapshot.width + column) * 4;
        let shade;
        if (value < 0) shade = 104;
        else if (value >= snapshot.occupied_threshold) shade = 15;
        else shade = Math.round(230 - (value / snapshot.occupied_threshold) * 85);
        image.data[pixel] = shade;
        image.data[pixel + 1] = value < 0 ? shade + 8 : shade + 5;
        image.data[pixel + 2] = value < 0 ? shade + 4 : shade + 2;
        image.data[pixel + 3] = 255;
      }
    }
    context.putImageData(image, 0, 0);
    return imageCanvas;
  }

  function worldToLocal(point) {
    if (!mapSnapshot) return null;
    const dx = point.x - mapSnapshot.origin.x;
    const dy = point.y - mapSnapshot.origin.y;
    const cosine = Math.cos(mapSnapshot.origin.yaw);
    const sine = Math.sin(mapSnapshot.origin.yaw);
    return {
      x: (cosine * dx + sine * dy) / mapSnapshot.resolution,
      y: (-sine * dx + cosine * dy) / mapSnapshot.resolution,
    };
  }

  function localToWorld(local) {
    const xMetres = local.x * mapSnapshot.resolution;
    const yMetres = local.y * mapSnapshot.resolution;
    const cosine = Math.cos(mapSnapshot.origin.yaw);
    const sine = Math.sin(mapSnapshot.origin.yaw);
    return {
      x: mapSnapshot.origin.x + cosine * xMetres - sine * yMetres,
      y: mapSnapshot.origin.y + sine * xMetres + cosine * yMetres,
    };
  }

  function worldToScreen(point) {
    const local = worldToLocal(point);
    if (!local || !mapView) return null;
    return {
      x: mapView.offsetX + local.x * mapView.scale,
      y: mapView.offsetY + (mapSnapshot.height - local.y) * mapView.scale,
    };
  }

  function eventToWorld(event) {
    if (!mapView || !mapSnapshot) return null;
    const bounds = elements['map-canvas'].getBoundingClientRect();
    const screenX = event.clientX - bounds.left;
    const screenY = event.clientY - bounds.top;
    const local = {
      x: (screenX - mapView.offsetX) / mapView.scale,
      y: mapSnapshot.height - (screenY - mapView.offsetY) / mapView.scale,
    };
    if (local.x < 0 || local.y < 0 || local.x >= mapSnapshot.width || local.y >= mapSnapshot.height) return null;
    return localToWorld(local);
  }

  function validateMapPoint(point, label = 'Waypoint') {
    if (!mapSnapshot || !mapCells) return { valid: false, reason: 'Occupancy map is unavailable.' };
    const local = worldToLocal(point);
    const column = Math.floor(local.x);
    const row = Math.floor(local.y);
    if (column < 0 || row < 0 || column >= mapSnapshot.width || row >= mapSnapshot.height) {
      return { valid: false, reason: `${label} is outside the active map.` };
    }
    const radius = Math.ceil(mapSnapshot.minimum_clearance_m / mapSnapshot.resolution);
    for (let offsetY = -radius; offsetY <= radius; offsetY += 1) {
      for (let offsetX = -radius; offsetX <= radius; offsetX += 1) {
        if (Math.hypot(offsetX, offsetY) * mapSnapshot.resolution > mapSnapshot.minimum_clearance_m) continue;
        const testColumn = column + offsetX;
        const testRow = row + offsetY;
        if (testColumn < 0 || testRow < 0 || testColumn >= mapSnapshot.width || testRow >= mapSnapshot.height) {
          return { valid: false, reason: `${label} is too close to the map boundary.` };
        }
        const value = mapCells[testRow * mapSnapshot.width + testColumn];
        if (value < 0) return { valid: false, reason: `${label} is in or too close to unknown space.` };
        if (value >= mapSnapshot.occupied_threshold) {
          return { valid: false, reason: `${label} is in or too close to an obstacle.` };
        }
      }
    }
    return { valid: true, reason: 'Free map location.' };
  }

  function drawPolyline(context, points, color, dashed = false) {
    const screenPoints = points.map(worldToScreen).filter(Boolean);
    if (screenPoints.length < 2) return;
    context.save();
    context.strokeStyle = color;
    context.lineWidth = 2.5;
    context.lineJoin = 'round';
    context.lineCap = 'round';
    if (dashed) context.setLineDash([7, 7]);
    context.beginPath();
    context.moveTo(screenPoints[0].x, screenPoints[0].y);
    screenPoints.slice(1).forEach((point) => context.lineTo(point.x, point.y));
    context.stroke();
    context.restore();
  }

  function drawPose(context, point, color, label, radius = 7) {
    const screen = worldToScreen(point);
    if (!screen) return;
    const headingWorld = {
      x: point.x + Math.cos(point.yaw || 0) * 0.38,
      y: point.y + Math.sin(point.yaw || 0) * 0.38,
    };
    const heading = worldToScreen(headingWorld);
    context.save();
    context.strokeStyle = color;
    context.fillStyle = color;
    context.lineWidth = 3;
    context.beginPath();
    context.moveTo(screen.x, screen.y);
    context.lineTo(heading.x, heading.y);
    context.stroke();
    context.beginPath();
    context.arc(screen.x, screen.y, radius, 0, Math.PI * 2);
    context.fill();
    context.font = '700 11px system-ui, sans-serif';
    context.fillStyle = '#effff7';
    context.shadowColor = '#07110e';
    context.shadowBlur = 4;
    context.fillText(label, screen.x + 10, screen.y - 9);
    context.restore();
  }

  function drawMap() {
    const canvas = elements['map-canvas'];
    const bounds = canvas.getBoundingClientRect();
    const ratio = Math.min(2, window.devicePixelRatio || 1);
    const pixelWidth = Math.max(1, Math.round(bounds.width * ratio));
    const pixelHeight = Math.max(1, Math.round(bounds.height * ratio));
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    const context = canvas.getContext('2d');
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, bounds.width, bounds.height);
    context.fillStyle = '#050a08';
    context.fillRect(0, 0, bounds.width, bounds.height);
    if (!mapSnapshot || !mapImage) return;

    const padding = 18;
    const scale = Math.max(0.01, Math.min(
      (bounds.width - padding * 2) / mapSnapshot.width,
      (bounds.height - padding * 2) / mapSnapshot.height,
    ));
    mapView = {
      scale,
      offsetX: (bounds.width - mapSnapshot.width * scale) / 2,
      offsetY: (bounds.height - mapSnapshot.height * scale) / 2,
    };
    context.imageSmoothingEnabled = false;
    context.drawImage(
      mapImage,
      mapView.offsetX,
      mapView.offsetY,
      mapSnapshot.width * scale,
      mapSnapshot.height * scale,
    );
    context.strokeStyle = 'rgba(94, 240, 166, .32)';
    context.lineWidth = 1;
    context.strokeRect(
      mapView.offsetX,
      mapView.offsetY,
      mapSnapshot.width * scale,
      mapSnapshot.height * scale,
    );

    if (plannedPath.length > 1) {
      drawPolyline(context, plannedPath, '#5ef0a6');
    } else {
      const direct = [robotPose, waypoints.pickup, waypoints.dropoff].filter(Boolean);
      drawPolyline(context, direct, 'rgba(94, 240, 166, .55)', true);
    }
    if (robotPose) drawPose(context, robotPose, '#5ef0a6', 'DOGZILLA', 8);
    if (waypoints.pickup) {
      const valid = validateMapPoint(waypoints.pickup, 'Pickup').valid;
      drawPose(context, waypoints.pickup, valid ? '#69baff' : '#ff5d68', 'PICKUP');
    }
    if (waypoints.dropoff) {
      const valid = validateMapPoint(waypoints.dropoff, 'Drop-off').valid;
      drawPose(context, waypoints.dropoff, valid ? '#ffbd59' : '#ff5d68', 'DROP-OFF');
    }
  }

  async function refreshMap(force = false) {
    const snapshot = await api('/api/v1/map');
    if (!force && mapSnapshot && snapshot.revision === mapSnapshot.revision) return;
    const cells = decodeMapRuns(snapshot);
    mapSnapshot = snapshot;
    mapCells = cells;
    mapImage = createMapImage(snapshot, cells);
    plannedPath = [];
    elements['map-loading'].classList.add('hidden');
    elements['map-editor-chip'].textContent = `map: ${snapshot.name}`;
    elements['map-revision'].textContent = `${snapshot.width}×${snapshot.height} · ${snapshot.resolution} m/cell · revision ${snapshot.revision}`;
    syncAllWaypointValidation();
    drawMap();
  }

  function inputValue(id) {
    const raw = elements[id].value.trim();
    if (!raw) return null;
    const value = Number(raw);
    return Number.isFinite(value) ? value : null;
  }

  function syncWaypointFromInputs(target, requestPreview = true) {
    const x = inputValue(`${target}-x`);
    const y = inputValue(`${target}-y`);
    const yaw = inputValue(`${target}-yaw`);
    waypoints[target] = x === null || y === null || yaw === null
      ? null
      : { x, y, yaw: normalizeAngle(yaw) };
    plannedPath = [];
    syncAllWaypointValidation();
    drawMap();
    if (requestPreview) scheduleRoutePreview();
  }

  function setWaypoint(target, point, yaw = 0, requestPreview = true) {
    const heading = elements[`${target}-yaw`];
    const options = [...heading.options];
    const selected = options.reduce((closest, option) => {
      const difference = Math.abs(normalizeAngle(Number(option.value) - yaw));
      return difference < closest.difference ? { option, difference } : closest;
    }, { option: options[0], difference: Infinity }).option;
    heading.value = selected.value;
    const selectedYaw = Number(selected.value);
    waypoints[target] = { x: point.x, y: point.y, yaw: selectedYaw };
    elements[`${target}-x`].value = point.x.toFixed(3);
    elements[`${target}-y`].value = point.y.toFixed(3);
    plannedPath = [];
    syncAllWaypointValidation();
    drawMap();
    if (requestPreview) scheduleRoutePreview();
  }

  function syncAllWaypointValidation() {
    const results = [];
    if (waypoints.pickup) results.push(validateMapPoint(waypoints.pickup, 'Pickup'));
    if (waypoints.dropoff) results.push(validateMapPoint(waypoints.dropoff, 'Drop-off'));
    const invalid = results.find((result) => !result.valid);
    if (invalid) setMapMessage(invalid.reason);
    else if (results.length) setMapMessage('Selected waypoint cells pass the local map check.', true);
    else setMapMessage('');
  }

  function setActiveTarget(target) {
    activeTarget = target;
    targetButtons.forEach((button) => {
      button.classList.toggle('active', button.dataset.mapTarget === target);
    });
    document.querySelectorAll('.apply-location').forEach((button) => {
      button.textContent = `Use as ${target === 'pickup' ? 'pickup' : 'drop-off'}`;
    });
  }

  function routePayload() {
    return {
      name: 'Map preview',
      map: currentMap,
      waypoints: [
        { label: 'Pickup', ...waypoints.pickup, dwell_seconds: 0 },
        { label: 'Drop-off', ...waypoints.dropoff, dwell_seconds: 0 },
      ],
    };
  }

  function scheduleRoutePreview() {
    clearTimeout(previewTimer);
    previewGeneration += 1;
    const generation = previewGeneration;
    if (!waypoints.pickup || !waypoints.dropoff) {
      elements['route-preview'].textContent = 'Select both waypoints to preview the route.';
      return;
    }
    const checks = [
      validateMapPoint(waypoints.pickup, 'Pickup'),
      validateMapPoint(waypoints.dropoff, 'Drop-off'),
    ];
    if (checks.some((check) => !check.valid)) {
      elements['route-preview'].textContent = 'Route preview blocked by an unsafe waypoint.';
      return;
    }
    elements['route-preview'].textContent = 'Asking Nav2 to calculate a safe path…';
    previewTimer = setTimeout(async () => {
      try {
        const preview = await post('/api/v1/routes/preview', routePayload());
        if (generation !== previewGeneration) return;
        plannedPath = Array.isArray(preview.path) ? preview.path : [];
        elements['route-preview'].textContent = `Nav2 preview · ${Number(preview.distance_m).toFixed(2)} m`;
        drawMap();
      } catch (error) {
        if (generation !== previewGeneration) return;
        plannedPath = [];
        elements['route-preview'].textContent = `Planner preview unavailable · ${error.message}`;
        drawMap();
      }
    }, 350);
  }

  function renderState(state) {
    setConnection(true);
    const telemetry = state.telemetry || {};
    renderVision(telemetry);
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
    robotPose = Number.isFinite(poseValue?.x) && Number.isFinite(poseValue?.y) && Number.isFinite(poseValue?.yaw)
      ? { x: poseValue.x, y: poseValue.y, yaw: poseValue.yaw }
      : null;
    elements['pose-x'].textContent = robotPose ? `${robotPose.x.toFixed(2)} m` : '—';
    elements['pose-y'].textContent = robotPose ? `${robotPose.y.toFixed(2)} m` : '—';
    elements['pose-yaw'].textContent = robotPose ? `${robotPose.yaw.toFixed(2)} rad` : '—';
    elements['pose-age'].textContent = ageLabel(pose, 'No map-frame localization');

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
    if (Number.isFinite(mapValue?.revision) && mapValue.revision !== mapSnapshot?.revision) {
      refreshMap().catch((error) => {
        elements['map-loading'].textContent = error.message;
        elements['map-loading'].classList.remove('hidden');
      });
    }

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
    elements['use-robot-pose'].disabled = !robotPose;
    drawMap();
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

  function locationElement(location) {
    const item = document.createElement('div');
    item.className = 'location-item';
    const name = document.createElement('strong');
    name.textContent = location.name;
    name.title = location.name;
    const coordinates = document.createElement('span');
    coordinates.className = 'location-coordinates';
    coordinates.textContent = `x ${location.x.toFixed(2)} · y ${location.y.toFixed(2)} · yaw ${location.yaw.toFixed(2)}`;
    const actions = document.createElement('div');
    actions.className = 'location-actions';
    const apply = document.createElement('button');
    apply.type = 'button';
    apply.className = 'apply-location';
    apply.textContent = `Use as ${activeTarget === 'pickup' ? 'pickup' : 'drop-off'}`;
    apply.addEventListener('click', () => {
      setWaypoint(activeTarget, location, location.yaw);
      showToast(`${location.name} applied to ${activeTarget}.`);
    });
    const remove = document.createElement('button');
    remove.type = 'button';
    remove.className = 'delete-location';
    remove.textContent = 'Delete';
    remove.addEventListener('click', async () => {
      if (!window.confirm(`Delete saved location “${location.name}”?`)) return;
      try {
        await api(`/api/v1/locations/${encodeURIComponent(location.id)}`, { method: 'DELETE' });
        await refreshLocations();
        showToast('Saved location deleted.');
      } catch (error) { showToast(error.message); }
    });
    actions.append(apply, remove);
    item.append(name, coordinates, actions);
    return item;
  }

  async function refreshLocations() {
    const { locations } = await api('/api/v1/locations');
    elements['location-list'].replaceChildren();
    if (!locations.length) {
      const empty = document.createElement('p');
      empty.className = 'empty-state';
      empty.textContent = 'No saved locations.';
      elements['location-list'].append(empty);
      return;
    }
    locations.forEach((location) => elements['location-list'].append(locationElement(location)));
  }

  async function refreshAll() {
    try {
      const [state] = await Promise.all([
        api('/api/v1/state'),
        refreshTasks(),
        refreshLocations(),
      ]);
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

  targetButtons.forEach((button) => {
    button.addEventListener('click', () => setActiveTarget(button.dataset.mapTarget));
  });

  ['pickup', 'dropoff'].forEach((target) => {
    ['x', 'y', 'yaw'].forEach((field) => {
      elements[`${target}-${field}`].addEventListener('input', () => syncWaypointFromInputs(target));
    });
  });

  elements['map-canvas'].addEventListener('click', (event) => {
    const point = eventToWorld(event);
    if (!point) return;
    const label = activeTarget === 'pickup' ? 'Pickup' : 'Drop-off';
    const validation = validateMapPoint(point, label);
    if (!validation.valid) {
      setMapMessage(validation.reason);
      return;
    }
    const yaw = inputValue(`${activeTarget}-yaw`) || 0;
    const completedTarget = activeTarget;
    setWaypoint(activeTarget, point, yaw);
    if (completedTarget === 'pickup' && !waypoints.dropoff) setActiveTarget('dropoff');
    event.preventDefault();
  });
  window.addEventListener('resize', drawMap);
  if (window.ResizeObserver) new ResizeObserver(drawMap).observe(elements['map-stage']);

  elements['use-robot-pose'].addEventListener('click', () => {
    if (!robotPose) return;
    setWaypoint(activeTarget, robotPose, robotPose.yaw);
  });

  elements['clear-waypoints'].addEventListener('click', () => {
    waypoints.pickup = null;
    waypoints.dropoff = null;
    plannedPath = [];
    ['pickup-x', 'pickup-y', 'dropoff-x', 'dropoff-y'].forEach((id) => { elements[id].value = ''; });
    elements['pickup-yaw'].value = '0';
    elements['dropoff-yaw'].value = '0';
    elements['route-preview'].textContent = 'Select both waypoints to preview the route.';
    setActiveTarget('pickup');
    syncAllWaypointValidation();
    drawMap();
  });

  elements['save-location'].addEventListener('click', async () => {
    const name = elements['location-name'].value.trim();
    const waypointValue = waypoints[activeTarget];
    if (!name) { showToast('Enter a location name first.'); return; }
    if (!waypointValue) { showToast(`Select a ${activeTarget} waypoint first.`); return; }
    const validation = validateMapPoint(waypointValue, name);
    if (!validation.valid) { showToast(validation.reason); return; }
    try {
      await post('/api/v1/locations', { map: currentMap, name, ...waypointValue });
      elements['location-name'].value = '';
      await refreshLocations();
      showToast(`${name} saved on ${currentMap}.`);
    } catch (error) { showToast(error.message); }
  });

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
      await Promise.all([refreshTasks(), refreshLocations()]);
      refreshMap(true).catch((error) => {
        elements['map-loading'].textContent = error.message;
        elements['map-loading'].classList.remove('hidden');
      });
      clearInterval(pollTimer);
      pollTimer = setInterval(refreshAll, 3000);
      connectEvents();
      clearTimeout(visionFrameTimer);
      refreshVisionFrame();
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
      if (!waypoints.pickup || !waypoints.dropoff) throw new Error('Select pickup and drop-off on the map first.');
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

  elements['vision-apply'].addEventListener('click', async () => {
    elements['vision-apply'].disabled = true;
    try {
      const response = await post('/api/v1/vision/mode', {
        mode: elements['vision-mode'].value,
        color: elements['vision-color'].value,
      });
      showToast(`${response.mode.replaceAll('-', ' ')} requested. Robot actions remain disabled.`);
    } catch (error) {
      showToast(error.message);
    } finally {
      await refreshAll();
    }
  });

  elements.estop.addEventListener('click', async () => {
    if (!window.confirm('Latch the emergency stop and cancel active navigation?')) return;
    try { await post('/api/v1/estop'); showToast('Emergency stop latched.'); await refreshAll(); }
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
