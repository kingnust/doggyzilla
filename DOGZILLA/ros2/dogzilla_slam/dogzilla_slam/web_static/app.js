(() => {
  'use strict';

  const elements = Object.fromEntries(
    [...document.querySelectorAll('[id]')].map((element) => [element.id, element]),
  );
  const targetButtons = [...document.querySelectorAll('[data-map-target]')];
  let password = sessionStorage.getItem('dogzillaGatewayPassword') || '';
  let currentMap = 'test1';
  let pollTimer = null;
  let eventAbort = null;
  let toastTimer = null;
  let mapSnapshot = null;
  let mapCells = null;
  let mapImage = null;
  let mapView = null;
  let mapZoom = 1;
  let robotPose = null;
  let activeTarget = 'pickup';
  let plannedPath = [];
  let previewTimer = null;
  let previewGeneration = 0;
  let visionFrameTimer = null;
  let visionFrameUrl = '';
  let visionFrameSequence = 0;
  const alertImageUrls = new Map();
  let patrolPolygon = [];
  let patrolWaypoints = [];
  let patrolAreas = [];
  let selectedPatrolAreaId = '';
  let keepoutPolygon = [];
  let keepoutZones = [];
  let selectedKeepoutZoneId = '';
  const waypoints = { pickup: null, dropoff: null };

  async function api(path, options = {}) {
    const headers = new Headers(options.headers || {});
    headers.set('X-Dogzilla-Password', password);
    if (options.body) headers.set('Content-Type', 'application/json');
    const response = await fetch(path, { ...options, headers, cache: 'no-store' });
    if (response.status === 401) {
      disconnect('The gateway rejected that password.');
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
    password = '';
    sessionStorage.removeItem('dogzillaGatewayPassword');
    clearInterval(pollTimer);
    clearTimeout(previewTimer);
    clearTimeout(visionFrameTimer);
    if (visionFrameUrl) URL.revokeObjectURL(visionFrameUrl);
    visionFrameUrl = '';
    visionFrameSequence = 0;
    alertImageUrls.forEach((url) => URL.revokeObjectURL(url));
    alertImageUrls.clear();
    if (eventAbort) eventAbort.abort();
    elements.app.classList.add('hidden');
    elements.login.classList.remove('hidden');
    elements['login-error'].textContent = message;
    elements.password.value = '';
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
    if (!password) return;
    if (document.hidden) {
      visionFrameTimer = setTimeout(refreshVisionFrame, 1000);
      return;
    }
    try {
      const response = await fetch(
        `/api/v1/vision/frame.jpg?after=${visionFrameSequence}`,
        {
        headers: { 'X-Dogzilla-Password': password },
        cache: 'no-store',
        },
      );
      if (response.status === 401) {
        disconnect('The gateway rejected that password.');
        return;
      }
      if (response.status === 204) return;
      if (!response.ok) throw new Error('Vision frame unavailable');
      const sequence = Number(
        response.headers.get('X-Dogzilla-Frame-Sequence'),
      );
      if (Number.isInteger(sequence) && sequence >= 0) {
        visionFrameSequence = sequence;
      }
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
      if (password) visionFrameTimer = setTimeout(refreshVisionFrame, 10);
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
    const proposals = Array.isArray(result?.action_proposals)
      ? result.action_proposals
      : [];
    const actionStatus = telemetry.vision_action_status;
    const actionValue = actionStatus?.value;
    const armed = Boolean(actionValue?.armed) && !actionStatus.stale;
    if (armed) {
      const state = String(actionValue.state || 'armed').replaceAll('-', ' ');
      elements['vision-safety'].className = 'vision-safety armed';
      elements['vision-safety'].textContent = `Action output: ARMED · ${state}`;
    } else {
      elements['vision-safety'].className = 'vision-safety';
      const objectStatus = statusValue?.object_detection;
      if (
        objectStatus?.ready
        && ['objects', 'dangerous-objects', 'floor-hazards', 'patrol'].includes(statusValue.mode)
      ) {
        const missing = objectStatus.missing_dangerous_classes || [];
        elements['vision-safety'].textContent = missing.length
          ? `Detection only · missing hazard classes: ${missing.join(', ')}`
          : 'Detection only · requested hazard classes covered';
      } else {
        elements['vision-safety'].textContent = (
          'Action output: disabled · proposals are never executed'
        );
      }
    }
    if (!detections.length) {
      elements['vision-result'].textContent = result?.mode
        ? `${String(result.mode).replaceAll('-', ' ')} · no target`
        : 'No detections';
    } else {
      const first = detections[0];
      if (first.kind === 'qr') {
        elements['vision-result'].textContent = `QR · ${first.text || '(empty)'}`;
      } else if (first.kind === 'object') {
        const floor = first.floor_candidate ? ' · floor' : '';
        const confidence = Number(first.confidence);
        const score = Number.isFinite(confidence) ? ` · ${(confidence * 100).toFixed(0)}%` : '';
        elements['vision-result'].textContent = `${first.label}${score}${floor} · ${first.risk}`;
      } else {
        const offset = Number.isFinite(first.error_x)
          ? ` · horizontal ${first.error_x.toFixed(2)}`
          : '';
        elements['vision-result'].textContent = `${first.kind}${offset}`;
      }
    }
    if (proposals.length) {
      const proposal = proposals[0];
      const state = armed ? 'guard evaluating' : 'disarmed';
      elements['vision-result'].textContent += ` · ${proposal.name} proposed (${state})`;
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

  function pointInPolygon(point, polygon) {
    let inside = false;
    for (let index = 0; index < polygon.length; index += 1) {
      const first = polygon[index];
      const second = polygon[(index + 1) % polygon.length];
      const cross = (point.x - first.x) * (second.y - first.y)
        - (point.y - first.y) * (second.x - first.x);
      const onEdge = Math.abs(cross) < 1e-9
        && point.x >= Math.min(first.x, second.x) - 1e-9
        && point.x <= Math.max(first.x, second.x) + 1e-9
        && point.y >= Math.min(first.y, second.y) - 1e-9
        && point.y <= Math.max(first.y, second.y) + 1e-9;
      if (onEdge) return true;
      if ((first.y > point.y) === (second.y > point.y)) continue;
      const crossingX = ((second.x - first.x) * (point.y - first.y))
        / (second.y - first.y) + first.x;
      if (point.x < crossingX) inside = !inside;
    }
    return inside;
  }

  function pointToPolygonDistance(point, polygon) {
    if (pointInPolygon(point, polygon)) return 0;
    return polygon.reduce((shortest, first, index) => {
      const second = polygon[(index + 1) % polygon.length];
      const edgeX = second.x - first.x;
      const edgeY = second.y - first.y;
      const lengthSquared = edgeX * edgeX + edgeY * edgeY;
      const projection = lengthSquared <= 1e-18
        ? 0
        : Math.max(0, Math.min(1, (
          ((point.x - first.x) * edgeX + (point.y - first.y) * edgeY)
          / lengthSquared
        )));
      const closestX = first.x + projection * edgeX;
      const closestY = first.y + projection * edgeY;
      return Math.min(shortest, Math.hypot(
        point.x - closestX,
        point.y - closestY,
      ));
    }, Number.POSITIVE_INFINITY);
  }

  function validateMapBoundaryPoint(point, label) {
    if (!mapSnapshot) return { valid: false, reason: 'Occupancy map is unavailable.' };
    const local = worldToLocal(point);
    if (!local || local.x < 0 || local.y < 0 || local.x >= mapSnapshot.width || local.y >= mapSnapshot.height) {
      return { valid: false, reason: `${label} is outside the active map.` };
    }
    return { valid: true, reason: 'Inside the active map.' };
  }

  function validateMapPoint(point, label = 'Waypoint') {
    if (!mapSnapshot || !mapCells) return { valid: false, reason: 'Occupancy map is unavailable.' };
    const local = worldToLocal(point);
    const column = Math.floor(local.x);
    const row = Math.floor(local.y);
    if (column < 0 || row < 0 || column >= mapSnapshot.width || row >= mapSnapshot.height) {
      return { valid: false, reason: `${label} is outside the active map.` };
    }
    const keepoutClearance = Number(mapSnapshot.keepout_clearance_m || 0);
    const keepout = keepoutZones.find((zone) => (
      pointToPolygonDistance(point, zone.polygon) <= keepoutClearance + 1e-9
    ));
    if (keepout) {
      return {
        valid: false,
        reason: `${label} is inside or too close to keepout zone “${keepout.name}”.`,
      };
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

  function drawPatrolPolygon(context) {
    const screenPoints = patrolPolygon.map(worldToScreen).filter(Boolean);
    if (!screenPoints.length) return;
    context.save();
    context.strokeStyle = '#c78cff';
    context.fillStyle = 'rgba(199, 140, 255, .12)';
    context.lineWidth = 2.5;
    context.beginPath();
    context.moveTo(screenPoints[0].x, screenPoints[0].y);
    screenPoints.slice(1).forEach((point) => context.lineTo(point.x, point.y));
    if (screenPoints.length >= 3) {
      context.closePath();
      context.fill();
    }
    context.stroke();
    screenPoints.forEach((point, index) => {
      context.beginPath();
      context.arc(point.x, point.y, 5, 0, Math.PI * 2);
      context.fillStyle = '#c78cff';
      context.fill();
      context.fillStyle = '#ffffff';
      context.font = '700 10px system-ui, sans-serif';
      context.fillText(String(index + 1), point.x + 7, point.y - 7);
    });
    context.restore();
  }

  function drawKeepoutPolygon(context, polygon, label, editing = false) {
    const screenPoints = polygon.map(worldToScreen).filter(Boolean);
    if (!screenPoints.length) return;
    context.save();
    context.strokeStyle = editing ? '#ff9b78' : '#ff725e';
    context.fillStyle = editing
      ? 'rgba(255, 114, 94, .24)'
      : 'rgba(255, 114, 94, .14)';
    context.lineWidth = editing ? 3 : 2;
    context.beginPath();
    context.moveTo(screenPoints[0].x, screenPoints[0].y);
    screenPoints.slice(1).forEach((point) => context.lineTo(point.x, point.y));
    if (screenPoints.length >= 3) {
      context.closePath();
      context.fill();
    }
    context.stroke();
    if (editing) {
      screenPoints.forEach((point, index) => {
        context.beginPath();
        context.arc(point.x, point.y, 5, 0, Math.PI * 2);
        context.fillStyle = '#ff9b78';
        context.fill();
        context.fillStyle = '#ffffff';
        context.font = '700 10px system-ui, sans-serif';
        context.fillText(String(index + 1), point.x + 7, point.y - 7);
      });
    }
    if (label && screenPoints.length >= 3) {
      const centre = screenPoints.reduce(
        (total, point) => ({ x: total.x + point.x, y: total.y + point.y }),
        { x: 0, y: 0 },
      );
      context.fillStyle = '#fff0eb';
      context.font = '700 10px system-ui, sans-serif';
      context.fillText(
        label,
        centre.x / screenPoints.length + 6,
        centre.y / screenPoints.length - 6,
      );
    }
    context.restore();
  }

  function drawKeepoutZones(context) {
    keepoutZones
      .filter((zone) => zone.id !== selectedKeepoutZoneId)
      .forEach((zone) => drawKeepoutPolygon(context, zone.polygon, zone.name));
    if (keepoutPolygon.length) {
      drawKeepoutPolygon(
        context,
        keepoutPolygon,
        elements['keepout-name'].value.trim(),
        true,
      );
    }
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
    const fitScale = Math.max(0.01, Math.min(
      (bounds.width - padding * 2) / mapSnapshot.width,
      (bounds.height - padding * 2) / mapSnapshot.height,
    ));
    const scale = fitScale * mapZoom;
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

    elements['map-zoom-level'].textContent = `${Math.round(mapZoom * 100)}%`;
    elements['map-zoom-out'].disabled = mapZoom <= 1;
    elements['map-zoom-in'].disabled = mapZoom >= 4;
    drawKeepoutZones(context);

    if (plannedPath.length > 1) {
      drawPolyline(context, plannedPath, '#5ef0a6');
    } else {
      const direct = [robotPose, waypoints.pickup, waypoints.dropoff].filter(Boolean);
      drawPolyline(context, direct, 'rgba(94, 240, 166, .55)', true);
    }
    drawPatrolPolygon(context);
    if (patrolWaypoints.length > 1) {
      drawPolyline(context, patrolWaypoints, '#c78cff', true);
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
    const normalizedPoint = {
      x: Number(point.x.toFixed(3)),
      y: Number(point.y.toFixed(3)),
    };
    waypoints[target] = { ...normalizedPoint, yaw: selectedYaw };
    elements[`${target}-x`].value = normalizedPoint.x.toFixed(3);
    elements[`${target}-y`].value = normalizedPoint.y.toFixed(3);
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
      const drawing = ['patrol', 'keepout'].includes(target);
      button.disabled = drawing;
      button.textContent = drawing
        ? 'Choose pickup or drop-off'
        : `Use as ${target === 'pickup' ? 'pickup' : 'drop-off'}`;
    });
    elements['use-robot-pose'].disabled = ['patrol', 'keepout'].includes(target) || !robotPose;
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

  function patrolAreaPayload() {
    return {
      map: currentMap,
      name: elements['patrol-name'].value.trim(),
      spacing_m: Number(elements['patrol-spacing'].value),
      polygon: patrolPolygon.map((point) => ({ x: point.x, y: point.y })),
    };
  }

  function clearPatrolPreview() {
    patrolWaypoints = [];
    drawMap();
  }

  function updatePatrolCount() {
    elements['patrol-count'].textContent = `${patrolPolygon.length} point${patrolPolygon.length === 1 ? '' : 's'}`;
    elements['patrol-undo'].disabled = !patrolPolygon.length;
    elements['patrol-clear'].disabled = !patrolPolygon.length;
  }

  function setPatrolStatus(message, success = false) {
    elements['patrol-status'].textContent = message;
    elements['patrol-status'].className = `form-message${success ? ' success' : ''}`;
  }

  async function previewPatrol(body = patrolAreaPayload()) {
    if (!body.patrol_area_id && patrolPolygon.length < 3) {
      throw new Error('Add at least three patrol boundary points first.');
    }
    const preview = await post('/api/v1/patrol-areas/preview', body);
    patrolWaypoints = Array.isArray(preview.waypoints) ? preview.waypoints : [];
    setPatrolStatus(
      `Safe sweep: ${preview.waypoint_count} points · ${Number(preview.coverage_distance_m).toFixed(2)} m`,
      true,
    );
    drawMap();
    return preview;
  }

  function selectPatrolArea(areaId) {
    const area = patrolAreas.find((item) => item.id === areaId);
    selectedPatrolAreaId = area?.id || '';
    elements['patrol-area-list'].value = selectedPatrolAreaId;
    elements['patrol-delete'].disabled = !area;
    if (!area) return;
    patrolPolygon = area.polygon.map((point) => ({ x: point.x, y: point.y }));
    elements['patrol-name'].value = area.name;
    elements['patrol-spacing'].value = area.spacing_m;
    clearPatrolPreview();
    updatePatrolCount();
    setPatrolStatus(`Loaded saved area “${area.name}”. Preview it before queuing.`, true);
  }

  async function refreshPatrolAreas(preferredId = selectedPatrolAreaId) {
    const response = await api('/api/v1/patrol-areas');
    patrolAreas = Array.isArray(response.patrol_areas) ? response.patrol_areas : [];
    const options = [new Option('Draw a new area', '')];
    patrolAreas.forEach((area) => options.push(new Option(area.name, area.id)));
    elements['patrol-area-list'].replaceChildren(...options);
    const selected = patrolAreas.some((area) => area.id === preferredId)
      ? preferredId
      : '';
    selectedPatrolAreaId = selected;
    elements['patrol-area-list'].value = selected;
    elements['patrol-delete'].disabled = !selected;
  }

  function keepoutZonePayload() {
    return {
      map: currentMap,
      name: elements['keepout-name'].value.trim(),
      polygon: keepoutPolygon.map((point) => ({ x: point.x, y: point.y })),
    };
  }

  function updateKeepoutCount() {
    const count = keepoutPolygon.length;
    elements['keepout-count'].textContent = `${count} point${count === 1 ? '' : 's'}`;
    elements['keepout-undo'].disabled = count === 0;
    elements['keepout-clear'].disabled = count === 0;
  }

  function setKeepoutStatus(message, success = false) {
    elements['keepout-status'].textContent = message;
    elements['keepout-status'].className = `form-message${success ? ' success' : ''}`;
  }

  function selectKeepoutZone(zoneId) {
    const zone = keepoutZones.find((item) => item.id === zoneId);
    selectedKeepoutZoneId = zone?.id || '';
    elements['keepout-zone-list'].value = selectedKeepoutZoneId;
    elements['keepout-delete'].disabled = !zone;
    if (!zone) return;
    keepoutPolygon = zone.polygon.map((point) => ({ x: point.x, y: point.y }));
    elements['keepout-name'].value = zone.name;
    updateKeepoutCount();
    setKeepoutStatus(`Loaded saved zone “${zone.name}”.`, true);
    drawMap();
  }

  async function refreshKeepoutZones(preferredId = selectedKeepoutZoneId) {
    const response = await api('/api/v1/keepout-zones');
    keepoutZones = Array.isArray(response.keepout_zones)
      ? response.keepout_zones
      : [];
    const options = [new Option('Draw a new zone', '')];
    keepoutZones.forEach((zone) => options.push(new Option(zone.name, zone.id)));
    elements['keepout-zone-list'].replaceChildren(...options);
    const selected = keepoutZones.some((zone) => zone.id === preferredId)
      ? preferredId
      : '';
    selectedKeepoutZoneId = selected;
    elements['keepout-zone-list'].value = selected;
    elements['keepout-delete'].disabled = !selected;
    syncAllWaypointValidation();
    drawMap();
  }

  function renderState(state) {
    setConnection(true);
    const telemetry = state.telemetry || {};
    renderVision(telemetry);
    const safety = state.safety || {};
    const robot = state.robot || {};
    const nextMap = state.configuration?.map || telemetry.map?.value?.name || currentMap;
    const mapChanged = nextMap !== currentMap;
    currentMap = nextMap;
    if (mapChanged) {
      mapSnapshot = null;
      mapCells = null;
      mapImage = null;
      keepoutPolygon = [];
      keepoutZones = [];
      selectedKeepoutZoneId = '';
      patrolPolygon = [];
      patrolWaypoints = [];
      patrolAreas = [];
      selectedPatrolAreaId = '';
      plannedPath = [];
      waypoints.pickup = null;
      waypoints.dropoff = null;
      setTimeout(() => {
        Promise.all([
          refreshMap(true),
          refreshLocations(),
          refreshPatrolAreas(),
          refreshKeepoutZones(),
          refreshAlerts(),
        ]).catch((error) => showToast(error.message));
      }, 0);
    }
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
      const points = active.payload?.waypoints?.length || 1;
      const repeats = Number(active.payload?.repeats || 1);
      const count = points * repeats;
      const step = Math.min(count, Number(active.current_step || 0));
      elements['active-task'].textContent = active.name;
      elements['active-task-detail'].textContent = `${active.state} · point ${Math.min(step + 1, count)} of ${count}`;
      elements['task-progress'].style.width = `${Math.round((step / count) * 100)}%`;
    } else {
      elements['active-task'].textContent = 'None';
      elements['active-task-detail'].textContent = 'Queue is idle';
      elements['task-progress'].style.width = '0%';
    }

    const latched = Boolean(safety.estop_latched);
    const autonomy = state.autonomy || {};
    const speedLevel = Number(autonomy.speed_level || 1);
    const turnLevel = Number(autonomy.turn_level || 1);
    elements['drive-speed'].value = String(speedLevel);
    elements['drive-turn'].value = String(turnLevel);
    elements['drive-speed-value'].value = String(speedLevel);
    elements['drive-turn-value'].value = String(turnLevel);
    const mapSwitchPending = Boolean(state.configuration?.map_switch_pending);
    const autonomyBlocked = latched || Boolean(active) || mapSwitchPending;
    elements['drive-state'].textContent = autonomyBlocked
      ? (latched ? 'e-stop' : (mapSwitchPending ? 'map switching' : 'task active'))
      : 'ready';
    elements['drive-speed'].disabled = autonomyBlocked;
    elements['drive-turn'].disabled = autonomyBlocked;
    elements['safety-panel'].classList.toggle('latched', latched);
    elements['safety-title'].textContent = latched ? 'Emergency stop latched' : 'Emergency stop ready';
    elements['safety-detail'].textContent = latched
      ? 'Autonomous movement is blocked until an operator resets it.'
      : 'Stops navigation and continuously commands zero velocity.';
    elements.estop.disabled = latched;
    elements['reset-estop'].classList.toggle('hidden', !latched);
    elements['submit-mission'].disabled = latched;
    elements['patrol-queue'].disabled = latched || !selectedPatrolAreaId;
    elements['use-robot-pose'].disabled = ['patrol', 'keepout'].includes(activeTarget) || !robotPose;
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
    const total = (task.payload?.waypoints?.length || 0) * Number(task.payload?.repeats || 1);
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
    const drawing = ['patrol', 'keepout'].includes(activeTarget);
    apply.disabled = drawing;
    apply.textContent = drawing
      ? 'Choose pickup or drop-off'
      : `Use as ${activeTarget === 'pickup' ? 'pickup' : 'drop-off'}`;
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

  async function loadAlertPhoto(alert) {
    if (!alert.photo_url) return '';
    if (alertImageUrls.has(alert.id)) return alertImageUrls.get(alert.id);
    const response = await fetch(alert.photo_url, {
      headers: { 'X-Dogzilla-Password': password },
      cache: 'no-store',
    });
    if (!response.ok) return '';
    const url = URL.createObjectURL(await response.blob());
    alertImageUrls.set(alert.id, url);
    return url;
  }

  function alertElement(alert, photoUrl) {
    const item = document.createElement('article');
    item.className = 'patrol-alert';
    if (photoUrl) {
      const image = document.createElement('img');
      image.src = photoUrl;
      image.alt = `${alert.category || 'patrol'} alert: ${alert.label || 'detection'}`;
      item.append(image);
    }
    const body = document.createElement('div');
    body.className = 'patrol-alert-body';
    const title = document.createElement('strong');
    title.textContent = `${alert.category || 'patrol'} · ${alert.label || 'detection'}`;
    const confidence = Number(alert.confidence);
    const detail = document.createElement('span');
    const score = Number.isFinite(confidence)
      ? `${Math.round(confidence * 100)}% confidence`
      : 'confirmed';
    const timestamp = new Date(alert.created_at || '');
    const timeText = Number.isFinite(timestamp.getTime())
      ? timestamp.toLocaleString()
      : 'time unavailable';
    detail.textContent = `${score} · ${timeText}`;
    body.append(title, detail);
    item.append(body);
    return item;
  }

  async function refreshAlerts() {
    const response = await api('/api/v1/alerts?limit=25');
    const alerts = Array.isArray(response.alerts) ? response.alerts : [];
    const activeIds = new Set(alerts.map((alert) => alert.id));
    alertImageUrls.forEach((url, id) => {
      if (!activeIds.has(id)) {
        URL.revokeObjectURL(url);
        alertImageUrls.delete(id);
      }
    });
    elements['patrol-alert-list'].replaceChildren();
    if (!alerts.length) {
      const empty = document.createElement('p');
      empty.className = 'empty-state';
      empty.textContent = 'No confirmed patrol sightings.';
      elements['patrol-alert-list'].append(empty);
      return;
    }
    const photos = await Promise.all(alerts.map(loadAlertPhoto));
    alerts.forEach((alert, index) => {
      elements['patrol-alert-list'].append(
        alertElement(alert, photos[index]),
      );
    });
  }

  async function refreshAll() {
    try {
      const [state] = await Promise.all([
        api('/api/v1/state'),
        refreshTasks(),
        refreshLocations(),
        refreshPatrolAreas(),
        refreshKeepoutZones(),
        refreshAlerts(),
      ]);
      renderState(state);
    } catch (error) {
      if (password) {
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

  async function updateAutonomousSpeed() {
    const speedLevel = Math.round(Number(elements['drive-speed'].value));
    const turnLevel = Math.round(Number(elements['drive-turn'].value));
    try {
      const settings = await post('/api/v1/autonomy/speed', {
        speed_level: speedLevel,
        turn_level: turnLevel,
      });
      elements['drive-message'].textContent = (
        `Autonomous walking ${settings.speed_level} · turning ${settings.turn_level}`
      );
    } catch (error) {
      elements['drive-message'].textContent = error.message;
      await refreshAll();
    }
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

  function handleGatewayEvent(record) {
    const dataLine = record.split('\n').find((line) => line.startsWith('data: '));
    if (!dataLine) return false;
    try {
      const event = JSON.parse(dataLine.slice(6));
      if (['hazard.confirmed', 'person.confirmed'].includes(event.type)) {
        const observation = event.data || {};
        const confidence = Number(observation.confidence);
        const confidenceText = Number.isFinite(confidence)
          ? ` at ${Math.round(confidence * 100)}% confidence`
          : '';
        const category = event.type === 'person.confirmed'
          ? 'Person detected'
          : 'Confirmed danger';
        const photo = observation.photo_url
          ? ' Photo saved.'
          : ' Photo unavailable.';
        showToast(`${category}: ${observation.label || 'object'}${confidenceText}.${photo}`);
      }
    } catch (_) {
      return false;
    }
    return true;
  }

  async function connectEvents() {
    if (eventAbort) eventAbort.abort();
    eventAbort = new AbortController();
    try {
      const response = await fetch('/api/v1/events', {
        headers: { 'X-Dogzilla-Password': password },
        cache: 'no-store',
        signal: eventAbort.signal,
      });
      if (!response.ok || !response.body) return;
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';
      while (password) {
        const { value, done } = await reader.read();
        if (done) break;
        buffer += decoder.decode(value, { stream: true });
        const records = buffer.split('\n\n');
        buffer = records.pop() || '';
        if (records.map(handleGatewayEvent).some(Boolean)) await refreshAll();
      }
      if (password) setTimeout(connectEvents, 1000);
    } catch (error) {
      if (error.name !== 'AbortError' && password) setTimeout(connectEvents, 3000);
    }
  }

  targetButtons.forEach((button) => {
    button.addEventListener('click', () => setActiveTarget(button.dataset.mapTarget));
  });

  elements['drive-speed'].addEventListener('input', () => {
    elements['drive-speed-value'].value = elements['drive-speed'].value;
  });
  elements['drive-turn'].addEventListener('input', () => {
    elements['drive-turn-value'].value = elements['drive-turn'].value;
  });
  elements['drive-speed'].addEventListener('change', updateAutonomousSpeed);
  elements['drive-turn'].addEventListener('change', updateAutonomousSpeed);
  elements['map-zoom-out'].addEventListener('click', () => {
    mapZoom = Math.max(1, mapZoom - 0.5);
    drawMap();
  });
  elements['map-zoom-in'].addEventListener('click', () => {
    mapZoom = Math.min(4, mapZoom + 0.5);
    drawMap();
  });
  elements['map-zoom-fit'].addEventListener('click', () => {
    mapZoom = 1;
    drawMap();
  });

  ['pickup', 'dropoff'].forEach((target) => {
    ['x', 'y', 'yaw'].forEach((field) => {
      elements[`${target}-${field}`].addEventListener('input', () => syncWaypointFromInputs(target));
    });
  });

  elements['map-canvas'].addEventListener('click', (event) => {
    const point = eventToWorld(event);
    if (!point) return;
    if (activeTarget === 'keepout') {
      if (keepoutPolygon.length >= 24) {
        setKeepoutStatus('A keepout zone can contain at most twenty-four points.');
        return;
      }
      const validation = validateMapBoundaryPoint(point, 'Keepout boundary point');
      if (!validation.valid) {
        setKeepoutStatus(validation.reason);
        return;
      }
      if (keepoutPolygon.some((item) => Math.hypot(item.x - point.x, item.y - point.y) < 0.05)) {
        setKeepoutStatus('Keepout boundary points must be at least 0.05 m apart.');
        return;
      }
      keepoutPolygon.push({ x: point.x, y: point.y });
      selectedKeepoutZoneId = '';
      elements['keepout-zone-list'].value = '';
      elements['keepout-delete'].disabled = true;
      updateKeepoutCount();
      setKeepoutStatus(
        keepoutPolygon.length < 3
          ? `Add ${3 - keepoutPolygon.length} more point${keepoutPolygon.length === 2 ? '' : 's'}.`
          : 'Zone ready to save. Add more points if needed.',
        keepoutPolygon.length >= 3,
      );
      drawMap();
      event.preventDefault();
      return;
    }
    if (activeTarget === 'patrol') {
      if (patrolPolygon.length >= 12) {
        setPatrolStatus('A patrol area can contain at most twelve points.');
        return;
      }
      const validation = validateMapPoint(point, 'Patrol boundary point');
      if (!validation.valid) {
        setPatrolStatus(validation.reason);
        return;
      }
      if (patrolPolygon.some((item) => Math.hypot(item.x - point.x, item.y - point.y) < 0.05)) {
        setPatrolStatus('Patrol boundary points must be at least 0.05 m apart.');
        return;
      }
      patrolPolygon.push({ x: point.x, y: point.y });
      selectedPatrolAreaId = '';
      elements['patrol-area-list'].value = '';
      elements['patrol-delete'].disabled = true;
      clearPatrolPreview();
      updatePatrolCount();
      setPatrolStatus(
        patrolPolygon.length < 3
          ? `Add ${3 - patrolPolygon.length} more point${patrolPolygon.length === 2 ? '' : 's'}.`
          : 'Area ready to preview. Add more points or preview the sweep.',
        patrolPolygon.length >= 3,
      );
      event.preventDefault();
      return;
    }
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
    if (!robotPose || ['patrol', 'keepout'].includes(activeTarget)) return;
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

  elements['patrol-undo'].addEventListener('click', () => {
    patrolPolygon.pop();
    selectedPatrolAreaId = '';
    elements['patrol-area-list'].value = '';
    elements['patrol-delete'].disabled = true;
    clearPatrolPreview();
    updatePatrolCount();
    setPatrolStatus('Last patrol boundary point removed.');
  });

  elements['patrol-clear'].addEventListener('click', () => {
    patrolPolygon = [];
    patrolWaypoints = [];
    selectedPatrolAreaId = '';
    elements['patrol-area-list'].value = '';
    elements['patrol-delete'].disabled = true;
    updatePatrolCount();
    setPatrolStatus('Choose Draw patrol area, then click at least three points on the map.');
    drawMap();
  });

  elements['patrol-area-list'].addEventListener('change', async (event) => {
    selectPatrolArea(event.currentTarget.value);
    if (!selectedPatrolAreaId) return;
    try {
      await previewPatrol({ patrol_area_id: selectedPatrolAreaId });
    } catch (error) { setPatrolStatus(error.message); }
  });

  ['patrol-name', 'patrol-spacing'].forEach((id) => {
    elements[id].addEventListener('input', () => {
      if (!selectedPatrolAreaId) return;
      selectedPatrolAreaId = '';
      elements['patrol-area-list'].value = '';
      elements['patrol-delete'].disabled = true;
      clearPatrolPreview();
      setPatrolStatus('Area settings changed. Preview and save this version.');
    });
  });

  elements['patrol-preview-button'].addEventListener('click', async () => {
    try {
      const body = selectedPatrolAreaId
        ? { patrol_area_id: selectedPatrolAreaId }
        : patrolAreaPayload();
      await previewPatrol(body);
    } catch (error) { setPatrolStatus(error.message); }
  });

  elements['patrol-save'].addEventListener('click', async () => {
    try {
      const area = await post('/api/v1/patrol-areas', patrolAreaPayload());
      selectedPatrolAreaId = area.id;
      await refreshPatrolAreas(area.id);
      await previewPatrol({ patrol_area_id: area.id });
      showToast(`Patrol area “${area.name}” saved.`);
    } catch (error) { setPatrolStatus(error.message); }
  });

  elements['patrol-queue'].addEventListener('click', async () => {
    if (!selectedPatrolAreaId) {
      setPatrolStatus('Save the patrol area before queuing it.');
      return;
    }
    try {
      const task = await post('/api/v1/tasks/patrol', {
        patrol_area_id: selectedPatrolAreaId,
        name: elements['patrol-name'].value.trim(),
        repeats: Number(elements['patrol-repeats'].value),
        dwell_seconds: Number(elements['patrol-dwell'].value),
      });
      setPatrolStatus(`Queued ${task.name}.`, true);
      showToast('Patrol added to the mission queue.');
      await refreshAll();
    } catch (error) { setPatrolStatus(error.message); }
  });

  elements['patrol-delete'].addEventListener('click', async () => {
    const area = patrolAreas.find((item) => item.id === selectedPatrolAreaId);
    if (!area || !window.confirm(`Delete patrol area “${area.name}”?`)) return;
    try {
      await api(`/api/v1/patrol-areas/${encodeURIComponent(area.id)}`, { method: 'DELETE' });
      patrolPolygon = [];
      patrolWaypoints = [];
      selectedPatrolAreaId = '';
      await refreshPatrolAreas();
      updatePatrolCount();
      setPatrolStatus('Saved patrol area deleted.');
      drawMap();
    } catch (error) { setPatrolStatus(error.message); }
  });

  elements['keepout-undo'].addEventListener('click', () => {
    keepoutPolygon.pop();
    selectedKeepoutZoneId = '';
    elements['keepout-zone-list'].value = '';
    elements['keepout-delete'].disabled = true;
    updateKeepoutCount();
    setKeepoutStatus('Last keepout boundary point removed.');
    drawMap();
  });

  elements['keepout-clear'].addEventListener('click', () => {
    keepoutPolygon = [];
    selectedKeepoutZoneId = '';
    elements['keepout-zone-list'].value = '';
    elements['keepout-delete'].disabled = true;
    updateKeepoutCount();
    setKeepoutStatus('Choose Draw keepout, then click at least three points around the obstacle.');
    drawMap();
  });

  elements['keepout-zone-list'].addEventListener('change', (event) => {
    if (!event.currentTarget.value) {
      keepoutPolygon = [];
      selectedKeepoutZoneId = '';
      elements['keepout-delete'].disabled = true;
      updateKeepoutCount();
      setKeepoutStatus('Draw a new zone with at least three map clicks.');
      drawMap();
      return;
    }
    selectKeepoutZone(event.currentTarget.value);
  });

  elements['keepout-name'].addEventListener('input', () => {
    if (!selectedKeepoutZoneId) return;
    selectedKeepoutZoneId = '';
    elements['keepout-zone-list'].value = '';
    elements['keepout-delete'].disabled = true;
    setKeepoutStatus('Zone name changed. Saving will create this named version.');
    drawMap();
  });

  elements['keepout-save'].addEventListener('click', async () => {
    if (keepoutPolygon.length < 3) {
      setKeepoutStatus('Add at least three keepout boundary points first.');
      return;
    }
    try {
      const zone = await post('/api/v1/keepout-zones', keepoutZonePayload());
      selectedKeepoutZoneId = zone.id;
      await refreshKeepoutZones(zone.id);
      selectKeepoutZone(zone.id);
      setKeepoutStatus(`Keepout zone “${zone.name}” is active in Nav2.`, true);
      showToast(`Keepout zone “${zone.name}” saved.`);
    } catch (error) { setKeepoutStatus(error.message); }
  });

  elements['keepout-delete'].addEventListener('click', async () => {
    const zone = keepoutZones.find((item) => item.id === selectedKeepoutZoneId);
    if (!zone || !window.confirm(`Delete keepout zone “${zone.name}”?`)) return;
    try {
      await api(`/api/v1/keepout-zones/${encodeURIComponent(zone.id)}`, { method: 'DELETE' });
      keepoutPolygon = [];
      selectedKeepoutZoneId = '';
      await refreshKeepoutZones();
      updateKeepoutCount();
      setKeepoutStatus('Saved keepout zone deleted. Nav2 mask updated.');
      showToast('Keepout zone deleted.');
    } catch (error) { setKeepoutStatus(error.message); }
  });

  elements['save-location'].addEventListener('click', async () => {
    const name = elements['location-name'].value.trim();
    const waypointValue = ['patrol', 'keepout'].includes(activeTarget)
      ? null
      : waypoints[activeTarget];
    if (!name) { showToast('Enter a location name first.'); return; }
    if (!waypointValue) { showToast('Choose and select a pickup or drop-off waypoint first.'); return; }
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
    password = elements.password.value.trim();
    elements['login-error'].textContent = '';
    try {
      const state = await api('/api/v1/state');
      sessionStorage.setItem('dogzillaGatewayPassword', password);
      elements.login.classList.add('hidden');
      elements.app.classList.remove('hidden');
      renderState(state);
      await Promise.all([
        refreshTasks(),
        refreshLocations(),
        refreshPatrolAreas(),
        refreshKeepoutZones(),
        refreshAlerts(),
      ]);
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
  elements.logout.addEventListener('click', () => {
    disconnect();
  });

  updatePatrolCount();
  updateKeepoutCount();

  if (password) {
    elements.password.value = password;
    elements['login-form'].requestSubmit();
  }
})();
