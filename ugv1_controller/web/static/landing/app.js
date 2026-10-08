(function () {
  var SVGNS = 'http://www.w3.org/2000/svg';

  // ---------------------------------------------------------------------
  // Diagram data. status: 'built' or 'planned'.
  // Edge anchors: fs / ts = side of the source / target block (l, r, t, b);
  // fy / ty = position along that side, 0 to 1 (default 0.5).
  // ---------------------------------------------------------------------
  var DIAGRAMS = {
    hardware: {
      w: 1000, h: 440,
      nodes: [
        { id: 'cam', x: 20, y: 130, w: 200, h: 64, label: 'Arducam camera', sub: 'IMX477 - CSI - motor focus', status: 'built',
          desc: 'Arducam IMX477 with a motorised focus lens (VCM) on the Pi CSI port. Frames are captured with rpicam-vid at 640x480 and 30 fps.',
          items: ['Lens position is a capture_node parameter, 0.0 to 10.0', 'Needs the custom Arducam device-tree overlay on Ubuntu 24.04', 'Intrinsics come from the Kalibr calibration'] },
        { id: 'imu', x: 20, y: 230, w: 200, h: 64, label: 'IMU', sub: 'ICG-20660L - I2C 0x69', status: 'built',
          desc: 'Six-axis accelerometer and gyroscope on the Pi I2C bus, read by a raw register-level driver with no third-party IMU library.',
          items: ['Published at 100 Hz', 'Raw samples are never smoothed, to keep the camera-IMU time alignment intact', 'Gyro scale is halved when WHO_AM_I reads 0x91'] },
        { id: 'lidar', x: 20, y: 330, w: 200, h: 64, label: '2D LiDAR', sub: 'RPLidar - USB', status: 'planned',
          desc: 'A 2D LiDAR is the next sensor to be integrated. It will feed RTAB-Map and Nav2 alongside the visual-inertial odometry.',
          items: ['Driver: ros-jazzy-rplidar-ros', 'Not yet wired into the system'] },
        { id: 'browser', x: 390, y: 20, w: 220, h: 56, label: 'Browser on the LAN', sub: 'http://<pi-ip>:8080', status: 'built',
          desc: 'Any device on the same network can open the web server hosted on the Pi to view the camera feeds and this page.',
          items: ['Landing page at /', 'Live camera viewer at /processed-feed'] },
        { id: 'pi', x: 380, y: 130, w: 240, h: 110, label: 'Raspberry Pi 4', sub: 'Ubuntu 24.04 - ROS 2 Jazzy', status: 'built',
          desc: 'The main computer. It runs every ROS 2 node, each in its own tmux session, started by scripts/start_ugv1.sh.',
          items: ['4 GB RAM, so OpenVINS is compiled on a separate ARM64 cloud VM and copied over', 'Visualisation (RViz2) runs on another machine over ROS 2 static-peer discovery', 'Temperature at full load is about 58 C without a heatsink'] },
        { id: 'batt', x: 390, y: 330, w: 220, h: 64, label: 'Battery pack', sub: 'Powers the Pi and motors', status: 'built',
          desc: 'Supplies the Raspberry Pi and the motor driver. The pack chemistry and regulation are not documented in the repository.',
          items: ['Packs without proper BMS protection are unsafe for unattended robots'] },
        { id: 'ftdi', x: 780, y: 30, w: 200, h: 64, label: 'FT232RL', sub: 'USB to serial - 9600 baud', status: 'built',
          desc: 'USB-to-serial adapter between the Pi and the Arduino Nano, appearing as /dev/ttyUSB0.',
          items: ['Carries the custom 15-byte motor packets'] },
        { id: 'nano', x: 780, y: 130, w: 200, h: 64, label: 'Arduino Nano', sub: 'Motor controller (AVR)', status: 'built',
          desc: 'Receives framed motor commands over UART and drives the H-bridge with PWM speed and direction signals.',
          items: ['Packet: start byte 0xAA, then 4 speeds and 4 direction states', 'Built with avr-gcc, avr-libc and avrdude directly'] },
        { id: 'hbridge', x: 780, y: 230, w: 200, h: 64, label: 'L293D H-bridge', sub: '4 motor channels', status: 'built',
          desc: 'Dual H-bridge driver stage that switches motor power for the four drive motors.',
          items: ['Speed and direction set by the Arduino Nano'] },
        { id: 'motors', x: 780, y: 330, w: 200, h: 64, label: 'DC drive motors', sub: '4 wheels - FL FR BL BR', status: 'built',
          desc: 'Four DC motors, one per wheel: M1 front-left, M2 front-right, M3 back-right, M4 back-left.',
          items: ['M1 direction is inverted in software to correct a wiring reversal'] }
      ],
      edges: [
        { from: 'cam', to: 'pi', fs: 'r', ts: 'l', fy: .5, ty: .3, label: 'CSI' },
        { from: 'imu', to: 'pi', fs: 'r', ts: 'l', fy: .5, ty: .65, label: 'I2C' },
        { from: 'lidar', to: 'pi', fs: 'r', ts: 'l', fy: .5, ty: .9, label: 'USB', planned: true },
        { from: 'pi', to: 'browser', fs: 't', ts: 'b', label: 'WiFi / HTTP', both: true },
        { from: 'pi', to: 'ftdi', fs: 'r', ts: 'l', fy: .25, ty: .5, label: 'USB' },
        { from: 'ftdi', to: 'nano', fs: 'b', ts: 't', label: 'UART' },
        { from: 'nano', to: 'hbridge', fs: 'b', ts: 't', label: 'PWM + dir' },
        { from: 'hbridge', to: 'motors', fs: 'b', ts: 't', label: 'motor power' },
        { from: 'batt', to: 'pi', fs: 't', ts: 'b', label: 'power' },
        { from: 'batt', to: 'hbridge', fs: 'r', ts: 'l', fy: .5, ty: .6, label: 'motor supply' }
      ]
    },

    software: {
      w: 1120, h: 440,
      nodes: [
        { id: 'capture', x: 20, y: 30, w: 170, h: 60, label: 'capture_node', sub: 'rpicam-vid to JPEG', status: 'built',
          desc: 'Captures frames from the camera with rpicam-vid and publishes them as compressed images.',
          items: ['Publishes /video_raw (CompressedImage) at 15 fps', 'With publish_raw:=true also publishes /video_raw_raw (Image) for OpenVINS', 'Parameter lens_position sets the motorised focus'] },
        { id: 'imu', x: 20, y: 170, w: 170, h: 60, label: 'imu_node', sub: 'raw I2C driver', status: 'built',
          desc: 'Reads the IMU registers directly and publishes standard ROS 2 messages.',
          items: ['Publishes /imu/data_raw (sensor_msgs/Imu) at 100 Hz', 'Sensor QoS: best effort, keep last, depth 1', 'Start-up parameters hold the pre-loaded bias values'] },
        { id: 'lidar', x: 20, y: 290, w: 170, h: 60, label: 'rplidar_ros', sub: 'LiDAR driver', status: 'planned',
          desc: 'Driver node for the 2D LiDAR. Not yet integrated.',
          items: ['Will publish /scan for RTAB-Map'] },
        { id: 'process', x: 310, y: 30, w: 170, h: 60, label: 'process_node', sub: 'OpenCV processing', status: 'built',
          desc: 'Subscribes to /video_raw, applies the selected OpenCV mode and republishes the result.',
          items: ['Modes: raw, gray, edges, overlay', 'Publishes /video_processed (CompressedImage)', 'Change live with: ros2 param set /process_node mode overlay'] },
        { id: 'openvins', x: 310, y: 130, w: 170, h: 70, label: 'OpenVINS', sub: 'ov_msckf - VIO', status: 'built',
          desc: 'Multi-state constraint Kalman filter fusing camera and IMU into odometry.',
          items: ['Inputs: /video_raw_raw and /imu/data_raw', 'Output: /odomimu (nav_msgs/Odometry), about 100 Hz once the vehicle has moved', 'Camera-IMU extrinsics and time offset come from the Kalibr calibration', 'Zero-velocity updates enabled; online calibration disabled'] },
        { id: 'viewer', x: 600, y: 30, w: 190, h: 60, label: 'viewer_node', sub: 'Flask web server :8080', status: 'built',
          desc: 'Serves this page and the live camera viewer, and relays processing-mode changes back to the ROS 2 nodes.',
          items: ['Subscribes to /video_raw and /video_processed', 'Streams both as MJPEG', 'Sets the process_node mode parameter through ros2 param set'] },
        { id: 'browser', x: 910, y: 30, w: 190, h: 60, label: 'Browser', sub: '/processed-feed', status: 'built',
          desc: 'The operator opens the viewer from any device on the same network.',
          items: ['Side-by-side raw and processed feeds', 'Mode buttons for the processing pipeline'] },
        { id: 'rtabmap', x: 600, y: 220, w: 190, h: 60, label: 'RTAB-Map', sub: 'SLAM and mapping', status: 'planned',
          desc: 'Appearance-based SLAM that will combine the LiDAR scan with the visual-inertial odometry.',
          items: ['Package: ros-jazzy-rtabmap-ros', 'Needs OpenVINS odometry wired into the TF tree first'] },
        { id: 'nav2', x: 910, y: 220, w: 190, h: 60, label: 'Nav2', sub: 'Planner and controller', status: 'planned',
          desc: 'Navigation stack that will plan paths on the map and command the drivetrain.',
          items: ['Will publish /cmd_vel (geometry_msgs/Twist)'] },
        { id: 'motor', x: 910, y: 350, w: 190, h: 60, label: 'motor_control_node', sub: 'cmd_vel to UART', status: 'built',
          desc: 'Converts velocity commands into the custom framed UART packets understood by the Arduino Nano.',
          items: ['Subscribes to /cmd_vel', 'Serial port /dev/ttyUSB0 at 9600 baud', 'Packet struct format <4B4h, start byte 0xAA'] }
      ],
      edges: [
        { from: 'capture', to: 'process', fs: 'r', ts: 'l', label: '/video_raw' },
        { from: 'capture', to: 'openvins', fs: 'b', ts: 'l', fy: .5, ty: .3, label: '/video_raw_raw' },
        { from: 'imu', to: 'openvins', fs: 'r', ts: 'l', fy: .5, ty: .75, label: '/imu/data_raw' },
        { from: 'process', to: 'viewer', fs: 'r', ts: 'l', label: '/video_processed' },
        { from: 'viewer', to: 'browser', fs: 'r', ts: 'l', label: 'HTTP MJPEG', both: true },
        { from: 'openvins', to: 'rtabmap', fs: 'r', ts: 'l', fy: .5, ty: .3, label: '/odomimu', planned: true },
        { from: 'lidar', to: 'rtabmap', fs: 'r', ts: 'l', fy: .5, ty: .75, label: '/scan', planned: true },
        { from: 'rtabmap', to: 'nav2', fs: 'r', ts: 'l', label: '/map', planned: true },
        { from: 'nav2', to: 'motor', fs: 'b', ts: 't', label: '/cmd_vel', planned: true }
      ]
    }
  };

  var detailEl = document.getElementById('detail');

  function el(name, attrs, parent) {
    var e = document.createElementNS(SVGNS, name);
    for (var k in attrs) { e.setAttribute(k, attrs[k]); }
    if (parent) { parent.appendChild(e); }
    return e;
  }

  function anchor(n, side, frac) {
    if (frac === undefined) { frac = 0.5; }
    if (side === 'l') { return { x: n.x, y: n.y + n.h * frac, dx: -1, dy: 0 }; }
    if (side === 'r') { return { x: n.x + n.w, y: n.y + n.h * frac, dx: 1, dy: 0 }; }
    if (side === 't') { return { x: n.x + n.w * frac, y: n.y, dx: 0, dy: -1 }; }
    return { x: n.x + n.w * frac, y: n.y + n.h, dx: 0, dy: 1 };
  }

  function buildDiagram(key, container) {
    var spec = DIAGRAMS[key];
    var byId = {};
    spec.nodes.forEach(function (n) { byId[n.id] = n; });

    var svg = el('svg', { viewBox: '0 0 ' + spec.w + ' ' + spec.h, role: 'img',
                          'aria-label': key + ' architecture diagram' });
    var defs = el('defs', {}, svg);
    ['arr', 'arr-hl', 'arr-pl'].forEach(function (id) {
      var color = id === 'arr' ? '#52606d' : (id === 'arr-hl' ? '#00e5a0' : '#e0a030');
      var m = el('marker', { id: key + '-' + id, viewBox: '0 0 10 10', refX: 9, refY: 5,
                             markerWidth: 7, markerHeight: 7, orient: 'auto-start-reverse' }, defs);
      el('path', { d: 'M0,0 L10,5 L0,10 z', fill: color }, m);
    });

    var edgeEls = [];
    spec.edges.forEach(function (e) {
      var a = anchor(byId[e.from], e.fs, e.fy);
      var b = anchor(byId[e.to], e.ts, e.ty);
      var dist = Math.max(30, Math.min(80, Math.hypot(b.x - a.x, b.y - a.y) / 2.5));
      var c1 = { x: a.x + a.dx * dist, y: a.y + a.dy * dist };
      var c2 = { x: b.x + b.dx * dist, y: b.y + b.dy * dist };
      var g = el('g', { 'class': 'edge' + (e.planned ? ' planned' : '') }, svg);
      var path = el('path', {
        d: 'M' + a.x + ',' + a.y + ' C' + c1.x + ',' + c1.y + ' ' + c2.x + ',' + c2.y + ' ' + b.x + ',' + b.y,
        'marker-end': 'url(#' + key + '-' + (e.planned ? 'arr-pl' : 'arr') + ')'
      }, g);
      if (e.both) { path.setAttribute('marker-start', 'url(#' + key + '-' + (e.planned ? 'arr-pl' : 'arr') + ')'); }
      var mx = (a.x + 3 * c1.x + 3 * c2.x + b.x) / 8;
      var my = (a.y + 3 * c1.y + 3 * c2.y + b.y) / 8;
      var t = el('text', { x: mx, y: my - 5 }, g);
      t.textContent = e.label;
      edgeEls.push({ g: g, path: path, spec: e });
    });

    var nodeEls = {};
    spec.nodes.forEach(function (n) {
      var g = el('g', { 'class': 'node' + (n.status === 'planned' ? ' planned' : ''),
                        tabindex: 0, role: 'button',
                        'aria-label': n.label + ', ' + n.status }, svg);
      el('rect', { x: n.x, y: n.y, width: n.w, height: n.h }, g);
      var l = el('text', { 'class': 'l', x: n.x + n.w / 2, y: n.y + n.h / 2 - 3 }, g);
      l.textContent = n.label;
      var s = el('text', { 'class': 's', x: n.x + n.w / 2, y: n.y + n.h / 2 + 14 }, g);
      s.textContent = n.sub;
      nodeEls[n.id] = g;
    });

    var selected = null;

    function connected(id) {
      var ids = {};
      ids[id] = true;
      edgeEls.forEach(function (e) {
        if (e.spec.from === id || e.spec.to === id) { ids[e.spec.from] = true; ids[e.spec.to] = true; }
      });
      return ids;
    }

    function highlight(id) {
      var ids = id ? connected(id) : null;
      edgeEls.forEach(function (e) {
        var on = id && (e.spec.from === id || e.spec.to === id);
        e.g.classList.toggle('hl', !!on);
        e.g.classList.toggle('dim', !!id && !on);
        var marker = on ? 'arr-hl' : (e.spec.planned ? 'arr-pl' : 'arr');
        e.path.setAttribute('marker-end', 'url(#' + key + '-' + marker + ')');
        if (e.spec.both) { e.path.setAttribute('marker-start', 'url(#' + key + '-' + marker + ')'); }
      });
      Object.keys(nodeEls).forEach(function (nid) {
        nodeEls[nid].classList.toggle('dim', !!ids && !ids[nid]);
      });
    }

    function showDetail(n) {
      detailEl.innerHTML = '';
      var h = document.createElement('h3');
      h.textContent = n.label + ' ';
      var pill = document.createElement('span');
      pill.className = 'pill' + (n.status === 'planned' ? ' planned' : '');
      pill.textContent = n.status === 'planned' ? 'Planned' : 'Implemented';
      h.appendChild(pill);
      detailEl.appendChild(h);
      var p = document.createElement('p');
      p.textContent = n.desc;
      detailEl.appendChild(p);
      var ul = document.createElement('ul');
      n.items.forEach(function (it) {
        var li = document.createElement('li');
        li.textContent = it;
        ul.appendChild(li);
      });
      detailEl.appendChild(ul);
    }

    function select(n) {
      selected = n.id;
      Object.keys(nodeEls).forEach(function (nid) { nodeEls[nid].classList.toggle('sel', nid === n.id); });
      highlight(n.id);
      showDetail(n);
    }

    spec.nodes.forEach(function (n) {
      var g = nodeEls[n.id];
      g.addEventListener('mouseenter', function () { highlight(n.id); });
      g.addEventListener('mouseleave', function () { highlight(selected); });
      g.addEventListener('focus', function () { highlight(n.id); });
      g.addEventListener('blur', function () { highlight(selected); });
      g.addEventListener('click', function () { select(n); });
      g.addEventListener('keydown', function (ev) {
        if (ev.key === 'Enter' || ev.key === ' ') { ev.preventDefault(); select(n); }
      });
    });

    container.appendChild(svg);
    return { reset: function () {
      selected = null;
      Object.keys(nodeEls).forEach(function (nid) { nodeEls[nid].classList.remove('sel'); });
      highlight(null);
    } };
  }

  var views = {
    hardware: buildDiagram('hardware', document.getElementById('diagram-hardware')),
    software: buildDiagram('software', document.getElementById('diagram-software'))
  };

  var HINT = 'Click any block to see details. Hover a block to highlight its connections.';

  document.querySelectorAll('.tab').forEach(function (tab) {
    tab.addEventListener('click', function () {
      var target = tab.getAttribute('data-target');
      Object.keys(views).forEach(function (k) {
        document.getElementById('diagram-' + k).hidden = (k !== target);
        document.getElementById('tab-' + k).setAttribute('aria-selected', k === target ? 'true' : 'false');
        views[k].reset();
      });
      detailEl.innerHTML = '';
      var p = document.createElement('p');
      p.className = 'hint';
      p.textContent = HINT;
      detailEl.appendChild(p);
    });
  });
})();
