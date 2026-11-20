/* Local WebGL renderer for the supplied binary STL meshes. */
"use strict";

function meshVertices(buffer) {
  const data = new DataView(buffer);
  const triangleCount = data.getUint32(80, true);
  if (84 + triangleCount * 50 !== buffer.byteLength) throw Error("STL 长度不符");
  const vertices = new Float32Array(triangleCount * 18);
  for (let face = 0; face < triangleCount; face++) {
    const start = 84 + face * 50;
    const point = offset => [0, 4, 8].map(i => data.getFloat32(start + offset + i, true));
    const a = point(12), b = point(24), c = point(36);
    const u = b.map((x, i) => x - a[i]), v = c.map((x, i) => x - a[i]);
    const normal = [u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0]];
    const length = Math.hypot(...normal);
    const n = length ? normal.map(x => x / length) : [0, 0, 1];
    [a, b, c].forEach((vertex, i) => vertices.set([...vertex, ...n], face * 18 + i * 6));
  }
  return vertices;
}

async function loadA10Meshes(canvas) {
  const response = await fetch("/model");
  if (!response.ok) throw Error("读取模型清单失败");
  const manifest = await response.json();
  if (!manifest) return null;
  const gl = canvas.getContext("webgl", { alpha: true, antialias: true });
  if (!gl) throw Error("浏览器未启用 WebGL");

  function shader(type, source) {
    const item = gl.createShader(type);
    gl.shaderSource(item, source);
    gl.compileShader(item);
    if (!gl.getShaderParameter(item, gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(item));
    return item;
  }

  const program = gl.createProgram();
  gl.attachShader(program, shader(gl.VERTEX_SHADER, `
    attribute vec3 vertex, normal;
    uniform vec3 position, scale, color;
    uniform vec4 rotation, camera;
    uniform vec2 viewport;
    varying vec3 shade;
    vec3 rotate(vec3 p) {
      vec3 d = 2.0 * cross(rotation.xyz, p);
      return p + rotation.w * d + cross(rotation.xyz, d);
    }
    void main() {
      vec3 p = rotate(vertex * scale) + position;
      float forward = dot(p.xy, camera.yx);
      float right = dot(p.xy, vec2(camera.x, -camera.y));
      float height = p.z - 0.30;
      gl_Position = vec4(right * viewport.x,
        (height * camera.z + forward * camera.w) * viewport.y - 0.22,
        (forward * camera.z - height * camera.w) * 0.12, 1.0);
      if (dot(normal, normal) < 0.5) {
        shade = color;
      } else {
        vec3 n = normalize(rotate(normal / scale));
        vec3 key = vec3(-0.507, -0.608, 0.608);
        vec3 fill = vec3(0.674, 0.674, 0.303);
        shade = color * (0.36 + 0.54 * max(dot(n, key), 0.0) + 0.26 * max(dot(n, fill), 0.0));
      }
    }`));
  gl.attachShader(program, shader(gl.FRAGMENT_SHADER, `
    precision mediump float;
    varying vec3 shade;
    uniform float feedback;
    void main() { gl_FragColor = vec4(shade * feedback, 1.0); }
  `));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);
  gl.enable(gl.DEPTH_TEST);
  const vertex = gl.getAttribLocation(program, "vertex");
  const normal = gl.getAttribLocation(program, "normal");
  gl.enableVertexAttribArray(vertex);
  gl.enableVertexAttribArray(normal);
  const uniforms = Object.fromEntries(["position", "scale", "color", "rotation", "camera", "viewport", "feedback"]
    .map(key => [key, gl.getUniformLocation(program, key)]));

  function upload(vertices) {
    const buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW);
    return { buffer, count: vertices.length / 6 };
  }
  const models = await Promise.all(manifest.map(async item => {
    const response = await fetch(item.url);
    if (!response.ok) throw Error("读取 STL 失败");
    return { ...upload(meshVertices(await response.arrayBuffer())), color: item.color, scale: item.scale };
  }));

  const lines = [];
  for (let i = -6; i <= 6; i++) {
    const k = 0.35 * i;
    lines.push(k, -2.1, 0, 0, 0, 0, k, 2.1, 0, 0, 0, 0,
      -2.1, k, 0, 0, 0, 0, 2.1, k, 0, 0, 0, 0);
  }
  const grid = upload(new Float32Array(lines));

  function draw(item, pose, offset, scale, color, mode) {
    gl.bindBuffer(gl.ARRAY_BUFFER, item.buffer);
    gl.vertexAttribPointer(vertex, 3, gl.FLOAT, false, 24, 0);
    gl.vertexAttribPointer(normal, 3, gl.FLOAT, false, 24, 12);
    gl.uniform3f(uniforms.position, pose[0] + offset[0], pose[1] + offset[1], pose[2] + offset[2]);
    gl.uniform4fv(uniforms.rotation, pose.slice(3));
    gl.uniform3fv(uniforms.scale, scale);
    gl.uniform3fv(uniforms.color, color);
    gl.drawArrays(mode, 0, item.count);
  }

  const tints = [[0.55, 0.92, 0.86], [1, 0.76, 0.52]];
  return function paint(frame, view, width, height, dpr, size, stale) {
    const pixelWidth = Math.round(width * dpr), pixelHeight = Math.round(height * dpr);
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    gl.viewport(0, 0, pixelWidth, pixelHeight);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.uniform2f(uniforms.viewport, 2 * size / width, 2 * size / height);
    gl.uniform4f(uniforms.camera, Math.cos(view.yaw), Math.sin(view.yaw),
      Math.cos(view.pitch), Math.sin(view.pitch));
    gl.uniform1f(uniforms.feedback, 1);
    draw(grid, [0, 0, 0, 0, 0, 0, 1], [0, 0, 0], [1, 1, 1], [0.14, 0.20, 0.28], gl.LINES);
    gl.uniform1f(uniforms.feedback, stale ? 0.57 : 1);
    for (const arm of frame?.arms ?? []) {
      arm.mesh_pq_m.forEach((pose, i) => {
        const item = models[i];
        const color = item.color.map((value, channel) => value * 0.68 + tints[arm.id][channel] * 0.32);
        draw(item, pose, arm.offset_m, item.scale, color, gl.TRIANGLES);
      });
    }
  };
}
