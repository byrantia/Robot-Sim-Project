// Floor plans from the nav graph file's "overlays": ROS-style occupancy images. resolution = metres per pixel,
// origin = world position of the bottom-left pixel. Pixel values: 255 free floor, 0 wall, anything else (128) unknown.
// Used to check that a detour round an obstacle stays on free floor.
const zlib = require('node:zlib');

// Decodes an 8-bit, non-interlaced PNG (grey, grey+alpha, RGB or RGBA) to one grey byte per pixel. null otherwise.
function decodePng(buf) {
  if (buf.length < 33 || buf.toString('latin1', 1, 4) !== 'PNG') return null;
  const w = buf.readUInt32BE(16), h = buf.readUInt32BE(20), depth = buf[24], type = buf[25], interlace = buf[28];
  const channels = { 0: 1, 2: 3, 4: 2, 6: 4 }[type];
  if (depth !== 8 || !channels || interlace) return null;
  const idat = [];
  for (let o = 8; o + 8 <= buf.length;) {
    const len = buf.readUInt32BE(o), kind = buf.toString('latin1', o + 4, o + 8);
    if (kind === 'IDAT') idat.push(buf.subarray(o + 8, o + 8 + len));
    o += 12 + len;
  }
  const raw = zlib.inflateSync(Buffer.concat(idat)), stride = w * channels;
  let prev = Buffer.alloc(stride), cur = Buffer.alloc(stride);
  const grey = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    const f = raw[y * (stride + 1)], src = y * (stride + 1) + 1;
    for (let x = 0; x < stride; x++) {
      const a = x >= channels ? cur[x - channels] : 0, b = prev[x], c = x >= channels ? prev[x - channels] : 0;
      let p = 0;
      if (f === 1) p = a; else if (f === 2) p = b; else if (f === 3) p = (a + b) >> 1;
      else if (f === 4) { const q = a + b - c, pa = Math.abs(q - a), pb = Math.abs(q - b), pc = Math.abs(q - c); p = pa <= pb && pa <= pc ? a : pb <= pc ? b : c; }
      cur[x] = (raw[src + x] + p) & 255;
    }
    for (let x = 0; x < w; x++) {
      const i = x * channels;
      grey[y * w + x] = channels >= 3 ? Math.round((cur[i] + cur[i + 1] + cur[i + 2]) / 3) : cur[i];
    }
    [prev, cur] = [cur, prev];
  }
  return { w, h, grey };
}

// overlay { meta: { resolution, origin: [x, y], imageWidth, imageHeight }, pngBase64 } -> occupancy map, or null
function occupancyOf(overlay) {
  const meta = overlay && overlay.meta;
  if (!meta || !(meta.resolution > 0) || !Array.isArray(meta.origin) || !overlay.pngBase64) return null;
  const img = decodePng(Buffer.from(String(overlay.pngBase64).split(',').pop(), 'base64'));
  if (!img) return null;
  const res = meta.resolution, ox = meta.origin[0], oy = meta.origin[1], { w, h, grey } = img;
  return {
    res, w, h,
    // is a circle of the given radius at (x, y) entirely on free floor? (off the image counts as not free)
    free(x, y, radius = 0) {
      const x0 = Math.floor((x - radius - ox) / res), x1 = Math.floor((x + radius - ox) / res);
      const y0 = Math.floor((y - radius - oy) / res), y1 = Math.floor((y + radius - oy) / res);
      if (x0 < 0 || y0 < 0 || x1 >= w || y1 >= h) return false;
      const r2 = (radius + res / 2) ** 2;
      for (let j = y0; j <= y1; j++) {
        const cy = oy + (j + 0.5) * res, row = (h - 1 - j) * w;
        for (let i = x0; i <= x1; i++) {
          const cx = ox + (i + 0.5) * res;
          if ((cx - x) ** 2 + (cy - y) ** 2 <= r2 && grey[row + i] < 250) return false;
        }
      }
      return true;
    },
  };
}
module.exports = { occupancyOf, decodePng };
