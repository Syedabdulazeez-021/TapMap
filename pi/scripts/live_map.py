#!/usr/bin/env python3
"""live_map.py - serve a LIVE defect heat map over HTTP.

Run on the Pi (after sourcing the workspace):
    cd ~/tapmap_ws && source install/setup.bash
    python3 ~/live_map.py
Then on the laptop browser open:  http://<PI_IP>:8080
The map updates every second as the scan runs.
"""
import io, threading
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tapmap_interfaces.msg import TapResult
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

state = {'grid': None, 'taps': [], 'lock': threading.Lock()}


class MapNode(Node):
    def __init__(self):
        super().__init__('live_map')
        self.create_subscription(OccupancyGrid, '/defect_grid', self.grid_cb, 1)
        self.create_subscription(TapResult, '/tap/classification', self.tap_cb, 20)
        self.get_logger().info('live_map up: open http://<pi-ip>:8080 on the laptop')

    def grid_cb(self, msg):
        with state['lock']:
            state['grid'] = msg

    def tap_cb(self, msg):
        with state['lock']:
            state['taps'].append((msg.x, msg.y, msg.label))


def render_png():
    with state['lock']:
        grid = state['grid']
        taps = list(state['taps'])
    fig, ax = plt.subplots(figsize=(7, 7))
    if grid is not None:
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        g = np.array(grid.data, dtype=float).reshape(h, w)
        g[g < 0] = np.nan
        ax.imshow(g, origin='lower', extent=[0, w * res, 0, h * res],
                  cmap='RdYlGn_r', vmin=0, vmax=100)
    for x, y, label in taps:
        ax.plot(x, y, 'o', ms=4, color='#C0392B' if label == 'hollow' else '#2E7D5B')
    n_h = sum(1 for t in taps if t[2] == 'hollow')
    ax.set_title(f'TapMap live    taps: {len(taps)}    hollow: {n_h}')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_aspect('equal')
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=90, bbox_inches='tight')
    plt.close(fig)
    return buf.getvalue()


PAGE = b"""<!doctype html><html><head><title>TapMap live</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;margin:0;padding:12px}
img{max-width:95vw;height:auto;border:1px solid #333;border-radius:8px;background:#fff}</style></head>
<body><h2>TapMap &mdash; live defect map</h2>
<img id="m" src="/map.png">
<script>setInterval(function(){document.getElementById('m').src='/map.png?t='+Date.now();},1000);</script>
</body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        try:
            if self.path.startswith('/map.png'):
                png = render_png()
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Length', str(len(png)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(png)
            else:
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
        except (BrokenPipeError, ConnectionResetError):
            pass


def serve():
    srv = ThreadingHTTPServer(('0.0.0.0', 8080), H)
    print('live_map web server listening on 0.0.0.0:8080', flush=True)
    srv.serve_forever()


def main():
    rclpy.init()
    node = MapNode()
    threading.Thread(target=serve, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
