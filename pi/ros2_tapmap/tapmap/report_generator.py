#!/usr/bin/env python3
"""report_generator: render the inspection report on demand.

SUBSCRIBES: /defect_grid (nav_msgs/OccupancyGrid)
            /tap/classification (tapmap_interfaces/TapResult)
SERVES    : /generate_report (std_srvs/Trigger) -> writes a PDF

The PDF has the defect heat map, a tap-by-tap table and summary counts.
Uses only matplotlib (no reportlab dependency).
"""
import os
import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from tapmap_interfaces.msg import TapResult

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


class ReportGenerator(Node):
    def __init__(self):
        super().__init__('report_generator')
        self.declare_parameter('out_path', os.path.expanduser('~/tapmap_report.pdf'))
        self.grid = None
        self.taps = []
        self.create_subscription(OccupancyGrid, 'defect_grid', self.grid_cb, 1)
        self.create_subscription(TapResult, 'tap/classification', self.tap_cb, 20)
        self.create_service(Trigger, 'generate_report', self.gen_cb)
        self.get_logger().info('report_generator up: call /generate_report when a scan finishes')

    def grid_cb(self, msg):
        self.grid = msg

    def tap_cb(self, msg):
        self.taps.append((msg.x, msg.y, msg.label, msg.confidence))

    def gen_cb(self, req, res):
        path = self.get_parameter('out_path').value
        try:
            self._render(path)
            res.success = True
            res.message = f'report written to {path}'
            self.get_logger().info(res.message)
        except Exception as e:                       # noqa: BLE001
            res.success = False
            res.message = f'report failed: {e}'
            self.get_logger().error(res.message)
        return res

    def _render(self, path):
        hollow = [t for t in self.taps if t[2] == 'hollow']
        with PdfPages(path) as pdf:
            # page 1 — heat map
            fig, ax = plt.subplots(figsize=(8.3, 8.3))
            if self.grid is not None:
                w, h = self.grid.info.width, self.grid.info.height
                res = self.grid.info.resolution
                g = np.array(self.grid.data, dtype=float).reshape(h, w)
                g[g < 0] = np.nan
                ax.imshow(g, origin='lower', extent=[0, w * res, 0, h * res],
                          cmap='RdYlGn_r', vmin=0, vmax=100)
            for x, y, label, _ in self.taps:
                ax.plot(x, y, 'o', ms=4,
                        color='#C0392B' if label == 'hollow' else '#2E7D5B')
            ax.set_title('TapMap — defect heat map')
            ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_aspect('equal')
            pdf.savefig(fig); plt.close(fig)

            # page 2 — summary + tap table
            fig, ax = plt.subplots(figsize=(8.3, 11.7)); ax.axis('off')
            lines = [
                'TapMap inspection report',
                datetime.datetime.now().strftime('%Y-%m-%d %H:%M'),
                '',
                f'Total taps      : {len(self.taps)}',
                f'Debonded (hollow): {len(hollow)}',
                f'Bonded (solid)  : {len(self.taps) - len(hollow)}',
                '',
                'x(m)   y(m)   label    conf',
                '-' * 34,
            ]
            for x, y, label, c in self.taps[:60]:
                lines.append(f'{x:5.2f}  {y:5.2f}  {label:7s}  {c:.2f}')
            ax.text(0.05, 0.98, '\n'.join(lines), va='top', family='monospace', fontsize=9)
            pdf.savefig(fig); plt.close(fig)


def main():
    rclpy.init()
    rclpy.spin(ReportGenerator())


if __name__ == '__main__':
    main()
