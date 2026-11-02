"""
diag_log.py — Registro de diagnóstico que no bloquea el bucle de control.

El bucle llama a log(dict) (put_nowait en una cola acotada; si se llena se
descartan muestras y se cuentan). Un hilo de fondo escribe CSV en logs/ y
muestra una línea resumen en consola a LOG_CONSOLE_HZ. Los eventos (cambios de
estado, motivos de búsqueda/parada) se imprimen siempre.
"""

import csv
import os
import queue
import threading
import time

import config as _config

FIELDS = [
    "t", "state", "coverage", "raw", "surface",
    "target_rpm_a", "target_rpm_b", "rpm_a", "rpm_b", "err_a", "err_b", "duty_a", "duty_b",
    "omega_ref", "omega_meas", "yaw_corr_rpm", "yaw_deg", "v_rpm", "line_err", "distance_v", "note",
]


class DiagLogger:
    def __init__(self, name, cfg=None, console=True, to_file=True):
        cfg = cfg or _config
        self.q = queue.Queue(maxsize=cfg.LOG_QUEUE_SIZE)
        self.dropped = 0
        self.console = console
        self.console_period = 1.0 / cfg.LOG_CONSOLE_HZ if cfg.LOG_CONSOLE_HZ > 0 else None
        self.path = None
        self._fh = None
        if to_file:
            os.makedirs(cfg.LOG_DIR, exist_ok=True)
            self.path = os.path.join(cfg.LOG_DIR, f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            self._fh = open(self.path, "w", newline="")
            self._w = csv.DictWriter(self._fh, fieldnames=FIELDS, extrasaction="ignore")
            self._w.writeheader()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, name="diag_log", daemon=True)
        self._th.start()

    def log(self, row):
        try:
            self.q.put_nowait(("row", row))
        except queue.Full:
            self.dropped += 1

    def event(self, text):
        try:
            self.q.put_nowait(("event", text))
        except queue.Full:
            self.dropped += 1

    def _run(self):
        last_print = 0.0
        while not self._stop.is_set() or not self.q.empty():
            try:
                kind, item = self.q.get(timeout=0.1)
            except queue.Empty:
                continue
            if kind == "event":
                if self.console:
                    print(f"[evento] {item}", flush=True)
                if self._fh:
                    self._w.writerow({"t": time.monotonic(), "note": item})
                continue
            if self._fh:
                self._w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in item.items()})
            now = time.monotonic()
            if self.console and self.console_period and now - last_print >= self.console_period:
                last_print = now
                print(format_row(item), flush=True)

    def close(self):
        self._stop.set()
        self._th.join(timeout=2.0)
        if self._fh:
            self._fh.close()
        if self.dropped and self.console:
            print(f"[diag] muestras de log descartadas: {self.dropped}")


def format_row(r):
    g = lambda k, f="{:6.1f}": f.format(r[k]) if isinstance(r.get(k), (int, float)) else "  -  "  # noqa: E731
    return (f"{r.get('state', ''):<16} cov={g('coverage', '{:4.2f}')} "
            f"tgt A/B={g('target_rpm_a')}/{g('target_rpm_b')} "
            f"rpm A/B={g('rpm_a')}/{g('rpm_b')} "
            f"pwm A/B={g('duty_a', '{:5.2f}')}/{g('duty_b', '{:5.2f}')} "
            f"w ref/med={g('omega_ref')}/{g('omega_meas')} yaw={g('yaw_deg')} dist={g('distance_v', '{:4.2f}')}V")
