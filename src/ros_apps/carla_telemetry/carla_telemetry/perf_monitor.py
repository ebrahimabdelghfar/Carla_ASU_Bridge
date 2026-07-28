"""
| File: perf_monitor.py
| Description: Lightweight pipeline performance instrumentation.
|
|   Enable:   CARLA_PERF=1  (environment variable)
|   Disable:  unset or CARLA_PERF=0  (all calls become no-ops)
|
|   Usage:
|       from carla_telemetry.perf_monitor import perf
|       t0 = perf.tick()
|       ...
|       perf.record("camera.parse", t0)
|
|   Every ``log_interval`` seconds the monitor dumps p50/p95/max
|   for each metric to the ``carla_telemetry.perf`` logger.
"""
__all__ = ["perf", "PerfMonitor"]

import collections
import logging
import os
import threading
import time

logger = logging.getLogger("carla_telemetry.perf")

_ENABLED = os.environ.get("CARLA_PERF", "0").strip() not in ("0", "", "false")


class PerfMonitor:
    """Per-metric ring-buffer timing collector."""

    def __init__(self, *, enabled: bool = True, log_interval: float = 10.0,
                 buffer_size: int = 2000):
        self._enabled = enabled
        self._log_interval = log_interval
        self._buffer_size = buffer_size
        self._lock = threading.Lock()
        self._metrics: dict[str, collections.deque] = {}
        self._last_log = time.monotonic()

    # ── Public API ────────────────────────────────────────────────────

    @staticmethod
    def tick() -> float:
        """Capture a start timestamp (always cheap — single syscall)."""
        return time.perf_counter()

    def record(self, name: str, start: float) -> None:
        """Record elapsed time since ``start`` under metric ``name``."""
        if not self._enabled:
            return
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        with self._lock:
            buf = self._metrics.get(name)
            if buf is None:
                buf = collections.deque(maxlen=self._buffer_size)
                self._metrics[name] = buf
            buf.append(elapsed_ms)
        self._maybe_log()

    def record_value(self, name: str, value: float) -> None:
        """Record an arbitrary value (e.g. queue depth)."""
        if not self._enabled:
            return
        with self._lock:
            buf = self._metrics.get(name)
            if buf is None:
                buf = collections.deque(maxlen=self._buffer_size)
                self._metrics[name] = buf
            buf.append(value)

    # ── Logging ───────────────────────────────────────────────────────

    def _maybe_log(self) -> None:
        now = time.monotonic()
        if (now - self._last_log) < self._log_interval:
            return
        # Avoid lock contention from multiple threads triggering log
        with self._lock:
            if (now - self._last_log) < self._log_interval:
                return
            self._last_log = now
            snapshot = {k: list(v) for k, v in self._metrics.items()}

        if not snapshot:
            return

        lines = ["\n╔══ PERF REPORT ══════════════════════════════════════════╗"]
        for name in sorted(snapshot):
            vals = snapshot[name]
            if not vals:
                continue
            vals_sorted = sorted(vals)
            n = len(vals_sorted)
            p50 = vals_sorted[n // 2]
            p95 = vals_sorted[int(n * 0.95)]
            mx  = vals_sorted[-1]
            mn  = vals_sorted[0]
            avg = sum(vals_sorted) / n
            lines.append(
                f"║  {name:40s}  "
                f"n={n:5d}  "
                f"avg={avg:7.2f}  "
                f"p50={p50:7.2f}  "
                f"p95={p95:7.2f}  "
                f"max={mx:7.2f} ms"
            )
        lines.append("╚════════════════════════════════════════════════════════╝")
        logger.info("\n".join(lines))


# Module-level singleton — no-op if disabled
perf = PerfMonitor(enabled=_ENABLED)
