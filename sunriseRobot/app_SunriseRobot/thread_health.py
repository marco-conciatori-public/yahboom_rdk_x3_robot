"""
Thread health registry for the main app.

Each long-lived thread reports here: it calls beat() once per loop iteration and report_exception() where it
catches an exception. snapshot() turns that into one row per thread with a state the touch panel can colour:

    ok       green   alive, beating within its warn threshold, no recent exception
    warning  yellow  alive, but silent for longer than warn_after, or caught an exception recently
    error    red     silent for longer than error_after, or the thread is dead without having said it was done
    stopped  grey    the thread ended on purpose (it called mark_stopped() first, e.g. no OLED screen fitted)

Thresholds are per thread, in configs/thread_health.yaml: a controller loop beats many times a second, while a
button listener blocks waiting for a press, so silence only means something for some threads (null = never).

Every function takes the thread name from threading.current_thread() by default, so a thread only has to call
thread_health.beat() / thread_health.report_exception(e, 'message'). A thread is registered the first time it
calls any of them; call register() at the top of the thread to make it visible before its first beat.

Beats are lock-free (one float assignment), so they are cheap enough for loops running at 100 Hz.
"""

import time
import threading

import args
import global_constants as gc
# straight from robot_link (utils re-exports it), so importing this module stays light: no cv2 / ultralytics
from robot_link.common import print_exception

STATE_OK = 'ok'
STATE_WARNING = 'warning'
STATE_ERROR = 'error'
STATE_STOPPED = 'stopped'

# used when configs/thread_health.yaml cannot be read (e.g. running the code off-robot)
_FALLBACK_CONFIG = {
    'verbose': 1,
    'default_warn_after': None,
    'default_error_after': None,
    'recent_error_window': 60,
    'log_transitions': True,
    'check_interval': 1.0,
    'threads': {},
}


class _ThreadEntry:
    def __init__(self, name: str, thread: threading.Thread, warn_after, error_after):
        self.name = name
        self.thread = thread
        self.warn_after = warn_after
        self.error_after = error_after
        now = time.monotonic()
        self.registered_at = now
        # silence is counted from registration until the first beat
        self.last_beat = now
        self.beat_count = 0
        self.last_error = None
        self.last_error_at = None  # monotonic, for ages
        self.last_error_wall = None  # time.time(), for display
        self.error_count = 0
        self.stopped_reason = None  # set by mark_stopped(): the thread ended on purpose


class ThreadHealthRegistry:
    def __init__(self, config: dict):
        self.default_warn_after = config.get('default_warn_after')
        self.default_error_after = config.get('default_error_after')
        self.recent_error_window = config.get('recent_error_window', 60)
        self.thresholds = config.get('threads') or {}
        self.log_transitions = config.get('log_transitions', True)
        self.check_interval = config.get('check_interval', 1.0)
        self.verbose = config.get('verbose', 1)
        self._entries = {}
        self._lock = threading.Lock()
        self._monitor_thread = None

    # ------------------------------------------------------------------ reporting (called by the threads)

    def register(self, name: str = None) -> _ThreadEntry:
        """Register the calling thread (or return its entry if it already is). Re-registering a name whose
        previous thread has died replaces the old entry, so a restarted thread starts clean."""
        current = threading.current_thread()
        name = name if name is not None else current.name
        entry = self._entries.get(name)
        if entry is not None and (entry.thread is current or entry.thread.is_alive()):
            return entry
        with self._lock:
            entry = self._entries.get(name)
            if entry is None or not (entry.thread is current or entry.thread.is_alive()):
                thresholds = self.thresholds.get(name) or {}
                entry = _ThreadEntry(
                    name=name,
                    thread=current,
                    warn_after=thresholds.get('warn_after', self.default_warn_after),
                    error_after=thresholds.get('error_after', self.default_error_after),
                )
                self._entries[name] = entry
        return entry

    def beat(self, name: str = None) -> None:
        """Record that the thread is making progress. Call once per loop iteration."""
        entry = self._entries.get(name if name is not None else threading.current_thread().name)
        if entry is None:
            entry = self.register(name)
        entry.last_beat = time.monotonic()
        entry.beat_count += 1

    def record_exception(self, exception: BaseException, message: str = None, name: str = None) -> None:
        entry = self.register(name)
        text = f'{type(exception).__name__}: {exception}'
        if message:
            text = f'{message}: {text}'
        entry.last_error = text
        entry.last_error_at = time.monotonic()
        entry.last_error_wall = time.time()
        entry.error_count += 1

    def report_exception(self, exception: BaseException, message: str = None, name: str = None) -> None:
        """Record the exception for the thread, then print it with its traceback."""
        self.record_exception(exception=exception, message=message, name=name)
        print_exception(exception=exception, message=message)

    def mark_stopped(self, reason: str = None, name: str = None) -> None:
        """Say the thread is about to end on purpose, so its exit shows as 'stopped' instead of 'error'."""
        entry = self.register(name)
        entry.stopped_reason = reason if reason else 'stopped'

    # ------------------------------------------------------------------ reading (panel, logs)

    def _evaluate(self, entry: _ThreadEntry, now: float) -> tuple:
        """Return (state, reason) for one entry."""
        if not entry.thread.is_alive():
            if entry.stopped_reason is not None:
                return STATE_STOPPED, entry.stopped_reason
            if entry.last_error is not None:
                return STATE_ERROR, 'thread died: ' + entry.last_error
            return STATE_ERROR, 'thread died'
        if entry.stopped_reason is not None:
            # marked stopped, still running its last lines
            return STATE_STOPPED, entry.stopped_reason

        silence = now - entry.last_beat
        if entry.error_after is not None and silence > entry.error_after:
            return STATE_ERROR, f'no activity for {silence:.1f}s'
        if entry.warn_after is not None and silence > entry.warn_after:
            return STATE_WARNING, f'no activity for {silence:.1f}s'
        if entry.last_error_at is not None and now - entry.last_error_at < self.recent_error_window:
            return STATE_WARNING, 'recent error: ' + entry.last_error
        return STATE_OK, ''

    def snapshot(self) -> list:
        """One dict per registered thread, sorted by name. Plain types only, so it can go straight into JSON."""
        now = time.monotonic()
        with self._lock:
            entries = list(self._entries.values())
        rows = []
        for entry in sorted(entries, key=lambda e: e.name):
            state, reason = self._evaluate(entry, now)
            rows.append({
                'name': entry.name,
                'state': state,
                'reason': reason,
                'alive': entry.thread.is_alive(),
                'seconds_since_beat': round(now - entry.last_beat, 1),
                'beat_count': entry.beat_count,
                'uptime': round(now - entry.registered_at, 1),
                'last_error': entry.last_error,
                'seconds_since_error': (
                    round(now - entry.last_error_at, 1) if entry.last_error_at is not None else None),
                'last_error_time': entry.last_error_wall,
                'error_count': entry.error_count,
            })
        return rows

    def summary(self) -> dict:
        """Count of threads per state, e.g. {'ok': 9, 'warning': 1, 'error': 0, 'stopped': 1}."""
        counts = {STATE_OK: 0, STATE_WARNING: 0, STATE_ERROR: 0, STATE_STOPPED: 0}
        for row in self.snapshot():
            counts[row['state']] += 1
        return counts

    # ------------------------------------------------------------------ transition log

    def start_monitor(self) -> None:
        """Start a daemon thread that prints every state change (ok -> warning, ...) to the journal.
        Useful on its own, before the touch panel exists. Does nothing if log_transitions is off."""
        if not self.log_transitions or self._monitor_thread is not None:
            return
        self._monitor_thread = threading.Thread(target=self._monitor_loop, name='thread_health_monitor',
                                                daemon=True)
        self._monitor_thread.start()

    def _monitor_loop(self) -> None:
        previous = {}
        while True:
            try:
                for row in self.snapshot():
                    old_state = previous.get(row['name'])
                    if row['state'] != old_state:
                        # a thread appearing healthy is not worth a line; anything else is
                        if old_state is not None or row['state'] != STATE_OK:
                            reason = f' ({row["reason"]})' if row['reason'] else ''
                            print(f'[thread health] {row["name"]}: {old_state or "new"} -> {row["state"]}{reason}')
                        previous[row['name']] = row['state']
            except Exception as e:
                print_exception(exception=e, message='thread health monitor error')
            time.sleep(self.check_interval)


def _load_config() -> dict:
    try:
        config = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'thread_health.yaml')
        merged = dict(_FALLBACK_CONFIG)
        merged.update(config or {})
        return merged
    except Exception as e:
        print_exception(exception=e, message='thread_health: could not read config, using defaults')
        return dict(_FALLBACK_CONFIG)


# one registry per process, shared by every thread
registry = ThreadHealthRegistry(config=_load_config())


def register(name: str = None) -> None:
    registry.register(name=name)


def beat(name: str = None) -> None:
    registry.beat(name=name)


def report_exception(exception: BaseException, message: str = None, name: str = None) -> None:
    registry.report_exception(exception=exception, message=message, name=name)


def record_exception(exception: BaseException, message: str = None, name: str = None) -> None:
    registry.record_exception(exception=exception, message=message, name=name)


def mark_stopped(reason: str = None, name: str = None) -> None:
    registry.mark_stopped(reason=reason, name=name)


def snapshot() -> list:
    return registry.snapshot()


def summary() -> dict:
    return registry.summary()


def start_monitor() -> None:
    registry.start_monitor()
