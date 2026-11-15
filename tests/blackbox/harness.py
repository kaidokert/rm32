"""
Python driver for the AM32 black-box test harness.

Usage:
    from harness import AM32Harness

    with AM32Harness() as h:
        h.config(armed=1, inputSet=1)
        state = h.tick(throttle=500)
        assert state['armed'] == 1
"""

import queue
import re
import subprocess
import threading
from fractions import Fraction

REPLY_TIMEOUT_S = 5
import os
from pathlib import Path


class AM32Harness:
    """Drives the am32_harness executable via stdin/stdout."""

    def __init__(self, exe_path=None):
        if exe_path is None:
            # Default: look for Rust harness relative to this file
            repo = Path(__file__).resolve().parent.parent.parent
            exe_path = repo / "target" / "release" / "rm32_harness"
            if not exe_path.exists():
                # Windows build artifact
                exe_path = exe_path.with_suffix(".exe")
            if not exe_path.exists():
                # Fallback: C harness
                exe_path = repo / "build" / "am32_harness"
        self.exe_path = str(exe_path)
        # The AM32 C harness has AM32's behaviour only; the Rust harness
        # takes a `tuning` command (rm32::tuning).
        self.is_c = Path(self.exe_path).stem == "am32_harness"
        # Port the Rust harness runs the core on: `stm32` (the AM32-parity
        # model) or `sim` (rm32-sim). RM32_PORT selects it.
        self.port = os.environ.get("RM32_PORT", "stm32")
        if self.is_c and self.port != "stm32":
            raise RuntimeError(f"RM32_PORT={self.port}: the C harness is the STM32 model only")
        self.proc = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()

    def start(self):
        self.proc = subprocess.Popen(
            [self.exe_path],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,  # line buffered
        )
        # Reader thread: every reply goes through a queue so a harness that
        # never answers (unknown command -> stderr only) fails the vector
        # after REPLY_TIMEOUT_S instead of blocking the suite forever.
        self._q = queue.Queue()
        threading.Thread(target=self._pump, args=(self.proc.stdout, self._q),
                         daemon=True).start()
        # Wait for ready marker
        line = self._recv()
        if line != "ready":
            raise RuntimeError(f"Expected 'ready', got '{line}'")

    def stop(self):
        if self.proc:
            try:
                self._send("quit")
                self.proc.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                self.proc.kill()
            self.proc = None

    def _send(self, cmd):
        self.proc.stdin.write(cmd + "\n")
        self.proc.stdin.flush()

    @staticmethod
    def _pump(stream, q):
        for line in stream:
            q.put(line)
        q.put(None)

    def _recv(self):
        try:
            line = self._q.get(timeout=REPLY_TIMEOUT_S)
        except queue.Empty:
            raise TimeoutError(
                f"harness {self.exe_path} sent no reply in {REPLY_TIMEOUT_S} s"
            ) from None
        if line is None:
            raise RuntimeError(f"harness {self.exe_path} exited")
        return line.strip()

    def _parse_state(self, line):
        """Parse 'key=value key=value ...' into a dict of ints."""
        state = {}
        for token in line.split():
            if "=" in token:
                k, v = token.split("=", 1)
                try:
                    state[k] = int(v)
                except ValueError:
                    state[k] = v
        return state

    def _kvargs(self, **kwargs):
        return " ".join(f"{k}={v}" for k, v in kwargs.items())

    # The C harness has no `rates` command: AM32's LOOP_FREQUENCY_HZ
    # default (Inc/targets.h) and the 2 MHz interval timer its vectors'
    # counts were written in.
    C_RATES = {"tick_hz": 20000, "timer_hz": 2_000_000}

    def rates(self):
        """{'tick_hz': control-tick Hz, 'timer_hz': interval-timer Hz}."""
        if self.is_c:
            return dict(self.C_RATES)
        self._send("rates")
        line = self._recv()
        fields = dict(kv.split("=", 1) for kv in line.split())
        if "tick_hz" not in fields or "timer_hz" not in fields:
            raise RuntimeError(f"Expected 'tick_hz=... timer_hz=...', got '{line}'")
        return {"tick_hz": int(fields["tick_hz"]), "timer_hz": int(fields["timer_hz"])}

    def tick_hz(self):
        """Control-tick rate the harness runs at (Hz)."""
        return self.rates()["tick_hz"]

    def tuning(self, name):
        """Select the port (RM32_PORT) and tuning (`rig` | `am32`); starts
        a fresh harness."""
        for cmd in (f"port {self.port}", f"tuning {name}"):
            self._send(cmd)
            line = self._recv()
            if line != "ok":
                raise RuntimeError(f"{cmd}: expected 'ok', got '{line}'")

    def reset(self):
        """Reset all firmware state to init values."""
        self._send("reset")
        line = self._recv()
        if line != "reset":
            raise RuntimeError(f"Expected 'reset', got '{line}'")

    def config(self, **kwargs):
        """Set config values (eeprom fields or state overrides)."""
        self._send(f"config {self._kvargs(**kwargs)}")
        line = self._recv()
        if line != "ok":
            raise RuntimeError(f"Expected 'ok', got '{line}'")

    def load_eeprom(self):
        """Call loadEEpromSettings() to apply eeprom config."""
        self._send("load_eeprom")
        line = self._recv()
        if line != "ok":
            raise RuntimeError(f"Expected 'ok', got '{line}'")

    def state(self):
        """Query current state without ticking."""
        self._send("state")
        return self._parse_state(self._recv())

    def crsf(self, data):
        """Feed raw serial bytes without advancing the control loop."""
        if not data:
            return self.state()
        self._send("crsf " + " ".join(f"{byte:02x}" for byte in data))
        return self._parse_state(self._recv())

    def tick(self, **kwargs):
        """Advance one tick with optional overrides. Returns state dict."""
        if kwargs:
            self._send(f"tick {self._kvargs(**kwargs)}")
        else:
            self._send("tick")
        return self._parse_state(self._recv())

    def ticks(self, n, **kwargs):
        """Advance N ticks (bulk). Returns final state dict."""
        if kwargs:
            self._send(f"ticks {n} {self._kvargs(**kwargs)}")
        else:
            self._send(f"ticks {n}")
        return self._parse_state(self._recv())

    def gcr_encode(self, com_time, **kwargs):
        """Encode com_time via make_dshot_package and return gcr buffer + metadata.
        Returns dict with 'gcr' (comma-separated), 'shift', 'dshot_full', 'padding'."""
        args = f"{com_time}"
        for k, v in kwargs.items():
            args += f" {k}={v}"
        self._send(f"gcr_encode {args}")
        return self._parse_state(self._recv())

    def edt_next(self, **kwargs):
        """Advance the EDT scheduler and return the selected frame."""
        self._send(f"edt_next {self._kvargs(**kwargs)}")
        return self._parse_state(self._recv())

    def arm(self, input_type="dshot"):
        """Convenience: run the arming sequence.
        Sets inputSet=1, zero throttle, waits for armed=1."""
        self.config(inputSet=1, dshot=1 if input_type == "dshot" else 0,
                    zero_input_count=31)
        state = self.ticks(20001, throttle=0)
        if state["armed"] != 1:
            raise RuntimeError(f"Failed to arm after 20001 ticks: armed={state['armed']}")
        return state


TUNINGS = ("rig", "am32")

_DURATION = re.compile(r"^(\d+(?:\.\d+)?)(us|ms|s)?$")
_UNIT_PER_S = {"us": 1_000_000, "ms": 1_000, "s": 1}


def ticks_for(count, tick_hz):
    """`ticks` operand -> control ticks. A bare integer is a tick count
    (event steps: frames, pulses, one state-machine step); `<n>us|ms|s` is
    a duration, converted at the harness's tick rate. A duration that does
    not land on a whole tick, or rounds to zero ticks, is refused."""
    m = _DURATION.match(count)
    if not m:
        raise ValueError(f"Bad ticks operand: {count!r}")
    value, unit = m.group(1), m.group(2)
    if unit is None:
        if "." in value:
            raise ValueError(f"Tick count must be an integer: {count!r}")
        return int(value)
    n = Fraction(value) * tick_hz / _UNIT_PER_S[unit]
    if n.denominator != 1 or n <= 0:
        raise ValueError(f"{count} is {float(n)} ticks at {tick_hz} Hz, not a whole tick count")
    return int(n)


# Keys whose value is an interval-timer count. Only these may be written as
# a duration (`5ms`, `48us`), converted at the harness's timer rate.
TIMER_KEYS = {
    "commutation_interval",
    "average_interval",
    "last_average_interval",
    "interval_timer_count",
}


def timer_value(key, value, timer_hz):
    """A config/assertion value: a duration on a timer key -> timer counts
    (exact, or refused); anything else is returned unchanged."""
    m = _DURATION.match(value)
    if not m or m.group(2) is None:
        return value
    if key not in TIMER_KEYS:
        raise ValueError(f"{key}={value}: only {sorted(TIMER_KEYS)} take a duration")
    n = Fraction(m.group(1)) * timer_hz / _UNIT_PER_S[m.group(2)]
    if n.denominator != 1:
        raise ValueError(f"{key}={value} is {float(n)} counts at {timer_hz} Hz")
    return str(int(n))


def read_tuning(vectors_file):
    """The vector's required `@tuning rig|am32` line."""
    found = None
    with open(vectors_file, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line.startswith("@tuning"):
                if found is not None:
                    raise ValueError(f"{vectors_file}: more than one @tuning")
                found = line.split(None, 1)[1].strip() if " " in line else ""
    if found not in TUNINGS:
        raise ValueError(
            f"{vectors_file}: needs exactly one `@tuning rig|am32` line, got {found!r}"
        )
    return found


def run_test_vectors(harness, vectors_file, tuning=None):
    """
    Run test vectors from a file.

    Format:
        # Comments start with #
        # Blank lines are ignored

        @tuning am32        # or rig (rm32::tuning); required

        @config
        key=value
        key=value

        @sequence
        # tick_spec | inputs | assertions
        tick 1      | throttle=0          |
        ticks 20000 | throttle=0          | armed=1
        ticks 1s    | throttle=0          |       # duration: us|ms|s,
                                                  # converted at tick_hz
        config commutation_interval=5ms           # timer keys (TIMER_KEYS)
                                                  # take durations too,
                                                  # converted at timer_hz
        tick 1      | throttle=500        | running=1 duty_cycle>0
    """

    section = None
    config_lines = []
    sequence_lines = []

    with open(vectors_file, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("@tuning"):
                continue
            if line == "@config":
                section = "config"
                continue
            if line == "@sequence":
                section = "sequence"
                continue
            if section == "config":
                config_lines.append(line)
            elif section == "sequence":
                sequence_lines.append(line)

    # Apply config
    declared = read_tuning(vectors_file)
    tuning = tuning or declared
    if not harness.is_c:
        harness.tuning(tuning)
    elif tuning != "am32":
        raise RuntimeError(f"{vectors_file}: `{tuning}` vector sent to the C harness")
    harness.reset()
    rates = harness.rates()
    tick_hz, timer_hz = rates["tick_hz"], rates["timer_hz"]
    for cl in config_lines:
        if "=" not in cl:
            continue
        k, v = cl.split("=", 1)
        k = k.strip()
        harness.config(**{k: timer_value(k, v.strip(), timer_hz)})

    # Operator dispatch table
    ops = {
        "=": lambda a, b: a == b,
        ">": lambda a, b: a > b,
        "<": lambda a, b: a < b,
        ">=": lambda a, b: a >= b,
        "<=": lambda a, b: a <= b,
    }

    # Run sequence
    results = []
    for seq_line in sequence_lines:
        # Inline config commands
        if seq_line.startswith("config "):
            kvs = seq_line[7:]
            for token in kvs.split():
                if "=" in token:
                    k, v = token.split("=", 1)
                    harness.config(**{k: timer_value(k, v, timer_hz)})
            continue

        # Inline commands
        if seq_line == "load_eeprom":
            harness.load_eeprom()
            continue

        parts = [p.strip() for p in seq_line.split("|")]
        cmd_part = parts[0]
        input_part = parts[1] if len(parts) > 1 else ""
        assert_part = parts[2] if len(parts) > 2 else ""

        # Parse inputs
        inputs = {}
        for token in input_part.split():
            if "=" in token:
                k, v = token.split("=", 1)
                inputs[k] = int(v)

        # Execute
        tokens = cmd_part.split()
        cmd = tokens[0]
        if cmd == "tick":
            state = harness.tick(**inputs)
        elif cmd == "ticks":
            n = ticks_for(tokens[1], tick_hz)
            state = harness.ticks(n, **inputs)
        elif cmd == "gcr_encode":
            com_time = int(tokens[1])
            state = harness.gcr_encode(com_time, **inputs)
        elif cmd == "edt_next":
            state = harness.edt_next(**inputs)
        else:
            raise ValueError(f"Unknown command: {cmd}")

        # Check assertions
        if assert_part:
            for assertion in assert_part.split():
                # Support: key=value, key>value, key<value, key>=value, key<=value
                # Value can be numeric or string (for gcr= comma-separated buffers)
                m = re.match(r"(\w+)(>=|<=|>|<|=)([\w,.-]+)", assertion)
                if not m:
                    raise ValueError(f"Bad assertion: {assertion}")
                key, op, expected_str = m.group(1), m.group(2), m.group(3)
                expected_str = timer_value(key, expected_str, timer_hz)
                actual = state.get(key)
                if actual is None:
                    raise KeyError(f"Key '{key}' not in state")
                # Try numeric comparison; fall back to string
                try:
                    expected = int(expected_str)
                except ValueError:
                    expected = expected_str

                if not ops[op](actual, expected):
                    raise AssertionError(
                        f"tick={state.get('tick')}: {key}={actual}, expected {key}{op}{expected}")

        results.append(state)

    return results
