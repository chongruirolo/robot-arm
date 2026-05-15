"""
Test logger for robot arm test scripts.

Usage
-----
    from robot_logger import RobotLogger

    log = RobotLogger("test_joints")

    with log.test("CAN connect"):
        arm = C_PiperInterface(...)
        arm.ConnectPort()
        log.info("port opened")

    with log.test("Read joints"):
        joints = arm.get_joints_deg()
        log.check("returns 6 values", len(joints) == 6, expected=6, actual=len(joints))
        log.check("J1 in range",       -360 < joints[0] < 360,
                  expected="(-360, 360)", actual=f"{joints[0]:.2f}°")

    log.summary()

- Writes to logs/<date>_<time>_<suite>.log
- Mirrors every line to stdout
- Context manager catches exceptions and marks the test FAIL automatically
"""

import os
import traceback
from contextlib import contextmanager
from datetime import datetime


_COL_LABEL  = 36
_COL_STATUS =  6
_DIVIDER    = "─" * 56


class RobotLogger:
    def __init__(self, suite: str = "test"):
        self._suite   = suite
        self._results: list[tuple[str, bool, str]] = []  # (name, passed, note)
        self._current: str | None = None
        self._checks_pass = 0
        self._checks_fail = 0

        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self._path = os.path.join("logs", f"{ts}_{suite}.log")
        self._fh   = open(self._path, "w", encoding="utf-8")

        self._write(f"{'=' * 56}")
        self._write(f"Robot Arm Test Log — {suite}")
        self._write(f"Run: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        self._write(f"{'=' * 56}\n")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @contextmanager
    def test(self, name: str):
        """Context manager for one test block. Catches exceptions → FAIL."""
        self._current      = name
        self._checks_pass  = 0
        self._checks_fail  = 0
        self._write(f"\n{_DIVIDER}")
        self._write(f"[{self._ts()}]  TEST: {name}")
        passed     = False
        error_note = ""
        try:
            yield
            passed = self._checks_fail == 0
            total  = self._checks_pass + self._checks_fail
            note   = f"{self._checks_pass}/{total} checks" if total else "no checks"
            self._write(f"[{self._ts()}]  {'PASS' if passed else 'FAIL'}  ({note})")
        except BaseException as e:
            error_note = f"{type(e).__name__}: {e}"
            self._write(f"[{self._ts()}]  ERROR  {error_note}")
            self._write(traceback.format_exc().strip())
            raise
        finally:
            self._results.append((name, passed, error_note))
            self._current = None

    def check(self, label: str, condition: bool,
              expected=None, actual=None):
        """Assert a condition and log the result."""
        status = "PASS" if condition else "FAIL"
        parts  = [f"[{self._ts()}]  CHECK  {label:<{_COL_LABEL}}  {status:<{_COL_STATUS}}"]
        if expected is not None:
            parts.append(f"expected={expected}")
        if actual is not None:
            parts.append(f"actual={actual}")
        self._write("  ".join(parts))
        if condition:
            self._checks_pass += 1
        else:
            self._checks_fail += 1

    def info(self, msg: str):
        """Log a free-form info line inside a test block."""
        self._write(f"[{self._ts()}]  INFO   {msg}")

    def summary(self):
        """Print and log the final pass/fail table, then close the file."""
        passed = sum(1 for _, ok, _ in self._results if ok)
        total  = len(self._results)

        self._write(f"\n{'=' * 56}")
        self._write(f"SUMMARY — {self._suite}")
        self._write(f"{'=' * 56}")
        for name, ok, note in self._results:
            tag  = "PASS" if ok else "FAIL"
            line = f"  [{tag}]  {name}"
            if note:
                line += f"\n         {note}"
            self._write(line)
        self._write(f"{'=' * 56}")
        self._write(f"Result: {passed}/{total} passed")
        self._write(f"Log:    {os.path.abspath(self._path)}")
        self._write(f"{'=' * 56}\n")
        self._fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        if not self._fh.closed:
            self.summary()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _ts(self) -> str:
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _write(self, line: str):
        print(line)
        self._fh.write(line + "\n")
        self._fh.flush()
