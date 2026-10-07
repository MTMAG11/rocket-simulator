"""Byte transports between the simulator and a flight computer.

A transport is anything with ``send(bytes)`` and ``recv() -> bytes`` carrying ONE protocol line per call (lock-step: the
simulation waits for the reply). Implemented here:

LoopbackTransport   in-process: ``recv`` runs a handler (a Python flight computer) on the last sent line
PipeTransport       a child process speaking the protocol on stdin/stdout (a firmware host build, a C++ binary,
                    ``python -m rocket_sim.hil.flight_computer``); a reply timeout kills it: the same bytes a serial port would carry
RecordingTransport  wraps another transport and records every request/response pair (JSONL log)
ReplayTransport     answers from a recorded log, verifying that the simulator sends exactly the recorded requests

NOT implemented (yet): serial port, UDP, real-time pacing, timeouts/retries. A serial or UDP transport only has to provide
send/recv for one line; the lock-step schedule stays the same.
"""

from __future__ import annotations

import json
import queue
import subprocess
import threading
from collections.abc import Callable
from pathlib import Path
from typing import Protocol

from ..errors import SimulationError


class Transport(Protocol):
    def send(self, data: bytes) -> None: ...

    def recv(self) -> bytes: ...


class LoopbackTransport:
    def __init__(self, handler: Callable[[bytes], bytes]) -> None:
        self.handler = handler
        self._reply = b""

    def send(self, data: bytes) -> None:
        self._reply = self.handler(data)

    def recv(self) -> bytes:
        return self._reply


class PipeTransport:
    """Child-process flight computer: one JSON line in, one JSON line out."""

    def __init__(self, command: list[str], timeout_s: float = 60.0) -> None:
        self.timeout_s = timeout_s
        self.proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self._lines: queue.Queue[bytes] = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self) -> None:
        assert self.proc.stdout is not None
        for line in iter(self.proc.stdout.readline, b""):
            self._lines.put(line)
        self._lines.put(b"")  # EOF marker

    def send(self, data: bytes) -> None:
        assert self.proc.stdin is not None
        try:
            self.proc.stdin.write(data)
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise SimulationError("flight-computer process closed its input") from exc

    def recv(self) -> bytes:
        try:
            line = self._lines.get(timeout=self.timeout_s)
        except queue.Empty:
            self.proc.kill()
            raise SimulationError(
                f"flight-computer process did not reply within {self.timeout_s:g} s (killed)"
            ) from None
        if not line:
            raise SimulationError(f"flight-computer process exited (return code {self.proc.poll()})")
        return line

    def close(self) -> None:
        if self.proc.stdin:
            self.proc.stdin.close()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()


class RecordingTransport:
    def __init__(self, inner: Transport) -> None:
        self.inner = inner
        self.entries: list[dict[str, str]] = []
        self._pending = ""

    def send(self, data: bytes) -> None:
        self._pending = data.decode("utf-8")
        self.inner.send(data)

    def recv(self) -> bytes:
        reply = self.inner.recv()
        self.entries.append({"request": self._pending, "response": reply.decode("utf-8")})
        return reply

    def write_jsonl(self, path: str | Path) -> None:
        Path(path).write_text("\n".join(json.dumps(e) for e in self.entries) + "\n", encoding="utf-8")


class ReplayTransport:
    """Plays a recorded session back WITHOUT a flight computer; any divergence of the simulator's requests is an error."""

    def __init__(self, entries: list[dict[str, str]]) -> None:
        self.entries = entries
        self.i = 0
        self._current: dict[str, str] | None = None

    @classmethod
    def from_jsonl(cls, path: str | Path) -> ReplayTransport:
        return cls([json.loads(x) for x in Path(path).read_text(encoding="utf-8").splitlines() if x.strip()])

    def finish(self) -> None:
        """Raise unless every recorded message was consumed (a shorter replay would otherwise pass silently)."""
        if self.i != len(self.entries):
            raise SimulationError(
                f"replay incomplete: consumed {self.i} of {len(self.entries)} recorded messages"
            )

    def send(self, data: bytes) -> None:
        if self.i >= len(self.entries):
            raise SimulationError(f"replay exhausted after {len(self.entries)} messages")
        e = self.entries[self.i]
        if data.decode("utf-8") != e["request"]:
            raise SimulationError(
                f"replay diverged at message {self.i}: the simulator sent a different request"
            )
        self._current = e
        self.i += 1

    def recv(self) -> bytes:
        assert self._current is not None
        return self._current["response"].encode("utf-8")
