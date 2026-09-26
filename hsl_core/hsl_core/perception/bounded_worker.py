# hsl_core/hsl_core/perception/bounded_worker.py
"""Single-flight bounded perception worker with deadline/version rejection."""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from threading import Lock
import math
from typing import Callable, Generic, TypeVar


T = TypeVar("T")
R = TypeVar("R")


@dataclass(frozen=True)
class WorkerResult(Generic[R]):
    request_id: int
    status: str
    value: R | None
    reason: str
    started_at: float
    completed_at: float
    expected_versions: tuple[int, int, str]


@dataclass
class _WorkItem(Generic[T, R]):
    request_id: int
    future: Future[R]
    deadline: float
    started_at: float
    expected_versions: tuple[int, int, str]
    generation: int


class WorkerBusyError(RuntimeError):
    """Raised when the only worker still owns an unfinished task."""


class BoundedWorker(Generic[T, R]):
    """One worker thread; caller-side polling never blocks on model execution.

    A timed-out Python thread cannot be forcibly terminated. Therefore the
    implementation never spawns replacement workers while a timed-out task is
    still running: resource use stays bounded to one task, and its eventual
    result is discarded.
    """

    def __init__(self, *, timeout_s: float, clock: Callable[[], float]) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive and finite")
        self._timeout_s = timeout_s
        self._clock = clock
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hsl-p4")
        self._lock = Lock()
        self._item: _WorkItem[T, R] | None = None
        self._next_id = 1
        self._generation = 0
        self._closed = False

    def submit(
        self,
        function: Callable[[T], R],
        payload: T,
        *,
        map_version: int,
        topology_version: int,
        localization_epoch: str,
    ) -> int:
        if map_version < 0 or topology_version < 0 or not localization_epoch:
            raise ValueError("worker version context is invalid")
        with self._lock:
            if self._closed:
                raise RuntimeError("worker is closed")
            now = self._clock()
            if not math.isfinite(now):
                raise ValueError("worker clock must return a finite value")
            if self._item is not None:
                if not self._item.future.done():
                    raise WorkerBusyError("single worker is still processing a task")
                self._item = None
            request_id = self._next_id
            self._next_id += 1
            future = self._executor.submit(function, payload)
            self._item = _WorkItem(
                request_id,
                future,
                now + self._timeout_s,
                now,
                (map_version, topology_version, localization_epoch),
                self._generation,
            )
            return request_id

    def invalidate(self) -> None:
        """Invalidate all work submitted before a stage/configuration change."""
        with self._lock:
            self._generation += 1

    def poll(
        self,
        *,
        map_version: int,
        topology_version: int,
        localization_epoch: str,
    ) -> WorkerResult[R] | None:
        if map_version < 0 or topology_version < 0 or not localization_epoch:
            raise ValueError("current version context is invalid")
        now = self._clock()
        if not math.isfinite(now):
            raise ValueError("worker clock must return a finite value")
        with self._lock:
            item = self._item
            if item is None:
                return None
            versions = (map_version, topology_version, localization_epoch)
            if versions != item.expected_versions or item.generation != self._generation:
                if item.future.done():
                    self._item = None
                return WorkerResult(
                    item.request_id, "DISCARDED", None,
                    "VERSION_OR_GENERATION_MISMATCH",
                    item.started_at, now, item.expected_versions,
                )
            if now >= item.deadline:
                if item.future.done():
                    self._item = None
                return WorkerResult(
                    item.request_id, "TIMEOUT", None, "DEADLINE_EXCEEDED",
                    item.started_at, now, item.expected_versions,
                )
            if not item.future.done():
                return None
            self._item = None
            try:
                value = item.future.result()
            except Exception as exc:
                return WorkerResult(
                    item.request_id, "FAILED", None,
                    f"{type(exc).__name__}: {exc}",
                    item.started_at, now, item.expected_versions,
                )
            return WorkerResult(
                item.request_id, "COMPLETED", value, "OK",
                item.started_at, now, item.expected_versions,
            )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            if self._item is not None and not self._item.future.running():
                self._item.future.cancel()
        self._executor.shutdown(wait=False, cancel_futures=True)
