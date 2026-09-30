"""Small deterministic process-pool helper for independent P6.3 fixture jobs."""

from concurrent.futures import ProcessPoolExecutor
from typing import Callable, Iterable, TypeVar


Job = TypeVar("Job")
Result = TypeVar("Result")


def validate_worker_count(workers: int) -> None:
    if (
        not isinstance(workers, int)
        or isinstance(workers, bool)
        or workers < 1
    ):
        raise ValueError("workers must be a positive integer")


def ordered_process_map(
    function: Callable[[Job], Result],
    jobs: Iterable[Job],
    *,
    workers: int,
) -> tuple[Result, ...]:
    """Run independent picklable jobs and preserve submission order."""
    validate_worker_count(workers)
    materialized = tuple(jobs)
    if workers == 1:
        return tuple(function(job) for job in materialized)
    with ProcessPoolExecutor(max_workers=workers) as executor:
        return tuple(executor.map(function, materialized))
