# tools/check_ros_graph_authority.py
"""Check that exactly one configured publisher owns the physical command topic."""

import argparse
import json
from typing import Iterable, Mapping, Tuple


def unauthorized_publishers(
    publishers: Iterable[Mapping[str, str]],
    *,
    physical_topic: str = "/commands/velocity",
    allowed_node: str = "/cmd_vel_mux",
) -> Tuple[Mapping[str, str], ...]:
    """Return publishers that violate the physical command authority contract."""

    if not physical_topic or not allowed_node:
        raise ValueError("physical_topic and allowed_node must be non-empty")
    return tuple(
        publisher
        for publisher in publishers
        if publisher.get("topic") == physical_topic
        and publisher.get("node") != allowed_node
    )


def validate_graph(
    publishers: Iterable[Mapping[str, str]],
    *,
    physical_topic: str = "/commands/velocity",
    allowed_node: str = "/cmd_vel_mux",
) -> None:
    """Reject missing mux authority or any additional physical publisher."""

    matching = tuple(
        publisher for publisher in publishers
        if publisher.get("topic") == physical_topic
    )
    if len(matching) != 1 or matching[0].get("node") != allowed_node:
        raise ValueError(
            "physical command authority must have exactly one mux publisher"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("graph_json", help="JSON file containing a publishers list")
    args = parser.parse_args()
    with open(args.graph_json, encoding="utf-8") as stream:
        graph = json.load(stream)
    validate_graph(graph["publishers"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
