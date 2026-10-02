"""Live, read-only verification of the HSL26 ROS graph contract.

R1 intentionally validates graph metadata only.  It neither creates command
publishers nor arms a robot; the only command authority it accepts is the mux
already present in the graph.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


@dataclass(frozen=True)
class Endpoint:
    """ROS endpoint metadata reduced to stable, serializable fields."""

    node: str
    topic_type: str
    reliability: str
    durability: str


@dataclass(frozen=True, order=True)
class TransformEdge:
    """A parent-to-child TF edge observed on the static TF channel."""

    parent: str
    child: str


def normalized_node_name(namespace: str, name: str) -> str:
    """Return an absolute ROS node name without introducing a double slash."""

    namespace = namespace.rstrip("/") or "/"
    return f"{namespace}/{name}" if namespace != "/" else f"/{name}"


def normalize_policy(value: Any) -> str:
    """Normalize rclpy QoS enums and test doubles to policy names."""

    text = str(getattr(value, "name", value)).lower()
    if "best_effort" in text or text.endswith(".2"):
        return "best_effort"
    if "reliable" in text or text.endswith(".1"):
        return "reliable"
    if "transient_local" in text or text.endswith(".1"):
        return "transient_local"
    if "volatile" in text or text.endswith(".2"):
        return "volatile"
    return text


def endpoint_from_ros(info: Any) -> Endpoint:
    """Translate ``TopicEndpointInfo`` without retaining rclpy objects."""

    qos = info.qos_profile
    return Endpoint(
        node=normalized_node_name(info.node_namespace, info.node_name),
        topic_type=info.topic_type,
        reliability=normalize_policy(qos.reliability),
        durability=normalize_policy(qos.durability),
    )


def validate_physical_authority(
    publishers: Sequence[Endpoint], *, topic: str, sole_publisher: str
) -> list[str]:
    """Require exactly the configured mux publisher on the physical topic."""

    errors: list[str] = []
    if len(publishers) != 1:
        errors.append(
            f"{topic}: expected exactly one publisher ({sole_publisher}), found "
            f"{len(publishers)}: {[publisher.node for publisher in publishers]}"
        )
    elif publishers[0].node != sole_publisher:
        errors.append(
            f"{topic}: sole publisher must be {sole_publisher}, found "
            f"{publishers[0].node}"
        )
    return errors


def qos_compatible(offer: Endpoint, request: Endpoint) -> bool:
    """Apply the reliability and durability compatibility rules used by R1.

    A reliable request cannot connect to a best-effort offer, and a
    transient-local request cannot connect to a volatile offer.  Unknown
    policies are rejected rather than treated as compatible.
    """

    known_reliability = {"reliable", "best_effort"}
    known_durability = {"transient_local", "volatile"}
    if offer.reliability not in known_reliability or request.reliability not in known_reliability:
        return False
    if offer.durability not in known_durability or request.durability not in known_durability:
        return False
    if request.reliability == "reliable" and offer.reliability != "reliable":
        return False
    return not (request.durability == "transient_local" and offer.durability != "transient_local")


def validate_topic_qos(
    topic: str, publishers: Iterable[Endpoint], subscribers: Iterable[Endpoint]
) -> list[str]:
    """Report every incompatible offered/requested endpoint pair for a topic."""

    errors: list[str] = []
    for publisher in publishers:
        for subscriber in subscribers:
            if not qos_compatible(publisher, subscriber):
                errors.append(
                    f"{topic}: incompatible QoS offer {publisher.node} "
                    f"({publisher.reliability}/{publisher.durability}) -> "
                    f"request {subscriber.node} "
                    f"({subscriber.reliability}/{subscriber.durability})"
                )
    return errors


def validate_static_tf_edges(
    observed: Iterable[TransformEdge], expected: Iterable[TransformEdge]
) -> list[str]:
    """Reject duplicate/missing static edges and children with two parents.

    TF messages do not contain a portable ROS node identity, so this check
    proves structural single ownership of static edges.  The configured
    semantic owner remains in the signed graph contract and is reviewed with
    the broadcaster launch configuration.
    """

    observed = tuple(observed)
    errors: list[str] = []
    for edge in sorted(set(observed)):
        if observed.count(edge) != 1:
            errors.append(f"/tf_static: duplicate edge {edge.parent}->{edge.child}")
    parents_by_child: dict[str, set[str]] = {}
    for edge in observed:
        parents_by_child.setdefault(edge.child, set()).add(edge.parent)
    for child, parents in sorted(parents_by_child.items()):
        if len(parents) > 1:
            errors.append(f"/tf_static: child {child} has multiple parents: {sorted(parents)}")
    observed_set = set(observed)
    for edge in sorted(set(expected) - observed_set):
        errors.append(f"/tf_static: required edge missing: {edge.parent}->{edge.child}")
    return errors


def expected_static_tf_edges(contract: Mapping[str, Any]) -> tuple[TransformEdge, ...]:
    """Return the calibration-owned edges expected on ``/tf_static``."""

    return tuple(
        TransformEdge(entry["parent"], entry["child"])
        for entry in contract.get("tf", [])
        if entry.get("owner") == "calibration_static_tf"
    )


def load_contract(path: Path) -> Mapping[str, Any]:
    """Load and minimally validate the versioned JSON graph inventory."""

    contract = json.loads(path.read_text(encoding="utf-8"))
    if contract.get("schema_version") != 1:
        raise ValueError("unsupported graph-contract schema_version")
    physical = contract.get("physical_command", {})
    if not physical.get("topic") or not physical.get("sole_publisher"):
        raise ValueError("graph contract has no complete physical_command declaration")
    return contract


def inspect_live_graph(
    node: Any, contract: Mapping[str, Any], additional_topics: Iterable[str] = ()
) -> dict[str, Any]:
    """Capture endpoints and validate authority/QoS without mutating the graph."""

    topics = {entry["name"] for entry in contract.get("topics", [])}
    physical = contract["physical_command"]
    topics.add(physical["topic"])
    topics.update(additional_topics)
    report: dict[str, Any] = {"schema_version": 1, "topics": {}, "errors": []}
    for topic in sorted(topics):
        publishers = [endpoint_from_ros(info) for info in node.get_publishers_info_by_topic(topic)]
        subscribers = [endpoint_from_ros(info) for info in node.get_subscriptions_info_by_topic(topic)]
        report["topics"][topic] = {
            "publishers": [asdict(endpoint) for endpoint in publishers],
            "subscribers": [asdict(endpoint) for endpoint in subscribers],
        }
        report["errors"].extend(validate_topic_qos(topic, publishers, subscribers))
        if topic == physical["topic"]:
            report["errors"].extend(
                validate_physical_authority(
                    publishers,
                    topic=topic,
                    sole_publisher=physical["sole_publisher"],
                )
            )
    return report


def default_contract_path() -> Path:
    """Locate the installed contract, with a source-tree fallback for tests."""

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return Path(__file__).resolve().parents[1] / "config" / "ros_graph_contract.json"
    return Path(get_package_share_directory("hsl_bringup")) / "config" / "ros_graph_contract.json"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=default_contract_path())
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument(
        "--additional-topic",
        action="append",
        default=[],
        help="additional topic to inspect (repeatable; useful for negative fixtures)",
    )
    parser.add_argument(
        "--check-tf-static",
        action="store_true",
        help="observe /tf_static and require the calibration-owned edges in the contract",
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.settle_seconds < 0:
        raise ValueError("--settle-seconds must be non-negative")
    try:
        import rclpy
    except ImportError as exc:  # pragma: no cover - exercised only without ROS
        raise RuntimeError("rclpy is required to inspect a live ROS graph") from exc

    contract = load_contract(args.contract)
    rclpy.init(args=None)
    node = rclpy.create_node("hsl_graph_verifier")
    observed_static_tf: list[TransformEdge] = []
    try:
        if args.check_tf_static:
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from tf2_msgs.msg import TFMessage

            def record_static_tf(message: Any) -> None:
                observed_static_tf.extend(
                    TransformEdge(transform.header.frame_id, transform.child_frame_id)
                    for transform in message.transforms
                )

            node.create_subscription(
                TFMessage,
                "/tf_static",
                record_static_tf,
                QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
        deadline = time.monotonic() + args.settle_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
        report = inspect_live_graph(node, contract, args.additional_topic)
        if args.check_tf_static:
            report["tf_static"] = [asdict(edge) for edge in sorted(observed_static_tf)]
            report["errors"].extend(
                validate_static_tf_edges(
                    observed_static_tf, expected_static_tf_edges(contract)
                )
            )
        report["contract"] = str(args.contract)
        report["result"] = "PASS" if not report["errors"] else "FAIL"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if report["errors"]:
            for error in report["errors"]:
                print(error, file=sys.stderr)
            return 1
        print(f"live graph PASS; wrote {args.output}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
