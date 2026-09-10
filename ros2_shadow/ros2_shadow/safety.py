"""A check on where the candidate publishes.

Putting the candidate in a namespace handles this for nodes that use relative
topic names, which is most of them; Nav2 relies on exactly that for its
multi-robot configurations. The gap is a node that hardcodes a leading slash or
builds a topic name at runtime, since a namespace does not apply to either.

This scan catches that case, but only once the publisher shows up in the graph,
so a message may already have gone out. For a guarantee rather than a warning,
run the candidate in its own ROS_DOMAIN_ID and bridge what it needs with
ros2/domain_bridge.
"""

from __future__ import annotations

from fnmatch import fnmatchcase


def matches_forbidden(topic: str, patterns: list[str]) -> bool:
    return any(topic == p or fnmatchcase(topic, p) for p in patterns)


def scan(node, patterns: list[str], candidate_namespace: str) -> list[tuple[str, str]]:
    """Return (topic, node name) for candidate publishers on forbidden topics."""
    offences = []
    namespace = candidate_namespace.rstrip("/")

    for topic, _types in node.get_topic_names_and_types():
        if not matches_forbidden(topic, patterns):
            continue
        for info in node.get_publishers_info_by_topic(topic):
            full = f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            if namespace and (full.startswith(namespace + "/") or info.node_namespace.rstrip("/") == namespace):
                offences.append((topic, full))
    return offences
