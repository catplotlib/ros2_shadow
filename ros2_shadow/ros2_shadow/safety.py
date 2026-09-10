"""Backstop against a candidate reaching hardware.

Namespace remapping is cooperative. A node can build a topic name at runtime,
or use a service or action instead of a topic, and remapping will not stop it.
This module watches the graph and shouts, which is strictly weaker than the
isolation it backs up: by the time a publisher is discovered, a message may
already have gone out.

Run the candidate in its own ROS_DOMAIN_ID if the guarantee matters. Then the
hardware topics do not exist in its world at all and no policy is needed.
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
