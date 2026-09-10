import pytest

from ros2_shadow.config import ConfigError, ShadowConfig
from ros2_shadow.safety import matches_forbidden

BASE = {
    "production": {"topic": "/planner/cmd_vel"},
    "shadow": {"topic": "/shadow/planner/cmd_vel"},
    "comparison": {"type": "geometry_msgs/msg/Twist"},
}


def merged(**overrides):
    import copy

    config = copy.deepcopy(BASE)
    for key, value in overrides.items():
        config.setdefault(key, {})
        if isinstance(value, dict):
            config[key].update(value)
        else:
            config[key] = value
    return config


def test_minimal_config_loads():
    config = ShadowConfig.from_dict(BASE)
    assert config.production_topic == "/planner/cmd_vel"
    assert config.tolerance_ms == 20.0


@pytest.mark.parametrize("missing", ["production", "shadow", "comparison"])
def test_required_sections_are_enforced(missing):
    config = dict(BASE)
    config.pop(missing)
    with pytest.raises(ConfigError):
        ShadowConfig.from_dict(config)


def test_comparing_a_topic_against_itself_is_rejected():
    """Silently comparing production to production would report a flawless
    candidate that was never running."""
    with pytest.raises(ConfigError, match="same"):
        ShadowConfig.from_dict(merged(shadow={"topic": "/planner/cmd_vel"}))


def test_metrics_accept_bare_names_and_thresholds():
    config = ShadowConfig.from_dict(
        merged(comparison={"metrics": ["linear_error", {"name": "angular_error", "critical": 0.3}]})
    )
    assert config.metrics[0].name == "linear_error"
    assert config.metrics[0].critical is None
    assert config.metrics[1].critical == 0.3


def test_metric_entry_without_a_name_is_rejected():
    with pytest.raises(ConfigError, match="name"):
        ShadowConfig.from_dict(merged(comparison={"metrics": [{"critical": 0.3}]}))


# -- isolation -----------------------------------------------------------

def test_isolation_parses():
    config = ShadowConfig.from_dict(merged(isolation={
        "production_domain": 1,
        "candidate_domain": 2,
        "outputs": [{"topic": "/planner/cmd_vel", "type": "geometry_msgs/msg/Twist",
                     "republish_as": "/shadow/planner/cmd_vel"}],
    }))
    assert config.isolation.candidate_domain == 2
    assert config.isolation.outputs[0].republish_as == "/shadow/planner/cmd_vel"


def test_identical_domains_are_rejected():
    """Same domain on both sides is not isolation, it just looks like it."""
    with pytest.raises(ConfigError, match="no isolation"):
        ShadowConfig.from_dict(merged(isolation={
            "production_domain": 5, "candidate_domain": 5,
        }))


def test_bridging_back_onto_a_forbidden_topic_is_rejected():
    """The whole point of the bridge is that the candidate cannot reach
    hardware. A return path onto /cmd_vel would hand it a route back."""
    with pytest.raises(ConfigError, match="forbidden"):
        ShadowConfig.from_dict(merged(isolation={
            "production_domain": 1,
            "candidate_domain": 2,
            "outputs": [{"topic": "/planner/cmd_vel",
                         "type": "geometry_msgs/msg/Twist",
                         "republish_as": "/cmd_vel"}],
        }))


def test_bridged_entry_needs_a_type():
    with pytest.raises(ConfigError, match="type"):
        ShadowConfig.from_dict(merged(isolation={
            "production_domain": 1, "candidate_domain": 2,
            "inputs": [{"topic": "/scan"}],
        }))


# -- forbidden patterns --------------------------------------------------

def test_forbidden_matching_handles_globs_and_exact_names():
    patterns = ["/cmd_vel", "/hardware/*"]
    assert matches_forbidden("/cmd_vel", patterns)
    assert matches_forbidden("/hardware/motors", patterns)
    assert not matches_forbidden("/shadow/cmd_vel", patterns)
    assert not matches_forbidden("/planner/cmd_vel", patterns)
