#!/usr/bin/env python3
"""
Joint-name parity test (URDF <-> controllers <-> SRDF <-> arm teleop)
=====================================================================
Verifies Requirement 4.1 and 9.1: the joint names referenced by the
controller YAML, the SRDF planning groups, and the arm teleop constants
exactly match the joint names declared in the combined URDF's
``ros2_control`` blocks.

Sources compared
----------------
1. Expanded combined URDF ``ros2_control`` blocks
     - ``ArmSystem``    -> the 5 arm joints
     - ``GripperSystem`` -> the 2 gripper finger joints
2. ``config/arm_controllers.yaml``
     - ``arm_controller`` / ``gripper_controller`` ``joints`` lists
3. ``moveit_config/srdf/mobile_manipulator.srdf``
     - ``arm`` group (a kinematic chain, resolved against the URDF tree)
     - ``gripper`` group (an explicit joint collection)
4. ``scripts/arm_teleop.py`` (OPTIONAL — created in a later task)
     - ``ARM_JOINTS`` / ``GRIPPER_JOINTS`` constants
     - The check is skipped gracefully while the file does not yet exist,
       and tightens automatically once it lands.

The arm set and the gripper set must each match exactly across every
source that is present.
"""

import ast
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

# ---------------------------------------------------------------------------
# Paths (resolved relative to this test file => the package root)
# ---------------------------------------------------------------------------
PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_XACRO = os.path.join(PKG_ROOT, "urdf", "mobile_manipulator.urdf.xacro")
ARM_CONTROLLERS_YAML = os.path.join(PKG_ROOT, "config", "arm_controllers.yaml")
SRDF_PATH = os.path.join(PKG_ROOT, "moveit_config", "srdf", "mobile_manipulator.srdf")
ARM_TELEOP_PY = os.path.join(PKG_ROOT, "scripts", "arm_teleop.py")

MOVABLE_JOINT_TYPES = {"revolute", "prismatic", "continuous", "planar", "floating"}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _expand_urdf() -> str:
    """Expand the combined Xacro into a flat URDF string using ``xacro``.

    Requires the workspace to be sourced so that ``xacro`` is on PATH and
    ``$(find ...)`` substitutions resolve. A clear failure is raised when
    that environment is missing so parity regressions are never hidden.
    """
    if shutil.which("xacro") is None:
        pytest.fail(
            "`xacro` was not found on PATH. Source the ROS 2 environment "
            "(e.g. `source /opt/ros/jazzy/setup.bash` and the workspace "
            "`install/setup.bash`) before running this test."
        )

    proc = subprocess.run(
        ["xacro", URDF_XACRO],
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        pytest.fail(
            "xacro failed to expand the combined URDF "
            f"({URDF_XACRO}).\nstderr:\n{proc.stderr}"
        )
    return proc.stdout


def _ros2_control_blocks(urdf_root: ET.Element) -> dict:
    """Return ``{block_name: set(joint_names)}`` for every ros2_control block."""
    blocks = {}
    for rc in urdf_root.findall("ros2_control"):
        name = rc.get("name")
        joints = {j.get("name") for j in rc.findall("joint")}
        blocks[name] = joints
    return blocks


def _joint_by_child(urdf_root: ET.Element) -> dict:
    """Map ``child_link -> (joint_name, joint_type, parent_link)``."""
    by_child = {}
    for j in urdf_root.findall("joint"):
        parent = j.find("parent")
        child = j.find("child")
        if parent is None or child is None:
            continue
        by_child[child.get("link")] = (
            j.get("name"),
            j.get("type"),
            parent.get("link"),
        )
    return by_child


def _resolve_chain_joints(by_child: dict, base_link: str, tip_link: str) -> set:
    """Walk the URDF tree from ``tip_link`` up to ``base_link``.

    Returns the set of *movable* joint names spanning the chain — this is the
    set of joints MoveIt actuates for a chain-defined planning group.
    """
    joints = set()
    link = tip_link
    visited = set()
    while link != base_link:
        if link in visited:
            raise AssertionError(f"Cycle detected while walking chain at '{link}'")
        visited.add(link)
        if link not in by_child:
            raise AssertionError(
                f"Reached the kinematic root before '{base_link}' "
                f"(stuck at '{link}'); SRDF chain does not exist in the URDF."
            )
        name, jtype, parent = by_child[link]
        if jtype in MOVABLE_JOINT_TYPES:
            joints.add(name)
        link = parent
    return joints


def _parse_srdf_groups(srdf_path: str) -> dict:
    """Return ``{group_name: {'chain': (base, tip) | None, 'joints': set()}}``."""
    root = ET.parse(srdf_path).getroot()
    groups = {}
    for g in root.findall("group"):
        chain_el = g.find("chain")
        chain = None
        if chain_el is not None:
            chain = (chain_el.get("base_link"), chain_el.get("tip_link"))
        joints = {j.get("name") for j in g.findall("joint")}
        groups[g.get("name")] = {"chain": chain, "joints": joints}
    return groups


def _controller_joints(yaml_path: str, controller: str) -> set:
    with open(yaml_path) as fh:
        data = yaml.safe_load(fh)
    return set(data[controller]["ros__parameters"]["joints"])


def _teleop_list_constant(py_path: str, const_name: str):
    """Extract a top-level list-of-strings constant from a Python source file.

    Uses AST parsing (no import) so the constant can be read without the ROS
    runtime. Handles two cases:

    1. A direct literal assignment ``NAME = [...]`` in ``py_path``.
    2. An ``from <module> import NAME`` statement — the constant is resolved
       by parsing ``<module>.py`` from the same directory as ``py_path``.
       This is how ``arm_teleop.py`` sources ``ARM_JOINTS`` / ``GRIPPER_JOINTS``
       from the shared, rclpy-free ``arm_joints`` module (task 5.4), so the
       parity intent (Req 4.1 / 9.1) stays enforced after deduplication.

    Returns ``None`` if the constant cannot be resolved.
    """
    with open(py_path) as fh:
        tree = ast.parse(fh.read())

    # Case 1: direct literal assignment in this file.
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == const_name:
                    try:
                        return set(ast.literal_eval(node.value))
                    except (ValueError, SyntaxError):
                        return None

    # Case 2: imported from a sibling module — follow the import.
    src_dir = os.path.dirname(os.path.abspath(py_path))
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            imported = {
                (alias.asname or alias.name): alias.name
                for alias in node.names
            }
            if const_name in imported:
                module_path = os.path.join(
                    src_dir, node.module.replace(".", os.sep) + ".py"
                )
                if os.path.isfile(module_path):
                    # Resolve the original (pre-alias) name in the source module.
                    return _teleop_list_constant(module_path, imported[const_name])
    return None


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def urdf_root() -> ET.Element:
    return ET.fromstring(_expand_urdf())


@pytest.fixture(scope="module")
def urdf_blocks(urdf_root):
    return _ros2_control_blocks(urdf_root)


@pytest.fixture(scope="module")
def srdf_groups():
    return _parse_srdf_groups(SRDF_PATH)


# ---------------------------------------------------------------------------
# Source-presence sanity checks
# ---------------------------------------------------------------------------
def test_source_files_exist():
    assert os.path.isfile(URDF_XACRO), f"missing {URDF_XACRO}"
    assert os.path.isfile(ARM_CONTROLLERS_YAML), f"missing {ARM_CONTROLLERS_YAML}"
    assert os.path.isfile(SRDF_PATH), f"missing {SRDF_PATH}"


def test_expected_ros2_control_blocks_present(urdf_blocks):
    """The combined URDF must expose the AGV, arm, and gripper systems."""
    for block in ("GazeboSystem", "ArmSystem", "GripperSystem"):
        assert block in urdf_blocks, (
            f"ros2_control block '{block}' missing from expanded URDF; "
            f"found: {sorted(urdf_blocks)}"
        )


# ---------------------------------------------------------------------------
# Arm joint parity: URDF <-> controllers <-> SRDF (Requirements 4.1, 9.1)
# ---------------------------------------------------------------------------
def test_arm_joint_parity(urdf_root, urdf_blocks, srdf_groups):
    urdf_arm = urdf_blocks["ArmSystem"]

    # SRDF arm group is a kinematic chain; resolve it against the URDF tree.
    arm_group = srdf_groups["arm"]
    assert arm_group["chain"] is not None, "SRDF 'arm' group must define a <chain>"
    base_link, tip_link = arm_group["chain"]
    srdf_arm = _resolve_chain_joints(_joint_by_child(urdf_root), base_link, tip_link)

    yaml_arm = _controller_joints(ARM_CONTROLLERS_YAML, "arm_controller")

    assert urdf_arm == yaml_arm, (
        "Arm joints differ between URDF ros2_control (ArmSystem) and "
        f"arm_controllers.yaml.\n  URDF: {sorted(urdf_arm)}\n  YAML: {sorted(yaml_arm)}"
    )
    assert urdf_arm == srdf_arm, (
        "Arm joints differ between URDF ros2_control (ArmSystem) and the "
        f"SRDF 'arm' chain.\n  URDF: {sorted(urdf_arm)}\n  SRDF: {sorted(srdf_arm)}"
    )


# ---------------------------------------------------------------------------
# Gripper joint parity: URDF <-> controllers <-> SRDF (Requirements 4.1, 9.1)
# ---------------------------------------------------------------------------
def test_gripper_joint_parity(urdf_blocks, srdf_groups):
    urdf_gripper = urdf_blocks["GripperSystem"]
    srdf_gripper = srdf_groups["gripper"]["joints"]
    yaml_gripper = _controller_joints(ARM_CONTROLLERS_YAML, "gripper_controller")

    assert urdf_gripper == yaml_gripper, (
        "Gripper joints differ between URDF ros2_control (GripperSystem) and "
        f"arm_controllers.yaml.\n  URDF: {sorted(urdf_gripper)}\n"
        f"  YAML: {sorted(yaml_gripper)}"
    )
    assert urdf_gripper == srdf_gripper, (
        "Gripper joints differ between URDF ros2_control (GripperSystem) and "
        f"the SRDF 'gripper' group.\n  URDF: {sorted(urdf_gripper)}\n"
        f"  SRDF: {sorted(srdf_gripper)}"
    )


# ---------------------------------------------------------------------------
# Arm-teleop joint parity (Requirements 4.1, 9.1)
# ---------------------------------------------------------------------------
# arm_teleop.py is created in a later task. While it is absent this test skips
# gracefully so the suite passes today; it tightens automatically once the
# file (with its ARM_JOINTS / GRIPPER_JOINTS constants) lands.
# ---------------------------------------------------------------------------
def test_arm_teleop_joint_parity(urdf_blocks):
    if not os.path.isfile(ARM_TELEOP_PY):
        pytest.skip(
            "scripts/arm_teleop.py not present yet (created in a later task); "
            "URDF<->YAML<->SRDF parity is still enforced by the other tests."
        )

    urdf_arm = urdf_blocks["ArmSystem"]
    urdf_gripper = urdf_blocks["GripperSystem"]

    teleop_arm = _teleop_list_constant(ARM_TELEOP_PY, "ARM_JOINTS")
    teleop_gripper = _teleop_list_constant(ARM_TELEOP_PY, "GRIPPER_JOINTS")

    assert teleop_arm is not None, (
        f"{ARM_TELEOP_PY} exists but ARM_JOINTS constant could not be parsed"
    )
    assert teleop_gripper is not None, (
        f"{ARM_TELEOP_PY} exists but GRIPPER_JOINTS constant could not be parsed"
    )
    assert urdf_arm == teleop_arm, (
        "Arm joints differ between URDF ros2_control (ArmSystem) and "
        f"arm_teleop.ARM_JOINTS.\n  URDF: {sorted(urdf_arm)}\n"
        f"  TELEOP: {sorted(teleop_arm)}"
    )
    assert urdf_gripper == teleop_gripper, (
        "Gripper joints differ between URDF ros2_control (GripperSystem) and "
        f"arm_teleop.GRIPPER_JOINTS.\n  URDF: {sorted(urdf_gripper)}\n"
        f"  TELEOP: {sorted(teleop_gripper)}"
    )


# ---------------------------------------------------------------------------
# Disjointness / cardinality sanity (guards against accidental overlap)
# ---------------------------------------------------------------------------
def test_arm_and_gripper_sets_are_disjoint(urdf_blocks):
    arm = urdf_blocks["ArmSystem"]
    gripper = urdf_blocks["GripperSystem"]
    assert arm.isdisjoint(gripper), (
        f"Arm and gripper joint sets overlap: {sorted(arm & gripper)}"
    )
    assert len(arm) == 5, f"expected 5 arm joints, got {sorted(arm)}"
    assert len(gripper) == 2, f"expected 2 gripper joints, got {sorted(gripper)}"
