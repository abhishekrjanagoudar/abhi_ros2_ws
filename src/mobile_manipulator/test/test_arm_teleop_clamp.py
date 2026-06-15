#!/usr/bin/env python3
"""
Property-based tests for arm-teleop joint clamping (Requirement 8.3)
====================================================================
Requirement 8.3 states that every commanded arm/gripper joint position must be
clamped to the joint's URDF ``[lower, upper]`` limits before a trajectory goal
is sent. This module verifies that guarantee with Hypothesis property tests.

The pure clamp helper and the joint constants are imported directly from
``arm_teleop`` (which sources them from the rclpy-free ``arm_joints`` module),
so these tests run under pytest without a running ROS graph: ``arm_teleop``
defers all ``rclpy`` imports into ``main()`` (see its module docstring).

Properties encoded
------------------
1. ``test_clamp_within_limits`` — for any known joint and any finite float
   value (including values far outside the limits), ``clamp_to_limits`` returns
   a value inside ``[lower, upper]``.
2. ``test_clamp_idempotent`` — clamping an already-in-range value returns it
   unchanged, and clamping is idempotent (``clamp(clamp(x)) == clamp(x)``).
3. ``test_clamp_unknown_joint_raises`` — an unknown joint name raises KeyError.
4. ``test_jog_loop_stays_within_limits`` — simulating the teleop jog loop
   (per-joint target vector, ``target = clamp(joint, target + dir*step)``
   applied each step, faithful to ``ArmTeleop.jog``) keeps every joint's target
   inside ``[lower, upper]`` after EVERY step of a random jog sequence.

NaN / inf handling decision
---------------------------
``clamp_to_limits`` is ``max(lower, min(upper, value))``. For ``+inf`` this
yields ``upper`` and for ``-inf`` it yields ``lower`` (both in range), so
infinities are handled safely. ``NaN`` propagates through ``min``/``max`` and
would NOT be clamped — but the teleop node never produces NaN: jog steps are
finite constants accumulated onto a finite seed, and the gripper/open-close
setpoints are finite. We therefore exclude NaN from the generated value space
(``allow_nan=False``) to mirror the real input domain, and we additionally
assert that finite infinities are clamped into range. This is a documented,
deliberate scoping of the input space, not a gap in the clamp guarantee.

**Validates: Requirements 8.3**
"""

import math
import os
import sys

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

# ---------------------------------------------------------------------------
# Import the pure helpers/constants from arm_teleop without a ROS runtime.
# arm_teleop defers rclpy imports into main(), so importing the module only
# needs its scripts dir on sys.path (it also re-exports the arm_joints consts).
# ---------------------------------------------------------------------------
SCRIPTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"
)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

import arm_teleop  # noqa: E402
from arm_teleop import (  # noqa: E402
    ARM_JOINTS,
    DEFAULT_JOG_STEP,
    JOINT_LIMITS,
    clamp_to_limits,
)

# All known joints (arm + gripper) for the generic clamp properties.
ALL_JOINTS = sorted(JOINT_LIMITS.keys())

# A wide but finite value range that straddles every joint limit by a large
# margin so Hypothesis exercises values far outside [lower, upper] as well as
# values inside the band. No NaN (see module docstring); infinities are tested
# separately and explicitly below.
WIDE_VALUES = st.floats(
    min_value=-1000.0, max_value=1000.0, allow_nan=False, allow_infinity=False
)


# ---------------------------------------------------------------------------
# Property 1 — clamp always lands within [lower, upper]
# ---------------------------------------------------------------------------
@settings(max_examples=400)
@given(joint=st.sampled_from(ALL_JOINTS), value=WIDE_VALUES)
def test_clamp_within_limits(joint, value):
    """clamp_to_limits(joint, value) is always within the joint's limits.

    **Validates: Requirements 8.3**
    """
    lower, upper = JOINT_LIMITS[joint]
    result = clamp_to_limits(joint, value)
    assert lower <= result <= upper


def test_clamp_handles_infinities():
    """+inf clamps to the upper bound, -inf clamps to the lower bound.

    **Validates: Requirements 8.3**
    """
    for joint in ALL_JOINTS:
        lower, upper = JOINT_LIMITS[joint]
        assert clamp_to_limits(joint, math.inf) == upper
        assert clamp_to_limits(joint, -math.inf) == lower


# ---------------------------------------------------------------------------
# Property 3 — idempotence / in-range values pass through unchanged
# ---------------------------------------------------------------------------
@settings(max_examples=400)
@given(joint=st.sampled_from(ALL_JOINTS), value=WIDE_VALUES)
def test_clamp_idempotent(joint, value):
    """Clamping is idempotent and leaves already-in-range values unchanged.

    **Validates: Requirements 8.3**
    """
    lower, upper = JOINT_LIMITS[joint]
    once = clamp_to_limits(joint, value)
    twice = clamp_to_limits(joint, once)
    # Idempotent: clamping a clamped value changes nothing.
    assert once == twice
    # An already-in-range input is returned unchanged.
    if lower <= value <= upper:
        assert once == value


# ---------------------------------------------------------------------------
# Unknown joints must raise (documented KeyError contract).
# ---------------------------------------------------------------------------
@given(joint=st.text().filter(lambda s: s not in JOINT_LIMITS), value=WIDE_VALUES)
def test_clamp_unknown_joint_raises(joint, value):
    """An unknown joint name raises KeyError.

    **Validates: Requirements 8.3**
    """
    with pytest.raises(KeyError):
        clamp_to_limits(joint, value)


# ---------------------------------------------------------------------------
# Property 2 — faithful teleop jog-loop simulation stays within limits
# ---------------------------------------------------------------------------
def _seed_within_limits(seed_fracs):
    """Build a per-arm-joint target vector seeded inside each joint's limits.

    ``seed_fracs[i]`` in [0, 1] linearly interpolates joint i between its
    lower and upper bound, mirroring how ArmTeleop seeds ``self._targets``
    from /joint_states (always an in-range starting position).
    """
    targets = []
    for joint, frac in zip(ARM_JOINTS, seed_fracs):
        lower, upper = JOINT_LIMITS[joint]
        targets.append(lower + frac * (upper - lower))
    return targets


# Each jog step: (arm_joint_index, direction in {-1,+1}). This mirrors the
# ARM_JOG_BINDINGS value shape consumed by ArmTeleop.jog(index, direction).
_jog_step_strategy = st.tuples(
    st.integers(min_value=0, max_value=len(ARM_JOINTS) - 1),
    st.sampled_from([-1, 1]),
)


@settings(max_examples=300)
@given(
    seed_fracs=st.lists(
        st.floats(min_value=0.0, max_value=1.0, allow_nan=False,
                  allow_infinity=False),
        min_size=len(ARM_JOINTS),
        max_size=len(ARM_JOINTS),
    ),
    steps=st.lists(_jog_step_strategy, min_size=0, max_size=200),
    jog_step=st.floats(min_value=0.01, max_value=0.5, allow_nan=False,
                       allow_infinity=False),
)
def test_jog_loop_stays_within_limits(seed_fracs, steps, jog_step):
    """Simulate ArmTeleop's jog accumulation; every step stays within limits.

    Faithful to ``ArmTeleop.jog``:
        joint   = ARM_JOINTS[index]
        desired = targets[index] + direction * jog_step
        targets[index] = clamp_to_limits(joint, desired)

    The same per-joint target vector is mutated in place across the whole
    sequence (the node keeps ``self._targets`` between keypresses), and the
    invariant is asserted after EVERY step, not just at the end.

    **Validates: Requirements 8.3**
    """
    targets = _seed_within_limits(seed_fracs)

    # Seed must itself be within limits (sanity for the simulation setup).
    for joint, t in zip(ARM_JOINTS, targets):
        lower, upper = JOINT_LIMITS[joint]
        assert lower <= t <= upper

    for index, direction in steps:
        joint = ARM_JOINTS[index]
        lower, upper = JOINT_LIMITS[joint]
        desired = targets[index] + direction * jog_step
        targets[index] = clamp_to_limits(joint, desired)
        # Invariant after every accumulation step (Req 8.3).
        assert lower <= targets[index] <= upper, (
            f"joint {joint} escaped limits: {targets[index]} "
            f"not in [{lower}, {upper}]"
        )


def test_default_jog_step_is_in_simulated_range():
    """Sanity: the node's DEFAULT_JOG_STEP lies within the simulated step band.

    Keeps the jog simulation honest about the real default step size.

    **Validates: Requirements 8.3**
    """
    assert 0.01 <= DEFAULT_JOG_STEP <= 0.5
    # arm_teleop is import-safe (no ROS needed) — confirms the deferred-import
    # contract the tests rely on.
    assert hasattr(arm_teleop, "clamp_to_limits")
