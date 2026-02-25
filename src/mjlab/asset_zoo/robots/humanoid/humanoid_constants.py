"""Humanoid robot constants.

Humanoid is a 12-DOF bipedal robot with 6 joints per leg:
- hip_pitch, hip_roll, hip_yaw, knee, foot1, foot2

Motor specifications from Synapticon datasheets:
- Hip pitch: EC-A6416-P2-25 (peak 120 Nm, rated 55 Nm)
- Hip roll: EC-A5013-H17-100 (peak 90 Nm, rated 45 Nm)
- Hip yaw: EC-A3814-H14-107 (peak 60 Nm, rated 30 Nm)
- Knee: EC-A4315-P2-36 (peak 75 Nm, rated 50 Nm)
- Ankle pitch/roll: EC-A4310-P2-36 (peak 36 Nm, rated 18 Nm)
"""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

HUMANOID_XML: Path = (
    MJLAB_SRC_PATH / "asset_zoo" / "robots" / "humanoid" / "xmls" / "humanoid.xml"
)
assert HUMANOID_XML.exists(), f"Humanoid XML not found at {HUMANOID_XML}"

HUMANOID_WALKING_REFERENCE: Path = (
    MJLAB_SRC_PATH / "asset_zoo" / "robots" / "humanoid" / "assets" / "walking_reference.csv"
)


def get_assets(meshdir: str) -> dict[str, bytes]:
    assets: dict[str, bytes] = {}
    update_assets(assets, HUMANOID_XML.parent.parent / "assets" / "meshes", meshdir)
    return assets


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(HUMANOID_XML))
    spec.assets = get_assets(spec.meshdir)
    return spec


##
# Actuator config.
#
# Using physics-based PD gains:
# KP = J_reflected * omega_n^2 (with omega_n = 10 Hz * 2*pi)
# KD = 5.0 Nm·s/rad (hardware max for all motors)
##

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

# Reflected inertias from motor spec sheets (J_rotor * gear_ratio^2)
# Hip pitch: 104.395 kg·mm² * 25² = 0.0652 kg·m²
# Hip roll: 10 kg·mm² * 100² = 0.100 kg·m²
# Hip yaw: 3 kg·mm² * 107² = 0.0343 kg·m²
# Knee: 25.5 kg·mm² * 36² = 0.0330 kg·m²
# Ankle: 18.2 kg·mm² * 36² = 0.0236 kg·m²
ARMATURE_HIP_PITCH = 0.0652
ARMATURE_HIP_ROLL = 0.100
ARMATURE_HIP_YAW = 0.0343
ARMATURE_KNEE = 0.0330
ARMATURE_FOOT1 = 0.0236
ARMATURE_FOOT2 = 0.0236

# Stiffness (KP) - physics based
STIFFNESS_HIP_PITCH = ARMATURE_HIP_PITCH * NATURAL_FREQ**2
STIFFNESS_HIP_ROLL = ARMATURE_HIP_ROLL * NATURAL_FREQ**2
STIFFNESS_HIP_YAW = ARMATURE_HIP_YAW * NATURAL_FREQ**2
STIFFNESS_KNEE = ARMATURE_KNEE * NATURAL_FREQ**2
STIFFNESS_FOOT1 = ARMATURE_FOOT1 * NATURAL_FREQ**2
STIFFNESS_FOOT2 = ARMATURE_FOOT2 * NATURAL_FREQ**2

# Damping (KD) - capped at hardware max 5.0 Nm·s/rad for sim2real
DAMPING_HIP_PITCH = 5.0
DAMPING_HIP_ROLL = 5.0
DAMPING_HIP_YAW = 5.0
DAMPING_KNEE = 5.0
DAMPING_ANKLE = 5.0

# Effort limits (peak torque from datasheets)
EFFORT_HIP_PITCH = 120.0  # EC-A6416-P2-25: peak 120 Nm
EFFORT_HIP_ROLL = 90.0    # EC-A5013-H17-100: peak 90 Nm
EFFORT_HIP_YAW = 60.0     # EC-A3814-H14-107: peak 60 Nm
EFFORT_KNEE = 75.0        # EC-A4315-P2-36: peak 75 Nm
EFFORT_ANKLE = 36.0       # EC-A4310-P2-36: peak 36 Nm

HUMANOID_ACTUATOR_HIP_PITCH = BuiltinPositionActuatorCfg(
    joint_names_expr=(".*_hip_pitch_joint",),
    stiffness=STIFFNESS_HIP_PITCH,
    damping=DAMPING_HIP_PITCH,
    effort_limit=EFFORT_HIP_PITCH,
    armature=ARMATURE_HIP_PITCH,
)

HUMANOID_ACTUATOR_HIP_ROLL = BuiltinPositionActuatorCfg(
    joint_names_expr=(".*_hip_roll_joint",),
    stiffness=STIFFNESS_HIP_ROLL,
    damping=DAMPING_HIP_ROLL,
    effort_limit=EFFORT_HIP_ROLL,
    armature=ARMATURE_HIP_ROLL,
)

HUMANOID_ACTUATOR_HIP_YAW = BuiltinPositionActuatorCfg(
    joint_names_expr=(".*_hip_yaw_joint",),
    stiffness=STIFFNESS_HIP_YAW,
    damping=DAMPING_HIP_YAW,
    effort_limit=EFFORT_HIP_YAW,
    armature=ARMATURE_HIP_YAW,
)

HUMANOID_ACTUATOR_KNEE = BuiltinPositionActuatorCfg(
    joint_names_expr=(".*_knee_joint",),
    stiffness=STIFFNESS_KNEE,
    damping=DAMPING_KNEE,
    effort_limit=EFFORT_KNEE,
    armature=ARMATURE_KNEE,
)

HUMANOID_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
    joint_names_expr=(".*_foot1_joint", ".*_foot2_joint"),
    stiffness=STIFFNESS_FOOT1,
    damping=DAMPING_ANKLE,
    effort_limit=EFFORT_ANKLE,
    armature=ARMATURE_FOOT1,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0, 0, 0.75),  # Standing height (~0.72m + margin)
    joint_pos={".*": 0.0},  # All joints at zero = straight standing
    joint_vel={".*": 0.0},
)

# Knees bent pose for stability
# NOTE: Left and right sides have OPPOSITE axis directions!
# Knees:
# - Left knee: axis=(0, 1, 0), range=[0, 1.5] → positive = extend back
# - Right knee: axis=(0, -1, 0), range=[-1.5, 0] → negative = extend back
# Hip pitch:
# - Left: axis=(0, 0.707, -0.707) → positive = forward lean
# - Right: axis=(0, -0.707, -0.707) → negative = forward lean
# Ankle pitch:
# - Left: axis=(0, 1, 0) → negative = pitch down
# - Right: axis=(0, -1, 0) → positive = pitch down
KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0, 0, 0.70),  # Lower height for bent knees
    joint_pos={
        "left_hip_pitch_joint": 0.0,
        "right_hip_pitch_joint": 0.0,
        ".*_hip_roll_joint": 0.0,
        ".*_hip_yaw_joint": 0.0,
        "left_knee_joint": 0.4,    # [0, 1.5] range
        "right_knee_joint": -0.4,  # [-1.5, 0] range
        "left_foot1_joint": -0.2,   # Compensate for knee bend
        "right_foot1_joint": 0.2,   # Opposite sign
        ".*_foot2_joint": 0.0,
    },
    joint_vel={".*": 0.0},
)

##
# Collision config.
##

# Full collision including self-collisions
FULL_COLLISION = CollisionCfg(
    geom_names_expr=(".*_collision",),
    condim={
        r"^(left|right)_foot2_link_collision$": 3,
        ".*_collision": 1,
    },
    priority={r"^(left|right)_foot2_link_collision$": 1},
    friction={r"^(left|right)_foot2_link_collision$": (0.6,)},
)

# Feet only collision (disable self-collision)
FEET_ONLY_COLLISION = CollisionCfg(
    geom_names_expr=(r"^(left|right)_foot2_link_collision$",),
    contype=0,
    conaffinity=1,
    condim=3,
    priority=1,
    friction=(0.6,),
)

##
# Final config.
##

HUMANOID_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        HUMANOID_ACTUATOR_HIP_PITCH,
        HUMANOID_ACTUATOR_HIP_ROLL,
        HUMANOID_ACTUATOR_HIP_YAW,
        HUMANOID_ACTUATOR_KNEE,
        HUMANOID_ACTUATOR_ANKLE,
    ),
    soft_joint_pos_limit_factor=0.9,
)


def get_humanoid_robot_cfg() -> EntityCfg:
    """Get a fresh Humanoid robot configuration instance.

    Returns a new EntityCfg instance each time to avoid mutation issues when
    the config is shared across multiple places.
    """
    return EntityCfg(
        init_state=HOME_KEYFRAME,
        collisions=(FEET_ONLY_COLLISION,),
        spec_fn=get_spec,
        articulation=HUMANOID_ARTICULATION,
    )


# Action scale: how much the action scales joint position target
# Using 0.3 * effort_limit / stiffness (like humanoid-mjlab)
HUMANOID_ACTION_SCALE: dict[str, float] = {}
for a in HUMANOID_ARTICULATION.actuators:
    assert isinstance(a, BuiltinPositionActuatorCfg)
    e = a.effort_limit
    s = a.stiffness
    names = a.joint_names_expr
    assert e is not None
    assert s is not None
    for n in names:
        HUMANOID_ACTION_SCALE[n] = 0.3 * e / s


if __name__ == "__main__":
    import mujoco.viewer as viewer

    from mjlab.entity.entity import Entity

    robot = Entity(get_humanoid_robot_cfg())

    viewer.launch(robot.spec.compile())
