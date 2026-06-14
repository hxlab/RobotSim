from dataclasses import asdict, dataclass, field


SCHEMA_VERSION = 1


@dataclass
class ControllerParameters:
    zeta: float = 1.0
    mass: float = 10.0
    damping: float = 140.0
    haptic_damping: float = 15.0
    k_orient: float = 50.0


@dataclass
class WorkspaceLimits:
    x_min: float = 0.2
    x_max: float = 0.6
    y_min: float = -0.25
    y_max: float = 0.25
    z_min: float = 0.1
    z_max: float = 0.7


@dataclass
class SharedControlParameters:
    enabled: bool = False
    grasp_assist: float = 0.0
    target_pursuance_assist: float = 0.0
    desired_trajectory_track: float = 0.0


@dataclass
class ParameterState:
    controller: ControllerParameters = field(default_factory=ControllerParameters)
    workspace_limits: WorkspaceLimits = field(default_factory=WorkspaceLimits)
    shared_control: SharedControlParameters = field(default_factory=SharedControlParameters)

    def to_payload(self):
        self.validate()
        return asdict(self)

    def to_ros_message_payload(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "source": "haptic_parameter_gui",
            "parameters": self.to_payload(),
        }

    def validate(self):
        errors = []
        limits = self.workspace_limits
        shared = self.shared_control
        controller = self.controller

        if limits.x_min >= limits.x_max:
            errors.append("x_min must be less than x_max")
        if limits.y_min >= limits.y_max:
            errors.append("y_min must be less than y_max")
        if limits.z_min >= limits.z_max:
            errors.append("z_min must be less than z_max")

        if controller.mass <= 0:
            errors.append("mass must be greater than 0")
        if controller.damping < 0:
            errors.append("damping must be greater than or equal to 0")
        if controller.haptic_damping < 0:
            errors.append("haptic_damping must be greater than or equal to 0")
        if controller.k_orient < 0:
            errors.append("k_orient must be greater than or equal to 0")
        if controller.zeta < 0:
            errors.append("zeta must be greater than or equal to 0")

        for name in (
            "grasp_assist",
            "target_pursuance_assist",
            "desired_trajectory_track",
        ):
            value = getattr(shared, name)
            if value < 0.0 or value > 1.0:
                errors.append(f"{name} must be between 0.0 and 1.0")

        if errors:
            raise ValueError("\n".join(errors))
