import json

from .parameter_state import SCHEMA_VERSION


PARAMETER_UPDATE_TOPIC = "/haptic_controller/parameters"


def build_ros_message_payload(parameters):
    return {
        "schema_version": SCHEMA_VERSION,
        "source": "haptic_parameter_gui",
        "parameters": parameters,
    }


def serialize_ros_message_payload(parameters):
    return json.dumps(build_ros_message_payload(parameters), sort_keys=True)


def flatten_parameters(parameters):
    flat = {}
    for section, values in parameters.items():
        for name, value in values.items():
            flat[f"{section}.{name}"] = value
    return flat
