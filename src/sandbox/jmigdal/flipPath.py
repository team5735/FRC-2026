import json
import sys
from copy import deepcopy

def mirror_y(value, field_width):
    return field_width - value

def mirror_point(pt, field_width):
    if pt is None:
        return None
    return {
        "x": pt["x"],
        "y": mirror_y(pt["y"], field_width)
    }

def mirror_rotation(deg):
    return -deg

def process_path(data, field_width):
    new_data = deepcopy(data)

    # Waypoints (anchors + bezier control points)
    for wp in new_data["waypoints"]:
        wp["anchor"] = mirror_point(wp["anchor"], field_width)
        wp["prevControl"] = mirror_point(wp["prevControl"], field_width)
        wp["nextControl"] = mirror_point(wp["nextControl"], field_width)

    # Rotation targets (if present in future files)
    for rt in new_data.get("rotationTargets", []):
        rt["rotation"] = mirror_rotation(rt["rotation"])

    # Goal end state
    if "goalEndState" in new_data and new_data["goalEndState"] is not None:
        new_data["goalEndState"]["rotation"] = mirror_rotation(
            new_data["goalEndState"]["rotation"]
        )

    # Ideal starting state
    if "idealStartingState" in new_data and new_data["idealStartingState"] is not None:
        new_data["idealStartingState"]["rotation"] = mirror_rotation(
            new_data["idealStartingState"]["rotation"]
        )

    return new_data


def main():
    if len(sys.argv) != 4:
        print("Usage: python mirror_path.py input.path output.path field_width_meters")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    field_width = float(sys.argv[3])

    with open(input_file, "r") as f:
        data = json.load(f)

    mirrored = process_path(data, field_width)

    with open(output_file, "w") as f:
        json.dump(mirrored, f, indent=2)

    print(f"Mirrored path written to {output_file}")


if __name__ == "__main__":
    main()