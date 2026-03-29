import json
import os
import re
from copy import deepcopy

# ---------- Geometry transforms ----------

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

    for wp in new_data.get("waypoints", []):
        wp["anchor"] = mirror_point(wp.get("anchor"), field_width)
        wp["prevControl"] = mirror_point(wp.get("prevControl"), field_width)
        wp["nextControl"] = mirror_point(wp.get("nextControl"), field_width)

    for rt in new_data.get("rotationTargets", []):
        if 'rotation' in rt:
            rt["rotation"] = mirror_rotation(rt["rotation"])
        if 'rotationDegrees' in rt:
            rt["rotationDegrees"] = mirror_rotation(rt["rotationDegrees"])

    if new_data.get("goalEndState"):
        new_data["goalEndState"]["rotation"] = mirror_rotation(
            new_data["goalEndState"]["rotation"]
        )

    if new_data.get("idealStartingState"):
        new_data["idealStartingState"]["rotation"] = mirror_rotation(
            new_data["idealStartingState"]["rotation"]
        )

    return new_data


# ---------- Smart renaming ----------

def match_case(original, replacement):
    if original.isupper():
        return replacement.upper()
    if original.islower():
        return replacement.lower()
    if original[0].isupper():
        return replacement.capitalize()
    return replacement

def swap_direction_words(name):
    swaps = {
        "right": "left",
        "left": "right",
        "top": "bottom",
        "bottom": "top"
    }

    def replacer(match):
        word = match.group(0)
        lower = word.lower()
        swapped = swaps[lower]
        return match_case(word, swapped)

    pattern = re.compile(r"\b(right|left|top|bottom)\b", re.IGNORECASE)
    return pattern.sub(replacer, name)


# ---------- Batch processing ----------

def mirror_folder(input_dir, output_dir, field_width, overwrite=False):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for filename in os.listdir(input_dir):
        if not filename.endswith(".path"):
            continue

        input_path = os.path.join(input_dir, filename)

        # Rename file intelligently
        new_name = swap_direction_words(filename)
        output_path = os.path.join(output_dir, new_name)

        if os.path.exists(output_path) and not overwrite:
            print(f"Skipping (exists): {output_path}")
            continue

        with open(input_path, "r") as f:
            print(input_path)
            data = json.load(f)

        mirrored = process_path(data, field_width)

        with open(output_path, "w") as f:
            json.dump(mirrored, f, indent=2)

        print(f"Mirrored: {filename} -> {new_name}")


# ---------- Entry point ----------

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Batch mirror PathPlanner paths")
    parser.add_argument("input_dir", help="Folder containing .path files")
    parser.add_argument("output_dir", help="Folder to write mirrored files")
    parser.add_argument("field_width", type=float, help="Field width (meters)")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing files")

    args = parser.parse_args()

    mirror_folder(args.input_dir, args.output_dir, args.field_width, args.overwrite)