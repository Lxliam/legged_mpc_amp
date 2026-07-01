#!/usr/bin/env python3
"""
Convert NMPC-WBC AMP CSV logs to an IsaacLab-friendly motion dataset.

The source logger stores joints in OCS2 order:
  [LF, LH, RF, RH] x [HAA, HFE, KFE]

Most IsaacLab Unitree assets use front-left, front-right, rear-left,
rear-right order. By default this script remaps legs to:
  [FL, FR, RL, RR] x [hip/haa, thigh/hfe, calf/kfe]

Output:
  isaaclab_motions.npz
    motions              [total_frames, flat_dim]
    sequence_lengths     [num_sequences]
    sequence_names       [num_sequences]
    fps                  scalar
    root_pos             [total_frames, 3]
    root_rot_rpy         [total_frames, 3]  yaw, pitch, roll from the CSV
    root_lin_vel_b       [total_frames, 3]  base linear velocity in body frame
    root_ang_vel_b       [total_frames, 3]  base angular velocity in body frame
    joint_pos            [total_frames, 12]
    joint_vel            [total_frames, 12]
    foot_contact         [total_frames, 4]

  sequences/
    <sequence_name>.npy       flat motion array for one source CSV
    <sequence_name>.npz       optional named arrays for the same source CSV

  isaaclab_motion_metadata.json
    human-readable layout, source files, and remapping details.
"""

import argparse
import csv
import glob
import json
import os
from typing import Dict, Iterable, List, Tuple

import numpy as np


PROJECT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

SOURCE_LEG_ORDER = ("LF", "LH", "RF", "RH")
ISAACLAB_LEG_ORDER = ("FL", "FR", "RL", "RR")
SOURCE_TO_ISAACLAB_LEG = {
    "FL": "LF",
    "FR": "RF",
    "RL": "LH",
    "RR": "RH",
}
JOINTS_PER_LEG = 3

BODY_VELOCITY_COLUMNS = (
    "root_lin_vel_bx",
    "root_lin_vel_by",
    "root_lin_vel_bz",
    "root_ang_vel_bx",
    "root_ang_vel_by",
    "root_ang_vel_bz",
)


def parse_leg_order(value: str) -> Tuple[str, ...]:
    order = tuple(part.strip().upper() for part in value.replace(",", " ").split() if part.strip())
    if len(order) != 4:
        raise argparse.ArgumentTypeError("leg order must contain exactly 4 legs")
    valid = set(ISAACLAB_LEG_ORDER)
    if set(order) != valid:
        raise argparse.ArgumentTypeError(f"leg order must be a permutation of {sorted(valid)}")
    return order


def read_numeric_csv(path: str) -> Tuple[List[str], List[Dict[str, float]]]:
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return [], rows
        header = list(reader.fieldnames)
        for raw_row in reader:
            row = {}
            for key, value in raw_row.items():
                if key == "gait_name" or value is None or value == "":
                    continue
                try:
                    row[key] = float(value)
                except ValueError:
                    continue
            rows.append(row)
    return header, rows


def require_columns(header: Iterable[str], columns: Iterable[str], csv_file: str) -> None:
    header_set = set(header)
    missing = [name for name in columns if name not in header_set]
    if missing:
        raise RuntimeError(f"{csv_file} is missing required columns: {', '.join(missing)}")


def source_leg_index(source_leg: str) -> int:
    return SOURCE_LEG_ORDER.index(source_leg)


def build_joint_indices(target_leg_order: Tuple[str, ...], remap_to_isaaclab: bool) -> List[int]:
    if not remap_to_isaaclab:
        return list(range(12))

    indices = []
    for target_leg in target_leg_order:
        source_leg = SOURCE_TO_ISAACLAB_LEG[target_leg]
        start = source_leg_index(source_leg) * JOINTS_PER_LEG
        indices.extend([start, start + 1, start + 2])
    return indices


def build_foot_indices(target_leg_order: Tuple[str, ...], remap_to_isaaclab: bool) -> List[int]:
    if not remap_to_isaaclab:
        return list(range(4))

    return [source_leg_index(SOURCE_TO_ISAACLAB_LEG[leg]) for leg in target_leg_order]


def take_columns(row: Dict[str, float], prefix: str, count: int) -> np.ndarray:
    return np.array([row[f"{prefix}{i}"] for i in range(count)], dtype=np.float32)


def resample_sequence(arrays: Dict[str, np.ndarray], target_fps: float, source_fps: float) -> Dict[str, np.ndarray]:
    if abs(target_fps - source_fps) < 1e-6:
        return arrays

    n_source = len(next(value for value in arrays.values() if isinstance(value, np.ndarray)))
    n_target = max(1, int(round(n_source * target_fps / source_fps)))
    indices = np.linspace(0, n_source - 1, n_target).round().astype(np.int64)
    return {key: value[indices] if isinstance(value, np.ndarray) else value for key, value in arrays.items()}


def build_flat_motion(arrays: Dict[str, np.ndarray], layout: List[str]) -> np.ndarray:
    parts = []
    for key in layout:
        value = arrays[key]
        if value.ndim == 1:
            value = value[:, None]
        parts.append(value)
    return np.concatenate(parts, axis=1).astype(np.float32)


def concat_by_key(sequences: List[Dict[str, np.ndarray]]) -> Dict[str, np.ndarray]:
    keys = [key for key, value in sequences[0].items() if isinstance(value, np.ndarray)]
    return {key: np.concatenate([seq[key] for seq in sequences], axis=0) for key in keys}


def save_individual_sequence(output_dir: str, sequence_name: str, arrays: Dict[str, np.ndarray],
                             target_fps: float, save_npz: bool) -> Dict[str, str]:
    os.makedirs(output_dir, exist_ok=True)
    safe_name = os.path.basename(sequence_name)
    output_npy = os.path.join(output_dir, f"{safe_name}.npy")

    np.save(output_npy, arrays["motion"])
    outputs = {
        "motion_npy": os.path.relpath(output_npy, os.path.dirname(output_dir)),
    }
    if save_npz:
        output_npz = os.path.join(output_dir, f"{safe_name}.npz")
        np.savez(
            output_npz,
            motion=arrays["motion"],
            fps=np.array(target_fps, dtype=np.float32),
            **{key: value for key, value in arrays.items() if key != "motion" and isinstance(value, np.ndarray)},
        )
        outputs["motion_npz"] = os.path.relpath(output_npz, os.path.dirname(output_dir))
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert AMP CSV logs to an IsaacLab-friendly motion dataset")
    parser.add_argument("--input_dir", type=str, default=os.path.join(PROJECT_DIR, "amp_data"),
                        help="Directory containing CSV log files")
    parser.add_argument("--output_dir", type=str, default=os.path.join(PROJECT_DIR, "amp_dataset_isaaclab"),
                        help="Output directory")
    parser.add_argument("--source_fps", type=float, default=50.0,
                        help="Source logging frequency")
    parser.add_argument("--target_fps", type=float, default=50.0,
                        help="Output motion frequency")
    parser.add_argument("--min_sequence_length", type=int, default=50,
                        help="Minimum frames required before resampling")
    parser.add_argument("--no_remap_to_isaaclab", action="store_true",
                        help="Keep the source [LF,LH,RF,RH] leg order")
    parser.add_argument("--isaaclab_leg_order", type=parse_leg_order, default=ISAACLAB_LEG_ORDER,
                        help="Target leg order when remapping, default: FL FR RL RR")
    parser.add_argument("--flat_layout", type=str,
                        default="root_pos,root_rot_rpy,root_lin_vel_b,root_ang_vel_b,joint_pos,joint_vel,foot_contact",
                        help="Comma-separated keys to concatenate into the motions array")
    parser.add_argument("--sequence_output_dir", type=str, default="sequences",
                        help="Directory for one-file-per-CSV outputs, relative to output_dir unless absolute")
    parser.add_argument("--no_save_individual", action="store_true",
                        help="Only save the legacy combined isaaclab_motions.npz")
    parser.add_argument("--save_individual_npz", action="store_true",
                        help="Also save one structured .npz per CSV for debugging. Default saves only per-sequence .npy files.")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    csv_files = sorted(glob.glob(os.path.join(args.input_dir, "*.csv")))
    if not csv_files:
        print(f"No CSV files found in {args.input_dir}")
        return

    remap_to_isaaclab = not args.no_remap_to_isaaclab
    joint_indices = build_joint_indices(args.isaaclab_leg_order, remap_to_isaaclab)
    foot_indices = build_foot_indices(args.isaaclab_leg_order, remap_to_isaaclab)
    flat_layout = [key.strip() for key in args.flat_layout.split(",") if key.strip()]

    required_columns = [
        "time",
        "base_px",
        "base_py",
        "base_pz",
        "base_yaw",
        "base_pitch",
        "base_roll",
        *BODY_VELOCITY_COLUMNS,
    ]
    required_columns += [f"q{i}" for i in range(12)]
    required_columns += [f"dq{i}" for i in range(12)]
    required_columns += [f"contact{i}" for i in range(4)]

    sequences = []
    sequence_names = []
    sequence_lengths = []
    source_files = []
    sequence_outputs = []
    sequence_output_dir = args.sequence_output_dir
    if not os.path.isabs(sequence_output_dir):
        sequence_output_dir = os.path.join(args.output_dir, sequence_output_dir)

    for csv_file in csv_files:
        header, rows = read_numeric_csv(csv_file)
        if len(rows) < args.min_sequence_length:
            print(f"Skipping {os.path.basename(csv_file)}: too short ({len(rows)} frames)")
            continue
        require_columns(header, required_columns, csv_file)

        per_frame = {
            "time": [],
            "root_pos": [],
            "root_rot_rpy": [],
            "root_lin_vel_b": [],
            "root_ang_vel_b": [],
            "joint_pos": [],
            "joint_vel": [],
            "foot_contact": [],
        }

        for row in rows:
            joint_pos = take_columns(row, "q", 12)[joint_indices]
            joint_vel = take_columns(row, "dq", 12)[joint_indices]
            foot_contact = take_columns(row, "contact", 4)[foot_indices]

            per_frame["time"].append(np.array([row["time"]], dtype=np.float32))
            per_frame["root_pos"].append(np.array([row["base_px"], row["base_py"], row["base_pz"]], dtype=np.float32))
            per_frame["root_rot_rpy"].append(np.array([row["base_yaw"], row["base_pitch"], row["base_roll"]], dtype=np.float32))
            per_frame["root_lin_vel_b"].append(
                np.array([row["root_lin_vel_bx"], row["root_lin_vel_by"], row["root_lin_vel_bz"]], dtype=np.float32)
            )
            per_frame["root_ang_vel_b"].append(
                np.array([row["root_ang_vel_bx"], row["root_ang_vel_by"], row["root_ang_vel_bz"]], dtype=np.float32)
            )
            per_frame["joint_pos"].append(joint_pos)
            per_frame["joint_vel"].append(joint_vel)
            per_frame["foot_contact"].append(foot_contact)

        arrays = {key: np.stack(value, axis=0).astype(np.float32) for key, value in per_frame.items()}
        arrays = resample_sequence(arrays, args.target_fps, args.source_fps)

        invalid_keys = [key for key in flat_layout if key not in arrays]
        if invalid_keys:
            raise RuntimeError(f"Unknown flat layout keys: {', '.join(invalid_keys)}")
        arrays["motion"] = build_flat_motion(arrays, flat_layout)

        sequences.append(arrays)
        sequence_names.append(os.path.splitext(os.path.basename(csv_file))[0])
        sequence_lengths.append(arrays["motion"].shape[0])
        source_files.append(os.path.basename(csv_file))
        if args.no_save_individual:
            sequence_outputs.append({})
        else:
            sequence_outputs.append(
                save_individual_sequence(sequence_output_dir, sequence_names[-1], arrays, args.target_fps, args.save_individual_npz)
            )
        print(f"Processed {os.path.basename(csv_file)}: {arrays['motion'].shape[0]} frames, flat dim {arrays['motion'].shape[1]}")

    if not sequences:
        print("No valid sequences found")
        return

    combined = concat_by_key(sequences)
    motions = combined.pop("motion")
    sequence_lengths_array = np.array(sequence_lengths, dtype=np.int32)
    sequence_names_array = np.array(sequence_names)

    output_npz = os.path.join(args.output_dir, "isaaclab_motions.npz")
    np.savez(
        output_npz,
        motions=motions,
        sequence_lengths=sequence_lengths_array,
        sequence_names=sequence_names_array,
        fps=np.array(args.target_fps, dtype=np.float32),
        **combined,
    )

    layout_dims = {key: int(combined[key].shape[1]) for key in flat_layout}
    flat_slices = {}
    cursor = 0
    for key in flat_layout:
        dim = layout_dims[key]
        flat_slices[key] = [cursor, cursor + dim]
        cursor += dim

    metadata = {
        "format": "legged_mpc_amp_isaaclab_npz",
        "fps": args.target_fps,
        "num_sequences": len(sequences),
        "total_frames": int(motions.shape[0]),
        "flat_dim": int(motions.shape[1]),
        "flat_layout": flat_layout,
        "flat_slices": flat_slices,
        "root_velocity_frame": "body",
        "source_leg_order": list(SOURCE_LEG_ORDER),
        "target_leg_order": list(args.isaaclab_leg_order if remap_to_isaaclab else SOURCE_LEG_ORDER),
        "joint_order": [
            f"{leg}_{joint}"
            for leg in (args.isaaclab_leg_order if remap_to_isaaclab else SOURCE_LEG_ORDER)
            for joint in ("HAA", "HFE", "KFE")
        ],
        "foot_order": list(args.isaaclab_leg_order if remap_to_isaaclab else SOURCE_LEG_ORDER),
        "remap_to_isaaclab": remap_to_isaaclab,
        "root_rot_rpy_order": ["yaw", "pitch", "roll"],
        "notes": [
            "root_lin_vel_b and root_ang_vel_b are intended to match IsaacLab asset.data.root_lin_vel_b and asset.data.root_ang_vel_b.",
            "CSV files must contain body-frame velocity columns. Recollect old logs if they only contain legacy centroidal momentum fields.",
            "Use the named arrays when possible. Use motions plus flat_slices only if your IsaacLab motion loader expects one flat observation vector.",
            "The combined motions array is a concatenation of independent sequences. Do not sample frame windows across sequence_lengths boundaries.",
            "For IsaacLab loaders that do not understand sequence_lengths, prefer the per-sequence .npy files under sequence_output_dir.",
            "Per-sequence .npz files are optional debug outputs and are only written with --save_individual_npz.",
        ],
        "sequence_output_dir": None if args.no_save_individual else os.path.relpath(sequence_output_dir, args.output_dir),
        "sequences": [
            {
                "name": name,
                "source_file": source_file,
                "num_frames": int(length),
                **outputs,
            }
            for name, source_file, length, outputs in zip(sequence_names, source_files, sequence_lengths, sequence_outputs)
        ],
    }

    output_json = os.path.join(args.output_dir, "isaaclab_motion_metadata.json")
    with open(output_json, "w") as f:
        json.dump(metadata, f, indent=2)
        f.write("\n")

    print(f"Saved {output_npz}")
    if not args.no_save_individual:
        print(f"Saved individual sequences to {sequence_output_dir}")
    print(f"Saved {output_json}")


if __name__ == "__main__":
    main()
