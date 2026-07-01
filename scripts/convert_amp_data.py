#!/usr/bin/env python3
"""
Convert NMPC-WBC CSV logs to AMP training format.

Usage:
    python3 convert_amp_data.py --input_dir amp_data --output_dir amp_dataset

Output format: .npy files compatible with LeggedGym/IsaacGym AMP training
Each file contains: [num_frames, num_features] array
Features: [q_joints(12), dq_joints(12), contact_flags(4)] = 28 dims
"""

import argparse
import csv
import glob
import os
import numpy as np


PROJECT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def load_csv(filepath):
    rows = []
    with open(filepath, 'r', newline='') as f:
        reader = csv.DictReader(f)
        header = list(reader.fieldnames or [])
        for raw_row in reader:
            row = {}
            for key, value in raw_row.items():
                if key == 'gait_name' or value is None or value == '':
                    continue
                row[key] = float(value)
            rows.append(row)
    return header, rows


def require_columns(header, columns, csv_file):
    missing = [name for name in columns if name not in header]
    if missing:
        raise RuntimeError(f"{csv_file} is missing required columns: {', '.join(missing)}")


def extract_amp_features(rows, header, csv_file, num_joints=12, num_feet=4):
    required_columns = [f'q{i}' for i in range(num_joints)]
    required_columns += [f'dq{i}' for i in range(num_joints)]
    required_columns += [f'contact{i}' for i in range(num_feet)]
    require_columns(header, required_columns, csv_file)

    features_list = []

    for row in rows:
        features = []

        features.extend(row[f'q{i}'] for i in range(num_joints))
        features.extend(row[f'dq{i}'] for i in range(num_joints))
        features.extend(row[f'contact{i}'] for i in range(num_feet))

        features_list.append(features)

    return np.array(features_list, dtype=np.float32)


def resample_data(data, target_freq=50.0, source_freq=50.0):
    if abs(target_freq - source_freq) < 0.1:
        return data
    ratio = source_freq / target_freq
    n_source = len(data)
    n_target = int(n_source / ratio)
    indices = np.linspace(0, n_source - 1, n_target).astype(int)
    return data[indices]


def normalize_data(data, mean=None, std=None):
    if mean is None:
        mean = np.mean(data, axis=0)
    if std is None:
        std = np.std(data, axis=0)
        std[std < 1e-6] = 1.0
    return (data - mean) / std, mean, std


def main():
    parser = argparse.ArgumentParser(description='Convert NMPC-WBC CSV logs to AMP training format')
    parser.add_argument('--input_dir', type=str, default=os.path.join(PROJECT_DIR, 'amp_data'),
                        help='Directory containing CSV log files')
    parser.add_argument('--output_dir', type=str, default=os.path.join(PROJECT_DIR, 'amp_dataset'),
                        help='Output directory for AMP dataset')
    parser.add_argument('--target_freq', type=float, default=50.0,
                        help='Target sampling frequency (Hz)')
    parser.add_argument('--normalize', action='store_true',
                        help='Whether to normalize the data')
    parser.add_argument('--min_sequence_length', type=int, default=50,
                        help='Minimum sequence length to keep')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    csv_files = sorted(glob.glob(os.path.join(args.input_dir, '*.csv')))
    if not csv_files:
        print(f"No CSV files found in {args.input_dir}")
        return

    print(f"Found {len(csv_files)} CSV files")

    all_sequences = []
    all_metadata = []

    for csv_file in csv_files:
        print(f"Processing: {os.path.basename(csv_file)}")
        header, data = load_csv(csv_file)

        if len(data) < args.min_sequence_length:
            print(f"  Skipping (too short: {len(data)} frames)")
            continue

        features = extract_amp_features(data, header, csv_file)
        features = resample_data(features, target_freq=args.target_freq)

        all_sequences.append(features)
        all_metadata.append({
            'file': os.path.basename(csv_file),
            'num_frames': len(features),
            'feature_dim': features.shape[1],
        })
        print(f"  Extracted {len(features)} frames, {features.shape[1]} features")

    if not all_sequences:
        print("No valid sequences found!")
        return

    combined = np.concatenate(all_sequences, axis=0)
    print(f"\nTotal: {combined.shape[0]} frames, {combined.shape[1]} features")

    if args.normalize:
        combined, mean, std = normalize_data(combined)
        np.save(os.path.join(args.output_dir, 'mean.npy'), mean)
        np.save(os.path.join(args.output_dir, 'std.npy'), std)
        print(f"Saved normalization stats")

    np.save(os.path.join(args.output_dir, 'amp_motions.npy'), combined)
    print(f"Saved combined data to {args.output_dir}/amp_motions.npy")

    for i, seq in enumerate(all_sequences):
        if args.normalize:
            seq, _, _ = normalize_data(seq, mean=mean, std=std)
        np.save(os.path.join(args.output_dir, f'sequence_{i:04d}.npy'), seq)

    with open(os.path.join(args.output_dir, 'metadata.txt'), 'w') as f:
        f.write(f"num_sequences: {len(all_sequences)}\n")
        f.write(f"total_frames: {combined.shape[0]}\n")
        f.write(f"feature_dim: {combined.shape[1]}\n")
        f.write(f"target_freq: {args.target_freq}\n")
        f.write(f"normalized: {args.normalize}\n")
        f.write(f"\nFeature layout:\n")
        f.write(f"  [0:12]  - joint positions (q)\n")
        f.write(f"  [12:24] - joint velocities (dq)\n")
        f.write(f"  [24:28] - contact flags (4 feet)\n")
        f.write(f"\nSequences:\n")
        for i, meta in enumerate(all_metadata):
            f.write(f"  {i}: {meta['file']} ({meta['num_frames']} frames)\n")

    print(f"\nMetadata saved to {args.output_dir}/metadata.txt")
    print("Done!")


if __name__ == '__main__':
    main()
