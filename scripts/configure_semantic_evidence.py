#!/usr/bin/env python3
"""Create a separate evidence profile, preserving calibration and storage options."""
import argparse
from pathlib import Path
import yaml
from semantic_evidence import evidence_options


def configure(source, destination, export_uncertain=False):
    cfg = yaml.safe_load(Path(source).read_text())
    cfg['semantic_evidence'] = dict(enabled=True, min_classified_ratio=0.6,
                                    min_positive_baseline_m=0.05,
                                    write_uncertain=export_uncertain)
    evidence_options(cfg)
    with Path(destination).open('x') as stream:
        yaml.safe_dump(cfg, stream, allow_unicode=True, sort_keys=False)
    return cfg


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--export-uncertain', action='store_true', help='Additional world-frame PCD; can be large')
    args = parser.parse_args()
    configure(args.input, args.output, args.export_uncertain)
    print(f'Created {args.output}')
