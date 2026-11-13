#!/usr/bin/env python3
"""Fix metadata.yaml by converting multi-line type_description_hash to single-line format"""

import re
import sys

def fix_metadata(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Pattern: "type_description_hash: \n      RIHS01_..." -> "type_description_hash: RIHS01_..."
    # Find lines with "type_description_hash:" followed by newline and indented hash value
    pattern = r'type_description_hash: \n\s+(RIHS01_[a-f0-9]+)'
    fixed_content = re.sub(pattern, r'type_description_hash: \1', content)

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(fixed_content)

    print(f"Fixed metadata written to: {output_file}")

if __name__ == '__main__':
    input_path = "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2/metadata.yaml"
    output_path = "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2/metadata.yaml"
    fix_metadata(input_path, output_path)
