#!/usr/bin/env python3
"""Read-only validation of the 30 rosbag sources used in the paper."""

import argparse
import csv
import hashlib
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--bag-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()

    rows = list(csv.DictReader(args.manifest.open(encoding='utf-8')))
    if len(rows) != 30:
        raise RuntimeError(f'正式清单应为30行，实际为{len(rows)}行')
    seen = set()
    output_rows = []
    for row in rows:
        bag = args.bag_root / row['bag_dir']
        if row['bag_dir'] in seen:
            raise RuntimeError(f'清单中重复bag：{bag.name}')
        seen.add(row['bag_dir'])
        if not bag.is_dir() or not (bag / 'metadata.yaml').is_file():
            raise RuntimeError(f'找不到完整bag目录：{bag}')
        db_files = sorted(list(bag.glob('*.db3.zstd')) + list(bag.glob('*.db3')))
        if len(db_files) != 1:
            raise RuntimeError(f'{bag} 应有唯一数据库文件，实际：{db_files}')
        db = db_files[0]
        digest = hashlib.sha256()
        with db.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
        output_rows.append({**row, 'bag_file': db.name, 'bag_size_bytes': db.stat().st_size,
                            'bag_sha256': digest.hexdigest()})

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]))
        writer.writeheader()
        writer.writerows(output_rows)
    print(f'PASS: 已核验30个正式bag，清单写入 {args.output}')


if __name__ == '__main__':
    main()
