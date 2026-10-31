"""Download distribution release attachments and verify them before extraction."""
from pathlib import Path
import argparse, hashlib, json, subprocess, zipfile

ROOT = Path(__file__).resolve().parent

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--group', help='Filename prefix, e.g. scenario-inputs')
    p.add_argument('--extract', action='store_true')
    p.add_argument('--repo', default='JPCG-27/multi-uav-wildfire-thesis-public')
    p.add_argument('--tag', default='reproduction-package')
    p.add_argument('--archive-dir', type=Path, help='Verify/extract local ZIPs without downloading')
    args = p.parse_args()
    items = []
    for name in ('release-files.json', 'supplemental-release-files.json', 'camera-cache-files.json'):
        path = ROOT / name
        if path.exists():
            items.extend(json.loads(path.read_text(encoding='utf-8'))['assets'])
    selected = [a for a in items if not args.group or a['file'].startswith(args.group)]
    if not selected:
        p.error('No matching attachment group')
    downloads = args.archive_dir.resolve() if args.archive_dir else ROOT / 'downloads'
    if args.archive_dir and not downloads.is_dir():
        p.error('--archive-dir must be an existing directory')
    downloads.mkdir(exist_ok=True)
    for item in selected:
        target = downloads / item['file']
        if not target.exists():
            if args.archive_dir:
                raise FileNotFoundError(f'Missing local archive: {target}')
            subprocess.run(['gh', 'release', 'download', args.tag, '--repo',
                            args.repo, '--pattern', item['file'],
                            '--dir', str(downloads)], check=True)
        if target.stat().st_size != item['bytes'] or digest(target) != item['sha256']:
            raise ValueError(f'Checksum mismatch: {target}; move it aside and retry')
        print(f'Verified {target.name}', flush=True)
        if args.extract:
            with zipfile.ZipFile(target) as z:
                for member in z.infolist():
                    resolved = (ROOT / member.filename).resolve()
                    if not resolved.is_relative_to(ROOT) or ((member.external_attr >> 16) & 0o170000) == 0o120000:
                        raise ValueError(f'Unsafe archive member: {member.filename}')
                z.extractall(ROOT)
            print(f'Extracted {target.name}', flush=True)

if __name__ == '__main__':
    main()
