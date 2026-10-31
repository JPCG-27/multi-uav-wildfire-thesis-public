"""Restore optional recorded camera responses, verifying every original byte payload."""
from pathlib import Path, PurePosixPath
import argparse, base64, hashlib, json, zipfile
ROOT = Path(__file__).resolve().parent

def digest_file(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(data)
    return h.hexdigest()

def restore_bundle(data, item, destination, verify_only=False):
    if len(data) != item['bytes'] or hashlib.sha256(data).hexdigest() != item['sha256']:
        raise ValueError('Bundle checksum mismatch: ' + item['path'])
    count = size = 0
    expected_directory = PurePosixPath(item['original_directory'])
    directory_text = item['original_directory']
    if expected_directory.is_absolute() or '..' in expected_directory.parts or '\\' in directory_text or ':' in directory_text or expected_directory.parts[:2] != ('Saved', 'VisionCache'):
        raise ValueError('Unsafe original cache directory')
    target_directory = destination.joinpath(*expected_directory.parts)
    if not target_directory.resolve().is_relative_to(destination):
        raise ValueError('Cache directory escapes simulator/current')
    if not verify_only:
        target_directory.mkdir(parents=True, exist_ok=True)
        if not target_directory.resolve().is_relative_to(destination):
            raise ValueError('Cache directory escapes simulator/current')
    for line in data.splitlines():
        relative, encoded, expected = json.loads(line)
        path = PurePosixPath(relative)
        if path.is_absolute() or '..' in path.parts or '\\' in relative or ':' in relative:
            raise ValueError('Unsafe cache path: ' + relative)
        if path.parts[:2] != ('Saved', 'VisionCache') or path.parent != expected_directory or path.suffix != '.json':
            raise ValueError('Unexpected cache path: ' + relative)
        raw = base64.b64decode(encoded, validate=True)
        if hashlib.sha256(raw).hexdigest() != expected:
            raise ValueError('Raw response checksum mismatch: ' + relative)
        count += 1
        size += len(raw)
        if verify_only:
            continue
        target = target_directory / path.name
        if target.is_symlink():
            raise ValueError('Cache file is a symbolic link: ' + str(target))
        if target.exists():
            if not target.is_file() or digest_file(target) != expected:
                raise ValueError('Existing cache differs; move it aside first: ' + str(target))
        else:
            with target.open('xb') as output:
                output.write(raw)
    if count != item['raw_files'] or size != item['raw_bytes']:
        raise ValueError('Response count/size mismatch: ' + item['path'])
    return count, size

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', action='append', type=Path, help='Downloaded cache ZIP; repeat for both cohorts')
    parser.add_argument('--cohort', choices=['mixed_camera', 'four_camera'])
    parser.add_argument('--verify-only', action='store_true', help='Verify payloads without restoring files')
    parser.add_argument('--limit-runs', type=int, help='Verify/restore only this many selected runs')
    args = parser.parse_args()
    if args.limit_runs is not None and args.limit_runs < 1:
        parser.error('--limit-runs must be positive')
    manifest = json.loads((ROOT / 'camera-cache-files.json').read_text(encoding='utf-8'))
    if args.cohort and args.cohort not in manifest['cohorts']:
        parser.error('The requested cohort is absent from the completed cache manifest')
    destination = (ROOT / 'simulator/current').resolve()
    selected = {k: v for k, v in manifest['cohorts'].items() if not args.cohort or k == args.cohort}
    provided = {p.name: p.resolve() for p in args.archive or []}
    archives = {}
    for asset in manifest['assets']:
        if args.cohort and ('-' + args.cohort + '-') not in asset['file']:
            continue
        path = provided.get(asset['file'], ROOT / 'downloads' / asset['file'])
        if path.exists():
            if path.stat().st_size != asset['bytes'] or digest_file(path) != asset['sha256']:
                raise ValueError('Archive checksum mismatch: ' + str(path))
            archives[asset['file']] = zipfile.ZipFile(path)
    runs = files = byte_count = 0
    try:
        for cohort, spec in selected.items():
            for item in spec['bundles']:
                if args.limit_runs is not None and runs >= args.limit_runs:
                    break
                local = (ROOT / item['path']).resolve()
                if not local.is_relative_to(ROOT):
                    raise ValueError('Unsafe bundle path')
                if local.is_file():
                    data = local.read_bytes()
                else:
                    matches = [z for z in archives.values() if item['path'] in z.namelist()]
                    if len(matches) != 1:
                        raise FileNotFoundError('Extract the cache bundles or supply their downloaded ZIP with --archive: ' + item['path'])
                    data = matches[0].read(item['path'])
                n, size = restore_bundle(data, item, destination, args.verify_only)
                files += n
                byte_count += size
                runs += 1
                if runs % 100 == 0:
                    print(f'Processed {runs} runs / {files} responses', flush=True)
    finally:
        for z in archives.values():
            z.close()
    print(f'{"Verified" if args.verify_only else "Restored"} {runs} runs, {files} responses, {byte_count} original bytes.')
    print('Original configurations use record mode; replay requires vision_cache_mode=replay and the recorded vision_cache_dir.')

if __name__ == '__main__':
    main()
