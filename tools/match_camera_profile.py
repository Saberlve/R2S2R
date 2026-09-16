"""Offline, zero-distortion same-optical-frame resampling between explicit profiles.

Profiles contain width, height, K and D. No camera or robot is opened.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def validate_profile(profile):
    w, h = profile['width'], profile['height']
    K = np.asarray(profile['K'], dtype=float)
    D = np.asarray(profile['D'], dtype=float)
    if not (isinstance(w, int) and isinstance(h, int) and w > 0 and h > 0):
        raise ValueError('Explicit positive integer width/height required')
    if (K.shape != (3, 3) or not np.isfinite(K).all()
            or K[0, 0] <= 0 or K[1, 1] <= 0
            or not np.allclose(K[2], [0, 0, 1])
            or K[0, 1] != 0 or K[1, 0] != 0):
        raise ValueError('Standard finite pinhole K required')
    if not np.isfinite(D).all() or np.any(D != 0):
        raise ValueError('Only explicitly zero distortion is supported')
    return w, h, K


def mapping(source, target):
    sw, sh, sk = validate_profile(source)
    tw, th, tk = validate_profile(target)
    if source.get('serial') != target.get('serial'):
        raise ValueError('Profiles must describe the same camera')
    H = sk @ np.linalg.inv(tk)
    yy, xx = np.mgrid[:th, :tw]
    points = H @ np.stack([xx.ravel(), yy.ravel(), np.ones(tw * th)])
    x = (points[0] / points[2]).reshape(th, tw)
    y = (points[1] / points[2]).reshape(th, tw)
    # Subpixel rounding at an edge is handled by replicate, never black padding.
    if x.min() < -.5 or y.min() < -.5 or x.max() > sw-.5 or y.max() > sh-.5:
        raise ValueError('Target field of view extends beyond source image')
    return x.astype(np.float32), y.astype(np.float32), H


def convert(image, source, target):
    sw, sh, _ = validate_profile(source)
    if image is None or image.shape[:2] != (sh, sw):
        raise ValueError('Image dimensions do not match declared source profile')
    x, y, _ = mapping(source, target)
    return cv2.remap(image, x, y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source-profile', required=True)
    ap.add_argument('--target-profile', required=True)
    ap.add_argument('--out', required=True, help='Fresh output directory')
    ap.add_argument('images', nargs='+')
    args = ap.parse_args()
    source = json.loads(Path(args.source_profile).read_text())
    target = json.loads(Path(args.target_profile).read_text())
    _, _, H = mapping(source, target)
    paths = [Path(x) for x in args.images]
    if len({p.stem for p in paths}) != len(paths):
        raise ValueError('Input image stems must be unique')
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    records = []
    for p in paths:
        image = cv2.imread(str(p), cv2.IMREAD_COLOR)
        result = convert(image, source, target)
        dest = out / (p.stem + '.png')
        if not cv2.imwrite(str(dest), result):
            raise IOError(dest)
        records.append({'source': str(p.resolve()), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest(), 'output': dest.name})
    (out / 'pixel_mapping.json').write_text(json.dumps({
        'schema_version': '1.0', 'source_profile': source, 'target_profile': target,
        'T_source_pixels_target_pixels': H.tolist(),
        'method': 'cv2.remap; INTER_LINEAR; BORDER_REPLICATE',
        'pixel_convention': 'integer coordinates are pixel centers; source = K_source @ inv(K_target) @ target',
        'assumption': 'same optical frame, zero distortion, unchanged extrinsic',
        'limitation': 'Matches rays/FOV, not hardware ISP, exposure or downsampling kernel',
        'images': records,
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
