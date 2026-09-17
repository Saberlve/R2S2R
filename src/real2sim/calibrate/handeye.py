#!/usr/bin/env python3
"""Static-board, eye-in-hand calibration. Robot interfaces are READ ONLY.
T_A_B transforms coordinates B -> A; all stored translations are meters.
"""
import argparse
import datetime as dt
import json
import pathlib
import sys
import time
import itertools
import numpy as np
import cv2
from scipy.spatial.transform import Rotation

ROOT = pathlib.Path(__file__).resolve().parent
PROJECT = ROOT.parent
sys.path.insert(0, str(PROJECT / 'deps'))
sys.path.insert(0, str(PROJECT / 'robot_sync_v1/deps/xArm-Python-SDK-master'))

def save(path, value):
    path = pathlib.Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    tmp.replace(path)

def load(path):
    return json.loads(pathlib.Path(path).read_text(encoding='utf-8'))

def transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T

def pose_aa(p):
    p = np.asarray(p, dtype=float)
    if p.shape != (6,) or not np.isfinite(p).all():
        raise ValueError('Invalid xArm axis-angle pose')
    return transform(Rotation.from_rotvec(p[3:]).as_matrix(), p[:3] / 1000)

def delta(A, B):
    return (float(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000),
            float(np.rad2deg(Rotation.from_matrix(A[:3, :3].T @ B[:3, :3]).magnitude())))

def mean_pose(Ts):
    return transform(Rotation.from_matrix(np.array([t[:3, :3] for t in Ts])).mean().as_matrix(),
                     np.mean([t[:3, 3] for t in Ts], axis=0))

def board_center_pose(T_parent_board, board_config):
    """Convert an OpenCV ChArUco outer-corner frame pose to its geometric center.

    The center frame keeps the board axes: +x across squares, +y down the board,
    and +z out according to the pose returned by solvePnP.
    """
    T = np.asarray(T_parent_board, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('Invalid board pose matrix')
    sx, sy = int(board_config['squares_x']), int(board_config['squares_y'])
    square = float(board_config['square_mm']) / 1000
    if sx < 1 or sy < 1 or not np.isfinite(square) or square <= 0:
        raise ValueError('Invalid board dimensions')
    T_board_center = transform(np.eye(3), [sx * square / 2, sy * square / 2, 0])
    return T @ T_board_center

def fixed_camera_pose(T_base_board_center, T_fixed_camera_board, board_config):
    """Return the fixed optical camera pose in robot base from one board view."""
    B = np.asarray(T_base_board_center, dtype=float)
    C = board_center_pose(T_fixed_camera_board, board_config)
    if B.shape != (4, 4) or not np.isfinite(B).all():
        raise ValueError('Invalid base-to-board-center pose')
    T = B @ np.linalg.inv(C)
    return {'T_base_fixed_camera': T.tolist(),
            'T_fixed_camera_base': np.linalg.inv(T).tolist(),
            'fixed_camera_position_in_base_mm': (T[:3, 3] * 1000).tolist(),
            'fixed_camera_quaternion_xyzw_in_base': Rotation.from_matrix(T[:3, :3]).as_quat().tolist(),
            'T_fixed_camera_board_center': C.tolist()}

def board_from(c):
    b = c['board']
    if not (b['squares_x'] >= 3 and b['squares_y'] >= 3 and
            0 < b['marker_mm'] < b['square_mm'] and b['square_mm'] < 1000):
        raise ValueError('Invalid measured board dimensions')
    board = cv2.aruco.CharucoBoard((b['squares_x'], b['squares_y']), b['square_mm']/1000,
                                 b['marker_mm']/1000, cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, b['dictionary'])))
    board.setLegacyPattern(b['legacy'])
    return board

def intrinsics(c):
    K, D = np.asarray(c['K'], float), np.asarray(c['D'], float)
    if K.shape != (3, 3) or D.size not in (4, 5, 8, 12, 14) or not np.isfinite(K).all() or not np.isfinite(D).all():
        raise ValueError('Invalid OpenCV camera intrinsics')
    if K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1]):
        raise ValueError('Invalid camera matrix')
    return K, D

def detect(image, board, camera):
    K, D = intrinsics(camera)
    if image.shape[1::-1] != (camera['width'], camera['height']):
        raise ValueError('Image size differs from intrinsics')
    params = cv2.aruco.CharucoParameters()
    params.cameraMatrix, params.distCoeffs = K, D
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board, params).detectBoard(image)
    if ids is None or len(ids) < 10:
        raise ValueError('Fewer than 10 ChArUco corners; move closer or improve visibility')
    ids = ids.reshape(-1)
    obj = board.getChessboardCorners()[ids].astype(np.float64)
    pts = corners.reshape(-1, 2).astype(np.float64)
    if np.linalg.matrix_rank(obj[:, :2] - obj[:, :2].mean(0)) < 2:
        raise ValueError('Collinear board corners')
    # IPPE exposes the two planar pose hypotheses; reject near-ambiguous views.
    out = cv2.solvePnPGeneric(obj, pts, K, D, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for r, t in zip(out[1], out[2]):
        R = cv2.Rodrigues(r)[0]
        if np.min((R @ obj.T + t.reshape(3, 1))[2]) <= 0:
            continue
        projected = cv2.projectPoints(obj, r, t, K, D)[0].reshape(-1, 2)
        candidates.append((float(np.sqrt(np.mean(np.sum((projected-pts)**2, axis=1)))), r, t))
    candidates.sort(key=lambda v: v[0])
    if not candidates:
        raise ValueError('No positive-depth board pose')
    if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 0.15:
        raise ValueError('Planar pose ambiguous; use a more oblique view')
    _, r, t = candidates[0]
    r, t = cv2.solvePnPRefineLM(obj, pts, K, D, r, t)
    projected = cv2.projectPoints(obj, r, t, K, D)[0].reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((projected-pts)**2, axis=1))))
    if not np.isfinite(rms) or rms > 1.0:
        raise ValueError(f'Reprojection RMS {rms:.3f}px exceeds 1px')
    T = transform(cv2.Rodrigues(r)[0], t)
    if np.min((T[:3, :3] @ obj.T + T[:3, 3:4])[2]) <= 0:
        raise ValueError('Refined pose has negative depth')
    preview = image.copy()
    cv2.aruco.drawDetectedCornersCharuco(preview, corners, ids.reshape(-1, 1))
    cv2.drawFrameAxes(preview, K, D, r, t, 0.05)
    return {'T_camera_board': T.tolist(), 'corners': len(ids), 'ids': ids.tolist(),
            'corners_px': pts.tolist(), 'reprojection_rms_px': rms}, preview

class Camera:
    def __init__(self, c):
        self.c, self.pipeline, self.cap = c, None, None
        if c['backend'] == 'realsense':
            import pyrealsense2 as rs
            self.pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(c['serial'])
            cfg.enable_stream(rs.stream.color, c['width'], c['height'], rs.format.bgr8, c['fps'])
            profile = self.pipeline.start(cfg)
            try:
                k = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
                # Do not reinterpret inverse/modified Brown or fisheye as OpenCV Brown.
                if k.model not in (rs.distortion.none, rs.distortion.brown_conrady) and any(abs(v) > 1e-12 for v in k.coeffs):
                    raise ValueError(f'Unsupported RealSense distortion model: {k.model}')
                self.meta = dict(c, K=[[k.fx, 0, k.ppx], [0, k.fy, k.ppy], [0, 0, 1]],
                                 D=list(k.coeffs), distortion_model=str(k.model))
            except Exception:
                self.close()
                raise
        else:
            intrinsics(c)
            self.meta = c.copy()
            self.cap = cv2.VideoCapture(c['device'], cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.close()
                raise ValueError('Camera could not be opened')
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, c['width'])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c['height'])
            self.cap.set(cv2.CAP_PROP_FPS, c['fps'])

    def frame(self):
        if self.pipeline:
            f = self.pipeline.wait_for_frames(10000).get_color_frame()
            if not f:
                raise ValueError('No color frame')
            return np.asanyarray(f.get_data()).copy(), {'device_timestamp_ms': f.get_timestamp(),
                    'frame_number': f.get_frame_number(), 'timestamp_domain': str(f.get_frame_timestamp_domain())}
        ok, im = self.cap.read()
        if not ok:
            raise ValueError('Camera read failed')
        return im, {'device_timestamp_ms': None}

    def close(self):
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None
        if self.cap:
            self.cap.release()
            self.cap = None

def checked(result):
    code, value = result
    if code != 0:
        raise ValueError(f'xArm read failed, code {code}')
    return value

def read_robot(arm):
    state = checked(arm.get_state())
    if state in (1, 4):
        raise ValueError(f'Robot moving or stopped with error (state={state})')
    p = checked(arm.get_position_aa(is_radian=True))
    return {'utc': dt.datetime.now(dt.timezone.utc).isoformat(), 'monotonic': time.monotonic(),
            'pose_mm_axis_angle_rad': p, 'T_reference_tcp': pose_aa(p).tolist(),
            'joints_rad': checked(arm.get_servo_angle(is_radian=True)), 'state': state,
            'tcp_offset_mm_rad': list(arm.tcp_offset), 'world_offset_mm_rad': list(arm.world_offset)}

def stable(readings):
    ref = readings[0]
    worst_mm, worst_deg = 0., 0.
    for r in readings[1:]:
        for key in ('tcp_offset_mm_rad', 'world_offset_mm_rad'):
            if not np.allclose(ref[key], r[key], atol=1e-6, rtol=0):
                raise ValueError('Robot coordinate offsets changed; start a new session')
        mm, deg = delta(np.array(ref['T_reference_tcp']), np.array(r['T_reference_tcp']))
        worst_mm, worst_deg = max(worst_mm, mm), max(worst_deg, deg)
        if mm > 0.3 or deg > 0.15:
            raise ValueError(f'Robot not stationary: {mm:.3f}mm, {deg:.3f}deg')
    return {'max_motion_mm': worst_mm, 'max_motion_deg': worst_deg}

def capture_sample(arm, cam, board, initial, folder, on_preview=None):
    """Shared CLI/GUI capture; only reads hardware. Always leaves an audit record."""
    record = {'status': 'failed', 'robot_readings': []}
    try:
        readings = record['robot_readings']
        readings.append(read_robot(arm))
        for i in range(45):
            im, timing = cam.frame()
            if i % 5 == 0:
                readings.append(read_robot(arm))
                if on_preview:
                    on_preview(im)
        readings.append(read_robot(arm))
        record['frame'] = timing
        record['T_reference_tcp'] = readings[-1]['T_reference_tcp']
        if not cv2.imwrite(str(folder/'image.png'), im):
            raise IOError('Image save failed')
        record['stability'] = stable(readings)
        stable([initial, dict(readings[-1], T_reference_tcp=initial['T_reference_tcp'])])
        if readings[-1]['monotonic']-readings[-2]['monotonic'] > 2:
            raise ValueError('Frame/robot read interval too long')
        detected, preview = detect(im, board, cam.meta)
        record.update(detected)
        if not cv2.imwrite(str(folder/'preview.jpg'), preview):
            raise IOError('Preview save failed')
        record['status'] = 'accepted'
    except Exception as e:
        record['error'] = str(e)
    finally:
        save(folder/'sample.json', record)
    return record

def collect(a):
    c = load(a.config)
    if not c.get('confirmed_board_and_wrist_camera'):
        raise ValueError('First confirm measured board and wrist camera in config; set confirmed_board_and_wrist_camera=true')
    measured = a.square_mm
    if measured is None:
        measured = float(input('Measured printed square edge in mm (measure with ruler; design=35): '))
    if not np.isfinite(measured) or not 5 <= measured <= 100:
        raise ValueError('Measured square edge must be between 5 and 100 mm')
    c['board']['marker_mm'] *= measured / c['board']['square_mm']
    c['board']['square_mm'] = measured
    c['measured_square_mm'] = measured
    board = board_from(c)
    dest = pathlib.Path(a.session).resolve()
    dest.mkdir(parents=True, exist_ok=True)
    session_file = dest/'session.json'
    if session_file.exists() and load(session_file)['config'] != c:
        raise ValueError('Config changed; use a NEW session directory')
    from xarm.wrapper import XArmAPI
    cam = arm = None
    try:
        arm = XArmAPI(c['robot_ip'], is_radian=True)
        time.sleep(1)
        initial = read_robot(arm)
        cam = Camera(c['camera'])
        if session_file.exists():
            old = load(session_file)
            if old['camera'] != cam.meta:
                raise ValueError('Camera/intrinsics changed; use a new session')
            stable([old['initial_robot'], dict(initial, T_reference_tcp=old['initial_robot']['T_reference_tcp'])])
        else:
            save(session_file, {'config': c, 'camera': cam.meta, 'initial_robot': initial,
                               'opencv': cv2.__version__, 'units': 'meters', 'schema': 1})
        print('Manual motion only. Fix board and camera mount. Enter=capture, d=discard last, q=quit.', flush=True)
        latest = None
        while True:
            cmd = input('Move robot, let it settle, then Enter > ').strip().lower()
            if cmd == 'q':
                break
            if cmd == 'd':
                if latest:
                    record = load(latest/'sample.json')
                    record['status'] = 'discarded'
                    save(latest/'sample.json', record)
                    print('Discarded', latest.name)
                continue
            if cmd:
                continue
            number = max([int(p.name[7:]) for p in dest.glob('sample_*') if p.name[7:].isdigit()] or [0])+1
            folder = dest/f'sample_{number:04d}'
            folder.mkdir()
            latest = folder
            record = capture_sample(arm, cam, board, initial, folder)
            if record['status'] == 'accepted':
                print(f"Saved {folder.name}: {record['corners']} corners, RMS={record['reprojection_rms_px']:.3f}px", flush=True)
            else:
                print('Rejected:', record['error'], flush=True)
    finally:
        if cam:
            cam.close()
        if arm:
            arm.disconnect()

def diversity(G):
    rotations = [Rotation.from_matrix(A[:3, :3].T@B[:3, :3]).as_rotvec() for A, B in itertools.combinations(G, 2)]
    v = np.asarray([v for v in rotations if np.linalg.norm(v) > np.deg2rad(5)])
    if len(v) < 3:
        raise ValueError('Insufficient rotation; capture varied wrist orientations')
    s = np.linalg.svd(v, compute_uv=False)
    if s[1]/s[0] < 0.15:
        raise ValueError('Rotations nearly single-axis; tilt around another axis')
    return s.tolist()

def fit(G, C, method):
    diversity(G)
    R, t = cv2.calibrateHandEye([g[:3, :3] for g in G], [g[:3, 3] for g in G],
                               [c[:3, :3] for c in C], [c[:3, 3] for c in C], method=method)
    X = transform(R, t)
    if not np.isfinite(X).all() or not np.allclose(R.T@R, np.eye(3), atol=1e-5) or abs(np.linalg.det(R)-1)>1e-5:
        raise ValueError('Non-finite or invalid hand-eye solution')
    return X

def residual(G, C, X, reference):
    return np.array([delta(g@X@c, reference) for g, c in zip(G, C)])

def base_camera_pose(T_tcp_camera, reading, sample_name):
    """Convert reported TCP pose to physical base only with verified zero world offset.
    Nonzero/missing offsets fail closed instead of silently relabeling world as base.
    """
    offset = np.asarray(reading.get('world_offset_mm_rad', []), dtype=float)
    if offset.shape != (6,) or not np.isfinite(offset).all() or not np.allclose(offset, 0, atol=1e-9, rtol=0):
        raise ValueError('Cannot output physical base pose: world offset is nonzero or unknown')
    G = np.asarray(reading['T_reference_tcp'], dtype=float)
    X = np.asarray(T_tcp_camera, dtype=float)
    if G.shape != (4,4) or X.shape != (4,4) or not np.isfinite(G).all() or not np.isfinite(X).all():
        raise ValueError('Invalid pose matrix')
    T = G @ X
    return dict(T_base_camera=T.tolist(), camera_position_in_base_mm=(T[:3,3]*1000).tolist(),
                camera_quaternion_xyzw_in_base=Rotation.from_matrix(T[:3,:3]).as_quat().tolist(),
                base_pose_sample=sample_name, base_pose_utc=reading.get('utc'),
                base_pose_reference='last_accepted_capture',
                base_pose_note='Pose at the named capture, NOT a live pose. Camera moves with the robot.',
                T_base_tcp_at_capture=G.tolist())

def solve(a):
    dest = pathlib.Path(a.session).resolve()
    meta = load(dest/'session.json')
    board = board_from(meta['config'])
    G, C, names, rejected, rms, robot_records = [], [], [], [], [], []
    for p in sorted(dest.glob('sample_*/sample.json')):
        r = load(p)
        if r['status'] != 'accepted':
            rejected.append({'sample': p.parent.name, 'reason': r['status']})
            continue
        try:
            stable(r['robot_readings'])
            stable([meta['initial_robot'], dict(r['robot_readings'][-1], T_reference_tcp=meta['initial_robot']['T_reference_tcp'])])
            d, _ = detect(cv2.imread(str(p.parent/'image.png')), board, meta['camera'])
            G.append(np.array(r['T_reference_tcp'], float))
            C.append(np.array(d['T_camera_board'], float))
            names.append(p.parent.name)
            rms.append(d['reprojection_rms_px'])
            robot_records.append(r['robot_readings'][-1])
        except Exception as e:
            rejected.append({'sample': p.parent.name, 'reason': str(e)})
    if len(G) < 15:
        raise ValueError(f'Only {len(G)} valid samples; need at least 15, recommend 25-35')
    spectrum = diversity(G)
    # Deterministic held-out subset; never use it to fit or select the method.
    validation = list(range(3, len(G), 4))
    train = [i for i in range(len(G)) if i not in validation]
    gt, ct = [G[i] for i in train], [C[i] for i in train]
    methods = {'PARK': cv2.CALIB_HAND_EYE_PARK, 'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
               'TSAI': cv2.CALIB_HAND_EYE_TSAI}
    trials = []
    for name, method in methods.items():
        try:
            X = fit(gt, ct, method)
            Y = mean_pose([g@X@c for g, c in zip(gt, ct)])
            e = residual(gt, ct, X, Y)
            trials.append((float(np.mean(e[:, 0])+5*np.mean(e[:, 1])), name, X, Y))
        except (ValueError, cv2.error):
            pass
    if not trials:
        raise ValueError('All solvers failed; acquire more varied orientations')
    _, chosen, Xtrain, Ytrain = min(trials, key=lambda x: x[0])
    ev = residual([G[i] for i in validation], [C[i] for i in validation], Xtrain, Ytrain)
    X = fit(G, C, methods[chosen])
    Y = mean_pose([g@X@c for g, c in zip(G, C)])
    e = residual(G, C, X, Y)
    passed = bool(np.max(ev[:, 0]) <= 5 and np.max(ev[:, 1]) <= 2 and
                  np.max(e[:, 0]) <= 5 and np.max(e[:, 1]) <= 2)
    result = {'status': 'consistency_passed' if passed else 'needs_review',
              'warning': 'Consistency is not absolute accuracy. Verify on independent poses before use.',
              'method': chosen, 'sample_count': len(G), 'units': 'meters',
              'convention': 'T_A_B maps B coordinates into A; camera=RGB optical x-right y-down z-forward; tcp=active xArm TCP; reference=xArm reported world/base frame',
              'T_tcp_camera': X.tolist(), 'T_camera_tcp': np.linalg.inv(X).tolist(),
              'camera_position_in_tcp_mm': (X[:3, 3]*1000).tolist(),
              'camera_quaternion_xyzw_in_tcp': Rotation.from_matrix(X[:3, :3]).as_quat().tolist(),
              'T_reference_board': Y.tolist(), 'initial_robot': meta['initial_robot'],
              'camera': meta['camera'], 'board': meta['config']['board'],
              'rotation_singular_values': spectrum, 'rejected': rejected,
              'validation': {'samples': [names[i] for i in validation], 'errors_mm_deg': ev.tolist(),
                             'max_mm': float(ev[:, 0].max()), 'max_deg': float(ev[:, 1].max()),
                             'fit_excludes_validation': True},
              'samples': [{'name': n, 'reprojection_rms_px': px, 'board_error_mm': float(err[0]),
                           'board_error_deg': float(err[1]), 'T_reference_camera': (g@X).tolist()}
                          for n, px, err, g in zip(names, rms, e, G)]}
    # OpenCV's board frame starts at an outer corner. Expose a deliberately
    # named geometric-center frame for the base-localization workflow.
    T_reference_board_center = board_center_pose(Y, meta['config']['board'])
    T_board_center_reference = np.linalg.inv(T_reference_board_center)
    result.update(T_reference_board_center=T_reference_board_center.tolist(),
                  T_board_center_reference=T_board_center_reference.tolist(),
                  reference_origin_in_board_center_mm=(T_board_center_reference[:3, 3] * 1000).tolist(),
                  reference_quaternion_xyzw_in_board_center=Rotation.from_matrix(
                      T_board_center_reference[:3, :3]).as_quat().tolist(),
                  board_center_convention='+x across board, +y down board, origin at geometric center')
    try:
        result.update(base_camera_pose(X, robot_records[-1], names[-1]))
        result['output_frame'] = 'robot_base'
        result['T_base_board'] = Y.tolist()
        result['T_base_board_center'] = T_reference_board_center.tolist()
        result['T_board_center_base'] = T_board_center_reference.tolist()
        result['base_origin_in_board_center_mm'] = result['reference_origin_in_board_center_mm']
        result['base_quaternion_xyzw_in_board_center'] = result['reference_quaternion_xyzw_in_board_center']
        for item, reading in zip(result['samples'], robot_records):
            pose = base_camera_pose(X, reading, item['name'])
            item['T_base_camera'] = pose['T_base_camera']
            item['capture_utc'] = reading.get('utc')
    except ValueError as e:
        result['output_frame'] = 'base_unavailable'
        result['base_pose_error'] = str(e)
    # Timestamped output prevents a previous result from silently being overwritten.
    out = dest/('result_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    save(out, result)
    print('Result:', out)
    print('Status:', result['status'], '(not an absolute accuracy guarantee)')
    if result['output_frame'] == 'robot_base':
        print('Camera position in robot BASE [mm]:', result['camera_position_in_base_mm'])
        print('Camera quaternion xyzw in BASE:', result['camera_quaternion_xyzw_in_base'])
        print('Pose captured at:', result['base_pose_sample'], result['base_pose_utc'])
    else:
        print('BASE pose unavailable:', result['base_pose_error'])
    print('Held-out maximum error:', result['validation']['max_mm'], 'mm,', result['validation']['max_deg'], 'deg')
    if not passed:
        raise SystemExit(2)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    c = sub.add_parser('collect', help='Manual-trigger capture; no robot motion commands')
    c.add_argument('--config', required=True)
    c.add_argument('--session', required=True)
    c.add_argument('--square-mm', type=float, help='Measured printed square length; prompted if omitted')
    s = sub.add_parser('solve', help='Offline hand-eye fit and held-out validation')
    s.add_argument('--session', required=True)
    sub.add_parser('list-cameras', help='Enumerate RealSense identities without streaming')
    a = p.parse_args()
    if a.command == 'collect':
        collect(a)
    elif a.command == 'solve':
        solve(a)
    else:
        import pyrealsense2 as rs
        for d in rs.context().query_devices():
            print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number))

if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as e:
        print('ERROR:', e, file=sys.stderr)
        sys.exit(1)
