#!/usr/bin/env python3
"""Loopback-only guided UI; one worker owns all hardware access."""
import argparse
import copy
import ipaddress
import queue
import re
import socket
import threading
import time
import pathlib
import subprocess
import sys
import atexit
from flask import Flask, jsonify, request, send_file, Response
# Two ways in: `bash gui.sh` runs this file directly (plain module import), while tests and
# other callers import it as real2sim.calibrate.gui. Both must resolve to the SAME handeye
# object, or patching one would not affect the other.
if __package__:
    from . import handeye as h
else:
    import handeye as h

ROOT = pathlib.Path(__file__).resolve().parent

def explain(e):
    text = str(e)
    messages = {
        'connect socket failed': '机械臂连接失败：检查电源、网线及 IP 地址。相机预览仍可单独使用。',
        'Fewer than 10': '可见角点不足 10 个，请让更多标定板进入画面。',
        'Planar pose ambiguous': '当前视角太正或位姿不明确，请适当倾斜腕部相机。',
        'Robot not stationary': '拍摄期间机械臂发生移动，请停稳后重拍。',
        'Robot moving or stopped': '机械臂正在运动或处于错误停止状态，请先停稳并检查控制器。',
        'offsets changed': 'TCP 或参考坐标偏移已改变，请恢复配置或新建标定。',
        'Reprojection RMS': '图像拟合误差偏大，请检查清晰度、反光和标定板平整度。',
        'Insufficient rotation': '腕部旋转变化不足，请增加不同倾斜角度的照片。',
        'nearly single-axis': '主要只绕一个轴旋转，请补充绕另一个轴倾斜的照片。',
        'No device connected': '未找到指定的 RealSense，请检查序列号和 USB 连接。',
        "Couldn't resolve requests": '无法打开指定相机模式，请检查设备和是否被其他程序占用。',
        'failed to set power state': '相机可能被其他程序占用，请关闭该程序后重试。'
    }
    for key, value in messages.items():
        if key in text:
            return value + ' [' + text[:220] + ']'
    return text[:500]

class Engine:
    def __init__(self, root=ROOT, start_worker=True):
        self.root = pathlib.Path(root)
        self.config = h.load(self.root/'config.json')
        self.board = h.board_from(self.config)
        self.cam = self.arm = self.session = self.initial = None
        self.result_path = self.fixed_result_path = None
        self.camera_role = None
        self.board_anchor = None
        self.frame = None
        self.lock = threading.RLock()
        self.jobs = queue.Queue()
        self.running = True
        self.last_preview = self.last_robot = 0
        self.state = dict(busy=False, operation='', camera=False, robot=False, session=None,
                          samples=[], accepted=0, corners=0, rms=None, image_time=None,
                          detection='尚未开启相机', message='先开启相机，查看标定板是否完整入镜。',
                          error='', result=None, serial=self.config['camera']['serial'],
                          fixed_serial=self.config.get('fixed_camera',{}).get('serial','148522072685'),
                          camera_role=None, board_anchor=None, fixed_result=None,
                          robot_ip=self.config['robot_ip'], robot_pose=None)
        self.worker = None
        if start_worker:
            self.worker = threading.Thread(target=self.loop, daemon=True)
            self.worker.start()

    def update(self, **fields):
        with self.lock:
            self.state.update(fields)

    def snapshot(self):
        with self.lock:
            s = copy.deepcopy(self.state)
        s['image_age'] = None if s['image_time'] is None else max(0, time.time()-s['image_time'])
        return s

    def submit(self, action, payload):
        if action not in ('preview','fixed_preview','robot','session','capture','discard','solve',
                          'board_moved','anchor_board','fixed_capture','stop'):
            raise ValueError('未知操作')
        with self.lock:
            if self.state['busy']:
                raise ValueError('上一项操作尚未完成，请稍候。')
            self.state.update(busy=True, operation=action, error='')
        self.jobs.put((action, payload))

    def refresh_samples(self):
        rows = []
        if self.session:
            for p in sorted(self.session.glob('sample_*/sample.json')):
                r = h.load(p)
                rows.append(dict(name=p.parent.name, status=r['status'],
                                 rms=r.get('reprojection_rms_px'), error=explain(r.get('error',''))))
        self.update(samples=rows, accepted=sum(r['status']=='accepted' for r in rows))

    def invalidate_result(self):
        self.result_path = None
        self.fixed_result_path = None
        self.board_anchor = None
        self.update(result=None, board_anchor=None, fixed_result=None)

    def switch_camera(self, config, role):
        if self.cam:
            self.cam.close()
            self.cam = None
        self.camera_role = None
        self.update(camera=False, camera_role=None, image_time=None)
        with self.lock:
            self.frame = None
        self.cam = h.Camera(config)
        self.camera_role = role
        self.update(camera=True, camera_role=role)

    def render(self, im):
        now = time.time()
        preview = im.copy()
        corners = 0
        rms = None
        try:
            d, preview = h.detect(im, self.board, self.cam.meta)
            corners, rms = d['corners'], d['reprojection_rms_px']
            hint = ('标定板识别通过；保持标定板不动后拍摄。' if self.camera_role == 'fixed'
                    else '标定板识别通过；请保持机械臂静止后拍摄。')
        except Exception as e:
            hint = explain(e)
            cc, ci, _, _ = h.cv2.aruco.CharucoDetector(self.board).detectBoard(im)
            if ci is not None:
                corners = len(ci)
                h.cv2.aruco.drawDetectedCornersCharuco(preview, cc, ci)
        small = h.cv2.resize(preview, (960,540))
        ok, jpg = h.cv2.imencode('.jpg', small, [h.cv2.IMWRITE_JPEG_QUALITY,80])
        if not ok:
            raise ValueError('画面编码失败')
        with self.lock:
            self.frame = jpg.tobytes()
            self.state.update(image_time=now, corners=corners, rms=rms, detection=hint)

    def action_preview(self, p):
        if not self.cam or self.camera_role != 'wrist':
            self.switch_camera(self.config['camera'], 'wrist')
        if self.session and h.load(self.session/'session.json')['camera'] != self.cam.meta:
            self.cam.close()
            self.cam = None
            self.camera_role = None
            self.update(camera=False, camera_role=None, image_time=None)
            raise ValueError('相机参数发生变化，请结束当前标定并使用新名称。')
        self.update(camera=True, camera_role='wrist', message='腕部相机画面已开启。')

    def action_fixed_preview(self, p):
        serial = str(p.get('serial','')).strip()
        if not re.fullmatch(r'[A-Za-z0-9_-]{3,64}', serial):
            raise ValueError('请输入有效的固定相机序列号。')
        c = copy.deepcopy(self.config.get('fixed_camera', self.config['camera']))
        c['serial'] = serial
        self.switch_camera(c, 'fixed')
        self.update(fixed_serial=serial, message='固定相机画面已开启。请让标定板完整入镜并保持不动。')

    def action_robot(self, p):
        if self.session:
            raise ValueError('先结束当前标定，再重新连接机械臂。')
        ip = str(ipaddress.ip_address(p.get('robot_ip','')))
        if self.arm:
            self.arm.disconnect()
            self.arm = None
        self.update(robot=False,robot_pose=None)
        # Quick reachability check before constructing SDK (never sends motion).
        with socket.create_connection((ip,502), timeout=3):
            pass
        from xarm.wrapper import XArmAPI
        arm = XArmAPI(ip, is_radian=True)
        try:
            time.sleep(1)
            r = h.read_robot(arm)
        except Exception:
            arm.disconnect()
            raise
        self.arm = arm
        self.config['robot_ip'] = ip
        self.update(robot=True, robot_ip=ip, robot_pose=r['pose_mm_axis_angle_rad'],
                    message='设备已连接。填写实测尺寸并确认板已固定，然后开始标定。')

    def action_session(self, p):
        if not self.cam or self.camera_role != 'wrist' or not self.arm:
            raise ValueError('请先连接相机和机械臂。')
        if self.session:
            raise ValueError('已有标定正在进行，请先结束当前标定。')
        if p.get('fixed') is not True:
            raise ValueError('请确认标定板固定、相机安装牢固且尺寸已实测。')
        name = p.get('name','')
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', name):
            raise ValueError('标定名称只允许字母、数字、下划线和短横线。')
        measured = float(p.get('square_mm',0))
        if not h.np.isfinite(measured) or not 5 <= measured <= 100:
            raise ValueError('请输入实测单格长度（5–100 mm）。')
        c = copy.deepcopy(self.config)
        c['board']['marker_mm'] *= measured/c['board']['square_mm']
        c['board']['square_mm'] = c['measured_square_mm'] = measured
        board = h.board_from(c)
        base = (self.root/'sessions').resolve()
        dest = (base/name).resolve()
        if dest.parent != base:
            raise ValueError('无效的标定目录')
        r = h.read_robot(self.arm)
        meta_file = dest/'session.json'
        initial = r
        if meta_file.exists():
            old = h.load(meta_file)
            if old['config'] != c or old['camera'] != self.cam.meta:
                raise ValueError('旧标定尺寸、相机或设置不同，请换一个新名称。')
            initial = old['initial_robot']
            h.stable([initial, dict(r,T_reference_tcp=initial['T_reference_tcp'])])
        else:
            dest.mkdir(parents=True, exist_ok=True)
            if any(dest.iterdir()):
                raise ValueError('目录中已有其他数据，请使用新名称。')
            h.save(meta_file,dict(config=c,camera=self.cam.meta,initial_robot=r,
                                 opencv=h.cv2.__version__,units='meters',schema=1))
        self.session, self.initial, self.board = dest, initial, board
        self.invalidate_result()
        self.refresh_samples()
        self.update(session=name,message='已开始标定。手动移动腕部，从不同位置和角度拍摄。')

    def action_capture(self, p):
        if not self.session or not self.cam or not self.arm or self.camera_role != 'wrist':
            raise ValueError('设备和标定尚未准备好。')
        self.invalidate_result()
        n = max([int(x.name[7:]) for x in self.session.glob('sample_*') if x.name[7:].isdigit()] or [0])+1
        folder = self.session/f'sample_{n:04d}'
        folder.mkdir()
        self.update(message='正在采集并检查静止状态，请保持机械臂和标定板不动……')
        r = h.capture_sample(self.arm,self.cam,self.board,self.initial,folder,self.render)
        self.refresh_samples()
        if r['status'] != 'accepted':
            raise ValueError(r['error'])
        self.update(message=f"第 {n} 组已保存。现在可以移动机械臂，换一个角度。")

    def action_discard(self, p):
        if not self.session:
            raise ValueError('请先开始标定。')
        name = p.get('name','')
        if not re.fullmatch(r'sample_\d{4,}', name):
            raise ValueError('无效的样本编号')
        path = (self.session/name/'sample.json').resolve()
        if path.parent.parent != self.session.resolve():
            raise ValueError('无效的样本目录')
        r = h.load(path)
        r['status'] = 'discarded'
        h.save(path,r)
        self.invalidate_result()
        self.refresh_samples()
        self.update(message=name+' 已弃用，原图保留。请补拍一组。')

    def action_solve(self, p):
        if not self.session or self.snapshot()['accepted'] < 15:
            raise ValueError('至少需要 15 组有效样本，建议 25–35 组。')
        self.invalidate_result()
        self.update(message='正在重新检测原图、拟合手眼变换并进行留出验证……')
        before = set(self.session.glob('result_*.json'))
        proc = subprocess.run([sys.executable,'-B',str(self.root/'handeye.py'),'solve','--session',str(self.session)],
                              capture_output=True,text=True,timeout=180)
        fresh = set(self.session.glob('result_*.json'))-before
        if proc.returncode not in (0,2) or len(fresh) != 1:
            raise ValueError((proc.stderr or proc.stdout)[-1200:])
        self.result_path = fresh.pop()
        result = h.load(self.result_path)
        if result.get('output_frame') == 'robot_base' and result.get('T_base_board_center'):
            self.board_anchor = dict(T_base_board_center=result['T_base_board_center'],
                                     source='handeye_multi_view', utc=None)
        self.update(result=result, board_anchor=copy.deepcopy(self.board_anchor), fixed_result=None,
                    message='腕部手眼求解和板中心下的基座定位已完成。移动标定板后，必须用腕部相机重新定位。')

    def action_board_moved(self, p):
        if not self.session or not self.snapshot().get('result'):
            raise ValueError('请先完成腕部手眼求解。')
        self.board_anchor = None
        self.fixed_result_path = None
        self.update(board_anchor=None, fixed_result=None,
                    message='已将旧板位姿作废。固定好标定板后，用腕部相机重新定位当前板。')

    def action_anchor_board(self, p):
        result = self.snapshot().get('result')
        if not result or result.get('output_frame') != 'robot_base':
            raise ValueError('需要先获得机械臂物理基座下的腕部手眼结果。')
        if not self.cam or self.camera_role != 'wrist' or not self.arm:
            raise ValueError('请切换到腕部相机并保持机械臂连接。')
        self.update(message='正在拍摄当前标定板并检查机械臂静止状态……')
        readings = [h.read_robot(self.arm)]
        im = timing = None
        for i in range(30):
            im, timing = self.cam.frame()
            if i % 5 == 0:
                readings.append(h.read_robot(self.arm))
        readings.append(h.read_robot(self.arm))
        stability = h.stable(readings)
        h.stable([self.initial, dict(readings[-1], T_reference_tcp=self.initial['T_reference_tcp'])])
        detected, preview = h.detect(im, self.board, self.cam.meta)
        # Reuse the strict zero-world-offset guard before calling this frame base.
        camera_pose = h.base_camera_pose(result['T_tcp_camera'], readings[-1], 'board_anchor')
        T_base_camera = h.np.asarray(camera_pose['T_base_camera'], float)
        T_base_board_center = h.board_center_pose(
            T_base_camera @ h.np.asarray(detected['T_camera_board'], float),
            h.load(self.session/'session.json')['config']['board'])
        stamp = time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns() % 1000000:06d}'
        folder = self.session/f'board_anchor_{stamp}'
        folder.mkdir()
        if not h.cv2.imwrite(str(folder/'image.png'), im) or not h.cv2.imwrite(str(folder/'preview.jpg'), preview):
            raise IOError('定位图片保存失败')
        record = dict(T_base_board_center=T_base_board_center.tolist(), source='wrist_single_view',
                      utc=readings[-1].get('utc'), robot_reading=readings[-1], stability=stability,
                      detection=detected, frame=timing, image=str(folder.name+'/image.png'))
        h.save(folder/'anchor.json', record)
        self.board_anchor = record
        self.fixed_result_path = None
        self.update(board_anchor=copy.deepcopy(record), fixed_result=None,
                    message='当前标定板已在基座坐标系中重新定位。现在不要再移动板，切换固定相机拍摄。')

    def action_fixed_capture(self, p):
        if not self.session or not self.cam or self.camera_role != 'fixed':
            raise ValueError('请先开启固定相机。')
        if not self.board_anchor:
            raise ValueError('当前标定板尚未用腕部相机定位。')
        if self.board_anchor.get('source') != 'wrist_single_view':
            raise ValueError('固定相机标定前，必须用腕部相机重新定位移动后的标定板。')
        self.update(message='正在刷新固定相机画面并检测标定板……')
        im = timing = None
        for _ in range(30):
            im, timing = self.cam.frame()
        detected, preview = h.detect(im, self.board, self.cam.meta)
        board_cfg = h.load(self.session/'session.json')['config']['board']
        pose = h.fixed_camera_pose(self.board_anchor['T_base_board_center'],
                                   detected['T_camera_board'], board_cfg)
        stamp = time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns() % 1000000:06d}'
        folder = self.session/f'fixed_camera_{stamp}'
        folder.mkdir()
        if not h.cv2.imwrite(str(folder/'image.png'), im) or not h.cv2.imwrite(str(folder/'preview.jpg'), preview):
            raise IOError('固定相机图片保存失败')
        out = dict(status='completed', units='meters',
                   convention='T_A_B maps B coordinates into A; camera is RGB optical frame',
                   fixed_camera=self.cam.meta, board=board_cfg, board_anchor=self.board_anchor,
                   detection=detected, frame=timing, image=str(folder.name+'/image.png'), **pose)
        self.fixed_result_path = folder/'result.json'
        h.save(self.fixed_result_path, out)
        self.update(fixed_result=out,
                    message='固定相机外参标定完成。结果已保存，可下载 JSON。')

    def action_stop(self, p):
        try:
            if self.cam:
                self.cam.close()
        finally:
            self.cam = None
            self.camera_role = None
            if self.arm:
                self.arm.disconnect()
            self.arm = self.session = self.initial = None
            self.board = h.board_from(self.config)
            self.update(camera=False,camera_role=None,robot=False,session=None,image_time=None,robot_pose=None,
                        message='设备已释放，已保存的数据和结果保留。可重新开始或继续旧标定。')
            with self.lock:
                self.frame = None

    def loop(self):
        while self.running:
            try:
                action, payload = self.jobs.get(timeout=0.015)
            except queue.Empty:
                if self.cam:
                    try:
                        im, _ = self.cam.frame()
                        if time.monotonic()-self.last_preview > .3:
                            self.render(im)
                            self.last_preview = time.monotonic()
                    except Exception as e:
                        try:
                            self.cam.close()
                        except Exception:
                            pass
                        self.cam = None
                        self.camera_role = None
                        self.update(camera=False,camera_role=None,error=explain(e),image_time=None)
                if self.arm and time.monotonic()-self.last_robot > 1:
                    try:
                        r = h.read_robot(self.arm)
                        self.update(robot=True,robot_pose=r['pose_mm_axis_angle_rad'])
                    except Exception as e:
                        self.update(robot=False,robot_pose=None)
                    self.last_robot = time.monotonic()
                continue
            try:
                getattr(self,'action_'+action)(payload)
            except Exception as e:
                text = explain(e)
                if action == 'robot':
                    self.update(robot=False)
                    if isinstance(e,(OSError,TimeoutError)):
                        text = '机械臂连接失败：请检查电源、网线和 IP 地址。相机预览仍可使用。'
                self.update(error=text)
            finally:
                self.update(busy=False,operation='')
                self.jobs.task_done()

def make_app(engine):
    app = Flask(__name__)
    app.config['MAX_CONTENT_LENGTH'] = 8192

    @app.before_request
    def local_only():
        if request.host.split(':')[0] not in ('127.0.0.1','localhost'):
            return jsonify(error='仅允许本机或 SSH 转发访问'),403
        if request.method == 'POST' and (not request.is_json or request.headers.get('X-Handeye-UI') != '1'):
            return jsonify(error='请求来源无效'),403

    @app.after_request
    def headers(response):
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['X-Frame-Options'] = 'DENY'
        return response

    @app.get('/')
    def index():
        return send_file(ROOT/'gui.html')

    @app.get('/api/state')
    def state():
        return jsonify(engine.snapshot())

    @app.get('/frame.jpg')
    def frame():
        with engine.lock:
            data = engine.frame
        if not data:
            return '',204
        return Response(data,mimetype='image/jpeg')

    @app.post('/api/<action>')
    def action(action):
        try:
            p = request.get_json()
            if not isinstance(p,dict):
                raise ValueError('参数格式无效')
            engine.submit(action,p)
            return jsonify(ok=True),202
        except ValueError as e:
            return jsonify(error=str(e)),409

    @app.get('/result.json')
    def result():
        with engine.lock:
            p = engine.result_path
        if not p or not p.exists():
            return jsonify(error='尚无本次求解结果'),404
        return send_file(p,as_attachment=True,download_name=p.name)

    @app.get('/fixed-result.json')
    def fixed_result():
        with engine.lock:
            p = engine.fixed_result_path
        if not p or not p.exists():
            return jsonify(error='尚无固定相机标定结果'),404
        return send_file(p,as_attachment=True,download_name='fixed_camera_extrinsic.json')

    return app

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8765)
    args = parser.parse_args()
    # Prevent two GUI workers from competing for the same hardware/session.
    import fcntl
    lockfile = open(ROOT/'.gui.lock','w')
    try:
        fcntl.flock(lockfile,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('GUI 已在运行，请打开现有窗口。')
    engine = Engine()
    print(f'Open http://127.0.0.1:{args.port}',flush=True)
    make_app(engine).run(host='127.0.0.1',port=args.port,threaded=True,debug=False,use_reloader=False)
