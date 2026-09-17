import tempfile
import pathlib
import shutil
import unittest
from unittest.mock import patch
import numpy as np
from real2sim.calibrate import gui
from real2sim.calibrate import handeye as h

class FakeCamera:
    def __init__(self, config):
        self.meta = dict(config,K=[[1400,0,960],[0,1400,540],[0,0,1]],D=[0]*5)
        self.count=0
    def frame(self):
        self.count+=1
        return np.full((1080,1920,3),200,np.uint8), {'frame_number':self.count}
    def close(self):
        pass

def reading():
    return dict(T_reference_tcp=np.eye(4).tolist(),tcp_offset_mm_rad=[0,0,172,0,0,0],
                world_offset_mm_rad=[0]*6,monotonic=h.time.monotonic())

class GuiTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.root=pathlib.Path(self.tmp.name)
        shutil.copy(pathlib.Path(gui.__file__).with_name('config.json'), self.root/'config.json')
        self.e=gui.Engine(self.root,start_worker=False)
        self.client=gui.make_app(self.e).test_client()
    def tearDown(self):
        self.tmp.cleanup()
    def test_http_guards_and_serialization(self):
        with self.client.get('/') as response:
            self.assertEqual(response.status_code,200)
        self.assertEqual(self.client.get('/frame.jpg').status_code,204)
        self.assertEqual(self.client.get('/api/state').json['accepted'],0)
        self.assertEqual(self.client.post('/api/capture',json={}).status_code,403)
        self.assertEqual(self.client.get('/api/state',headers={'Host':'evil.example'}).status_code,403)
        headers={'X-Handeye-UI':'1'}
        self.assertEqual(self.client.post('/api/preview',json={},headers=headers).status_code,202)
        self.assertEqual(self.client.post('/api/preview',json={},headers=headers).status_code,409)
        self.assertEqual(self.client.get('/result.json').status_code,404)
    def test_camera_without_robot_and_failed_connection(self):
        with patch.object(h,'Camera',FakeCamera):
            self.e.action_preview({})
        self.assertTrue(self.e.snapshot()['camera'])
        self.assertFalse(self.e.snapshot()['robot'])
        with self.assertRaises(ValueError):
            self.e.action_session(dict(name='x',square_mm=35,fixed=True))
        with patch.object(gui.socket,'create_connection',side_effect=TimeoutError('timeout')):
            with self.assertRaises(TimeoutError):
                self.e.action_robot({'robot_ip':'192.168.1.245'})
        self.assertIsNotNone(self.e.cam)
    def test_capture_resume_reject_discard_and_invalidation(self):
        self.e.cam=FakeCamera(self.e.config['camera']); self.e.arm=object(); self.e.camera_role='wrist'
        self.e.update(camera=True,camera_role='wrist',robot=True)
        with patch.object(h,'read_robot',side_effect=lambda _:reading()):
            for name, mm in [('../escape',35),('valid',float('nan'))]:
                with self.assertRaises(ValueError):
                    self.e.action_session(dict(name=name,square_mm=mm,fixed=True))
            self.e.action_session(dict(name='valid',square_mm=35,fixed=True))
            detected=dict(T_camera_board=np.eye(4).tolist(),corners=24,ids=list(range(24)),corners_px=[],reprojection_rms_px=.1)
            with patch.object(h,'detect',side_effect=lambda im,b,c:(detected,im)):
                self.e.action_capture({})
            self.assertEqual(self.e.cam.count,45)
            self.assertEqual(self.e.snapshot()['accepted'],1)
            self.assertTrue((self.e.session/'sample_0001/image.png').exists())
            self.e.result_path=self.root/'stale.json';self.e.update(result={'stale':True})
            self.e.action_discard({'name':'sample_0001'})
            self.assertEqual(self.e.snapshot()['accepted'],0)
            self.assertIsNone(self.e.result_path)
            self.assertTrue((self.e.session/'sample_0001/image.png').exists())
            with self.assertRaises(ValueError):
                self.e.action_discard({'name':'../outside'})
            with patch.object(h,'capture_sample',return_value={'status':'failed','error':'Robot not stationary'}):
                with self.assertRaises(ValueError):
                    self.e.action_capture({})
            with self.assertRaises(ValueError):
                self.e.action_solve({})
            self.e.session=None
            self.e.action_session(dict(name='valid',square_mm=35,fixed=True))
            self.assertEqual(self.e.snapshot()['accepted'],0)
            self.e.session=None
            with self.assertRaises(ValueError):
                self.e.action_session(dict(name='valid',square_mm=34,fixed=True))

    def test_moved_board_anchor_and_single_fixed_camera_capture(self):
        session = self.root/'sessions'/'workflow'
        session.mkdir(parents=True)
        h.save(session/'session.json', {'config':self.e.config,
                                        'camera':FakeCamera(self.e.config['camera']).meta,
                                        'initial_robot':reading()})
        self.e.session=session; self.e.initial=reading(); self.e.arm=object()
        self.e.cam=FakeCamera(self.e.config['camera']); self.e.camera_role='wrist'
        result={'output_frame':'robot_base','T_tcp_camera':np.eye(4).tolist()}
        self.e.update(session='workflow',camera=True,camera_role='wrist',robot=True,result=result)
        detected=dict(T_camera_board=np.eye(4).tolist(),corners=24,ids=list(range(24)),
                      corners_px=[],reprojection_rms_px=.1)
        with patch.object(h,'read_robot',side_effect=lambda _:reading()), \
             patch.object(h,'detect',side_effect=lambda im,b,c:(detected,im)):
            self.e.action_anchor_board({})
            self.assertEqual(self.e.board_anchor['source'],'wrist_single_view')
            self.e.cam=FakeCamera(self.e.config['camera']); self.e.camera_role='fixed'
            self.e.update(camera_role='fixed')
            self.e.action_fixed_capture({})
        fixed=self.e.snapshot()['fixed_result']
        self.assertTrue(np.allclose(fixed['T_base_fixed_camera'],np.eye(4)))
        self.assertTrue(self.e.fixed_result_path.exists())
        with self.client.get('/fixed-result.json') as response:
            self.assertEqual(response.status_code,200)

if __name__=='__main__':
    unittest.main(verbosity=2)
