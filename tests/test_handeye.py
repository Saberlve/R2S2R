import argparse
import contextlib
import io
import json
import pathlib
import tempfile
import unittest
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from real2sim.calibrate import handeye as h

# The board/camera config belongs to the module, not to the test tree.
CONFIG = pathlib.Path(h.__file__).with_name('config.json')

def robot(T):
    return {'T_reference_tcp': T.tolist(), 'tcp_offset_mm_rad': [0,0,172,0,0,0],
            'world_offset_mm_rad': [0]*6}

class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.rng = np.random.default_rng(123)
        self.X = h.transform(R.from_euler('xyz', [0.2,-0.1,0.3]).as_matrix(), [0.04,-0.03,0.09])
        self.Y = h.transform(R.from_euler('xyz', [0.3,0.2,-0.1]).as_matrix(), [0.5,0.1,0.05])
        self.G = [h.transform(R.from_rotvec(self.rng.normal(0,0.5,3)).as_matrix(), self.rng.uniform(-0.4,0.4,3)) for _ in range(30)]
        self.C = [np.linalg.inv(g@self.X)@self.Y for g in self.G]

    def test_ground_truth_and_noise(self):
        X = h.fit(self.G, self.C, cv2.CALIB_HAND_EYE_PARK)
        self.assertTrue(np.allclose(X, self.X, atol=1e-8))
        noisy = [c@h.transform(R.from_rotvec(self.rng.normal(0,0.0005,3)).as_matrix(), self.rng.normal(0,0.0001,3)) for c in self.C]
        mm, deg = h.delta(h.fit(self.G, noisy, cv2.CALIB_HAND_EYE_PARK), self.X)
        self.assertLess(mm, 1)
        self.assertLess(deg, 0.2)

    def test_axis_angle_units(self):
        T = h.pose_aa([100,200,300,0,0,np.pi/2])
        self.assertTrue(np.allclose(T@[1,0,0,1], [0.1,1.2,0.3,1]))

    def test_board_center_and_fixed_camera_chain(self):
        board = {'squares_x':5, 'squares_y':7, 'square_mm':35}
        T_parent_board = h.transform(np.eye(3), [1, 2, 3])
        centered = h.board_center_pose(T_parent_board, board)
        self.assertTrue(np.allclose(centered[:3,3], [1.0875, 2.1225, 3]))
        T_base_fixed_truth = h.transform(R.from_euler('z', .4).as_matrix(), [.3,-.2,.8])
        T_base_center = h.transform(R.from_euler('xyz',[.1,.2,.3]).as_matrix(), [.5,.1,.2])
        T_fixed_center = np.linalg.inv(T_base_fixed_truth) @ T_base_center
        T_board_center = h.transform(np.eye(3), [.0875,.1225,0])
        T_fixed_board = T_fixed_center @ np.linalg.inv(T_board_center)
        result = h.fixed_camera_pose(T_base_center, T_fixed_board, board)
        self.assertTrue(np.allclose(result['T_base_fixed_camera'], T_base_fixed_truth))
        self.assertTrue(np.allclose(np.array(result['T_fixed_camera_base']) @ T_base_fixed_truth,
                                    np.eye(4)))

    def test_base_output_rotation_translation_and_offset_guard(self):
        G = h.pose_aa([100,200,300,0,0,np.pi/2])
        X = h.transform(np.eye(3), [1,0,0])
        r = dict(robot(G),utc='2026-09-14T08:00:00+00:00')
        out = h.base_camera_pose(X,r,'sample_0020')
        self.assertTrue(np.allclose(out['camera_position_in_base_mm'],[100,1200,300]))
        self.assertTrue(np.allclose(np.array(out['T_base_camera'])[:3,:3], G[:3,:3]))
        self.assertEqual(out['base_pose_sample'],'sample_0020')
        r['world_offset_mm_rad'][0]=10
        with self.assertRaises(ValueError):
            h.base_camera_pose(X,r,'sample_0020')
        del r['world_offset_mm_rad']
        with self.assertRaises(ValueError):
            h.base_camera_pose(X,r,'sample_0020')

    def test_degenerate_and_moving(self):
        G = [h.transform(R.from_rotvec([0,0,t]).as_matrix(), [t,0,0]) for t in np.linspace(0,1,20)]
        with self.assertRaises(ValueError):
            h.diversity(G)
        a, b = robot(np.eye(4)), robot(h.transform(np.eye(3), [0.001,0,0]))
        with self.assertRaises(ValueError):
            h.stable([a,b])
        b = robot(np.eye(4)); b['tcp_offset_mm_rad'][2] = 100
        with self.assertRaises(ValueError):
            h.stable([a,b])

    def test_rendered_images_and_full_solver(self):
        cfg = h.load(CONFIG)
        board = h.board_from(cfg)
        K = np.array([[1400.,0,960],[0,1400,540],[0,0,1]])
        camera = dict(cfg['camera'], K=K.tolist(), D=[0.]*5)
        pattern = board.generateImage((700,980))
        src = np.float32([[0,0],[699,0],[699,979],[0,979]])
        obj = np.float64([[0,0,0],[.175,0,0],[.175,.245,0],[0,.245,0]])
        valid = 0
        with tempfile.TemporaryDirectory() as td:
            root = pathlib.Path(td)
            h.save(root/'session.json', {'config':cfg,'camera':camera,'initial_robot':robot(self.G[0])})
            for i in range(32):
                rv = self.rng.uniform([-0.5,-0.5,-0.3],[0.5,0.5,0.3])
                if np.linalg.norm(rv[:2]) < 0.2:
                    rv[0] += 0.3
                t = np.array([-.085,-.12,self.rng.uniform(.5,.75)]) + self.rng.normal(0,.015,3)
                C = h.transform(R.from_rotvec(rv).as_matrix(), t)
                dst = cv2.projectPoints(obj, rv, t, K, np.zeros(5))[0].reshape(-1,2).astype(np.float32)
                im = cv2.warpPerspective(pattern, cv2.getPerspectiveTransform(src,dst), (1920,1080), borderValue=255)
                im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
                try:
                    detected, _ = h.detect(im,board,camera)
                except ValueError:
                    continue
                self.assertLess(h.delta(np.array(detected['T_camera_board']), C)[0], 3)
                G = self.Y@np.linalg.inv(C)@np.linalg.inv(self.X)
                folder = root/f'sample_{i:04d}'; folder.mkdir()
                cv2.imwrite(str(folder/'image.png'),im)
                h.save(folder/'sample.json', dict(status='accepted', T_reference_tcp=G.tolist(), robot_readings=[robot(G),robot(G)]))
                valid += 1
            self.assertGreaterEqual(valid, 20)
            with contextlib.redirect_stdout(io.StringIO()):
                h.solve(argparse.Namespace(session=str(root)))
            result = h.load(next(root.glob('result_*.json')))
            mm, deg = h.delta(np.array(result['T_tcp_camera']), self.X)
            self.assertLess(mm, 3)
            self.assertLess(deg, 0.5)
            self.assertEqual(result['status'],'consistency_passed')
            self.assertTrue(result['validation']['fit_excludes_validation'])
            self.assertEqual(result['output_frame'],'robot_base')
            self.assertEqual(result['base_pose_sample'],result['samples'][-1]['name'])
            self.assertLess(h.delta(np.array(result['T_base_camera']),G@self.X)[0],3)
            print('Rendered end-to-end:', valid, 'views;', round(mm,3), 'mm;', round(deg,3), 'deg')

if __name__ == '__main__':
    unittest.main(verbosity=2)
