import numpy as np
from ..contracts import rigid,profile_id,pixel_transform

def fit_extrinsics(camera,correspondences):
 import cv2
 if correspondences['profile_id']!=profile_id(camera):raise ValueError('Camera profile changed; annotations must be regenerated')
 if correspondences.get('coordinates')!='native_distorted_pixels':raise ValueError('PnP requires declared native sensor pixels')
 xyz=np.asarray(correspondences['world_points_m'],float);uv=np.asarray(correspondences['pixels_xy'],float)
 if xyz.ndim!=2 or xyz.shape[1]!=3 or uv.shape!=(len(xyz),2) or len(xyz)<6 or not np.isfinite(xyz).all() or not np.isfinite(uv).all():raise ValueError('Need at least six finite 2D/3D pairs')
 if np.linalg.matrix_rank(xyz-xyz.mean(0))<2:raise ValueError('Collinear points cannot calibrate camera')
 K=np.asarray(camera['K_native']);dist=np.asarray(camera['distortion']['coefficients'],float)
 ok,r,t,inliers=cv2.solvePnPRansac(xyz,uv,K,dist,iterationsCount=300,reprojectionError=3.,flags=cv2.SOLVEPNP_ITERATIVE)
 if not ok or inliers is None or len(inliers)<6:raise ValueError('PnP rejected')
 idx=inliers.ravel();r,t=cv2.solvePnPRefineLM(xyz[idx],uv[idx],K,dist,r,t)
 T=np.eye(4);T[:3,:3]=cv2.Rodrigues(r)[0];T[:3,3]=t.ravel();depth=(xyz@T[:3,:3].T+T[:3,3])[:,2]
 if np.any(depth<=0):raise ValueError('Calibration puts a correspondence behind camera')
 pred=cv2.projectPoints(xyz,r,t,K,dist)[0].reshape(-1,2);e=np.linalg.norm(pred-uv,axis=1)
 return rigid(np.linalg.inv(T)),{'inlier_count':len(idx),'point_count':len(xyz),'all_error_px':e.tolist(),'inlier_rms_px':float(np.sqrt(np.mean(e[idx]**2))),'planar':np.linalg.matrix_rank(xyz-xyz.mean(0))==2,'independent_validation':False,'quality':'image_fitted','warning':'Use independently measured holdout points and multiple views before declaring calibrated'}

def undistort(image,camera):
 import cv2
 K=np.asarray(camera['K_native']);d=np.asarray(camera['distortion']['coefficients'],float)
 return cv2.undistort(image,K,d,None,K)
