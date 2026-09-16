import importlib.util
from pathlib import Path
import numpy as np
import pytest
spec = importlib.util.spec_from_file_location('match', Path(__file__).parents[1]/'tools/match_camera_profile.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
def profile(w=12,h=8):
    return dict(width=w,height=h,K=[[12,0,6],[0,12,4],[0,0,1]],D=[0]*5,serial='test')
def test_identity_preserves_image():
    p=profile(); image=np.arange(8*12*3,dtype=np.uint8).reshape(8,12,3)
    assert np.array_equal(m.convert(image,p,p),image)
def test_projected_rays_and_crop():
    s=profile();t=profile(4,4);t['K']=[[6,0,2],[0,6,2],[0,0,1]]
    x,y,H=m.mapping(s,t)
    ray=np.array([.1,-.1,1]);src=np.array(s['K'])@ray;dst=np.array(t['K'])@ray
    assert np.allclose(H@dst,src)
    assert x[0,0]==2 and y[0,0]==0
    assert np.allclose(H,[[2,0,2],[0,2,0],[0,0,1]])
def test_rejects_invalid_inputs():
    p=profile()
    with pytest.raises(ValueError):m.convert(np.zeros((7,12,3),np.uint8),p,p)
    bad=profile();bad['D']=[.1]
    with pytest.raises(ValueError):m.mapping(p,bad)
    wide=profile();wide['K']=[[1,0,6],[0,1,4],[0,0,1]]
    with pytest.raises(ValueError):m.mapping(p,wide)
