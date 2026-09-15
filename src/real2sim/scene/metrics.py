import pathlib,numpy as np
from PIL import Image
from scipy.ndimage import binary_erosion

def score(real,sim,mask,lpips_cache=None):
 a=np.asarray(Image.open(real).convert('RGB'),float)/255;b=np.asarray(Image.open(sim).convert('RGB'),float)/255
 m=np.asarray(Image.open(mask).convert('L'))>127
 if a.shape!=b.shape or m.shape!=a.shape[:2] or not m.any():raise ValueError('Images/mask must match exactly; no implicit resize and no empty mask')
 from skimage.metrics import structural_similarity
 ssim_map=structural_similarity(a,b,data_range=1.,channel_axis=2,full=True,win_size=7)[1].mean(2)
 interior=binary_erosion(m,structure=np.ones((7,7)))
 if not interior.any():raise ValueError('Mask too small for valid SSIM windows')
 mae=float(np.abs(a[m]-b[m]).mean());ssim=float(ssim_map[interior].mean())
 out={'mae':mae,'ssim':ssim,'lpips':None,'loss':.5*mae+.5*(1-ssim),'loss_definition':'0.5*MAE+0.5*(1-SSIM)','mask_pixels':int(m.sum()),'colorspace':'display RGB in [0,1]; not linear radiance'}
 if lpips_cache:
  import os,torch,lpips
  os.environ['TORCH_HOME']=str(pathlib.Path(lpips_cache).resolve())
  old=torch.hub.download_url_to_file
  def deny(*args,**kw):raise RuntimeError('LPIPS weights not cached; automatic network download disabled')
  torch.hub.download_url_to_file=deny
  try:model=lpips.LPIPS(net='alex',spatial=True).eval()
  finally:torch.hub.download_url_to_file=old
  # Common masked background; report explicitly because this is not whole-image LPIPS.
  aa=np.where(m[:,:,None],a,.5);bb=np.where(m[:,:,None],b,.5)
  with torch.no_grad():
   value=model(torch.tensor(aa.transpose(2,0,1)[None]*2-1,dtype=torch.float32),torch.tensor(bb.transpose(2,0,1)[None]*2-1,dtype=torch.float32))[0,0].numpy()
  if value.shape!=m.shape:raise ValueError('Unexpected LPIPS spatial output')
  out['lpips']=float(value[interior].mean());out['lpips_protocol']='alex spatial, common gray outside mask, valid 7px interior; receptive fields can still cross boundary'
  out['loss']=.5*mae+.25*(1-ssim)+.25*out['lpips'];out['loss_definition']='0.5*MAE+0.25*(1-SSIM)+0.25*LPIPS'
 return out
