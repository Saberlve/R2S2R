#!/usr/bin/env python3
"""Independent held-out corner prediction. Does not modify GUI or input files."""
import argparse
import hashlib
import html
import io
import csv
import zipfile
import pathlib
import datetime
import numpy as np
import cv2
# Runs both as a script and as real2sim.calibrate.validate_overlay; see the note in gui.py.
if __package__:
    from . import handeye as h
else:
    import handeye as h

def predict(G, X, Y, obj, K, D):
    C = np.linalg.inv(G @ X) @ Y
    points = C[:3,:3] @ obj.T + C[:3,3:4]
    if np.any(points[2] <= 0):
        raise ValueError('Predicted corners behind camera')
    pixels = cv2.projectPoints(obj,cv2.Rodrigues(C[:3,:3])[0],C[:3,3],K,D)[0].reshape(-1,2)
    return pixels, C

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session',required=True)
    ap.add_argument('--output',type=pathlib.Path)
    a=ap.parse_args()
    session=pathlib.Path(a.session).resolve()
    output=a.output or session/('overlay_validation_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True,exist_ok=False)
    hashes={}
    def read(p):
        data=p.read_bytes();hashes[str(p)]=hashlib.sha256(data).hexdigest();return data
    meta=h.json.loads(read(session/'session.json'))
    board=h.board_from(meta['config']); K,D=h.intrinsics(meta['camera'])
    samples=[]; excluded=[]
    for p in sorted(session.glob('sample_*/sample.json')):
        r=h.json.loads(read(p))
        if r['status']!='accepted':
            excluded.append(dict(name=p.parent.name,reason=r['status']));continue
        try:
            h.stable(r['robot_readings'])
            h.stable([meta['initial_robot'],dict(r['robot_readings'][-1],T_reference_tcp=meta['initial_robot']['T_reference_tcp'])])
            im=cv2.imdecode(np.frombuffer(read(p.parent/'image.png'),np.uint8),cv2.IMREAD_COLOR)
            d,_=h.detect(im,board,meta['camera'])
            samples.append(dict(name=p.parent.name,G=np.array(r['T_reference_tcp']),C=np.array(d['T_camera_board']),
                                d=d,image=im,utc=r['robot_readings'][-1].get('utc')))
        except Exception as e:excluded.append(dict(name=p.parent.name,reason=str(e)))
    if len(samples)<15:raise ValueError(f'Only {len(samples)} valid samples')
    vi=list(range(3,len(samples),4)); ti=[i for i in range(len(samples)) if i not in vi]
    gt=[samples[i]['G'] for i in ti];ct=[samples[i]['C'] for i in ti]
    trials=[];diagnostics=[]
    for name,method in [('PARK',cv2.CALIB_HAND_EYE_PARK),('HORAUD',cv2.CALIB_HAND_EYE_HORAUD),('TSAI',cv2.CALIB_HAND_EYE_TSAI)]:
        try:
            X=h.fit(gt,ct,method);Y=h.mean_pose([g@X@c for g,c in zip(gt,ct)])
            e=h.residual(gt,ct,X,Y);score=float(e[:,0].mean()+5*e[:,1].mean())
            trials.append((score,name,X,Y));diagnostics.append(dict(method=name,training_score=score))
        except Exception as e:diagnostics.append(dict(method=name,error=str(e)))
    if not trials:raise ValueError('All training solvers failed')
    score,method,X,Y=min(trials,key=lambda x:x[0])
    rows=[]; all_errors=[]; thumbs=[]
    for i in vi:
        s=samples[i]; d=s['d'];obj=board.getChessboardCorners()[d['ids']].astype(float)
        obs=np.array(d['corners_px']);pred,Cpred=predict(s['G'],X,Y,obj,K,D)
        vectors=pred-obs;errs=np.linalg.norm(vectors,axis=1);all_errors.extend(errs.tolist())
        mm,deg=h.delta(s['G']@X@s['C'],Y)
        row=dict(name=s['name'],corners=len(obs),prediction_rms_px=float(np.sqrt(np.mean(errs**2))),
                 prediction_mean_px=float(errs.mean()),prediction_max_px=float(errs.max()),
                 own_pnp_rms_px=d['reprojection_rms_px'],board_consistency_mm=mm,board_consistency_deg=deg,
                 predicted_board_origin_z_m=float(Cpred[2,3]),utc=s['utc'],
                 detected_px=obs.tolist(),predicted_px=pred.tolist(),corner_ids=d['ids'],
                 error_vectors_px=vectors.tolist(),T_camera_board_predicted=Cpred.tolist())
        rows.append(row)
        im=s['image'].copy();height,width=im.shape[:2]
        for actual,expected in zip(obs,pred):
            pt=tuple(np.rint(actual).astype(int)); qt=tuple(np.rint(expected).astype(int))
            if np.max(np.abs(expected))>100000:continue
            cv2.line(im,pt,qt,(0,210,255),2,cv2.LINE_AA)
            cv2.circle(im,pt,5,(40,220,30),2,cv2.LINE_AA)
            cv2.drawMarker(im,qt,(30,30,245),cv2.MARKER_CROSS,13,2,cv2.LINE_AA)
        cv2.rectangle(im,(0,0),(width,88),(25,32,30),-1)
        cv2.putText(im,f"{s['name']}  HOLDOUT   predicted RMS {row['prediction_rms_px']:.2f}px / max {errs.max():.2f}px",(18,33),cv2.FONT_HERSHEY_SIMPLEX,.78,(255,255,255),2)
        cv2.putText(im,'GREEN circle: observed | RED cross: predicted from TRAINING ONLY | YELLOW: residual (true scale)',(18,68),cv2.FONT_HERSHEY_SIMPLEX,.62,(230,230,230),1)
        if not cv2.imwrite(str(output/(s['name']+'_overlay.jpg')),im):raise IOError('Image write failed')
        # Board crop enlarges the residuals and symbols together, without changing their relative scale.
        box=np.vstack([obs,pred]);lo=np.maximum(np.floor(box.min(0)-55).astype(int),[0,88]);hi=np.minimum(np.ceil(box.max(0)+55).astype(int),[width,height])
        if np.all(hi>lo):
            crop=im[lo[1]:hi[1],lo[0]:hi[0]]
            cv2.imwrite(str(output/(s['name']+'_detail.jpg')),crop)
        thumb=cv2.resize(im,(768,432));thumbs.append(thumb)
    canvas=np.full((((len(thumbs)+1)//2)*432,1536,3),240,np.uint8)
    for k,t in enumerate(thumbs):canvas[(k//2)*432:(k//2+1)*432,(k%2)*768:(k%2+1)*768]=t
    cv2.imwrite(str(output/'overview.jpg'),canvas)
    for path,digest in hashes.items():
        if hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Input changed during analysis; rerun: '+path)
    all_errors=np.array(all_errors)
    summary=dict(session=str(session),valid_count=len(samples),training=[samples[i]['name'] for i in ti],
                 validation=[samples[i]['name'] for i in vi],excluded=excluded,method=method,methods=diagnostics,
                 T_tcp_camera_training=X.tolist(),T_reference_board_training=Y.tolist(),
                 prediction_formula='T_camera_board = inv(T_reference_tcp @ T_tcp_camera_training) @ T_reference_board_training',
                 note='Validation PnP used only for original quality screening and diagnostic comparison, NEVER to adjust prediction or select method. No full-data refit.',
                 pooled_rms_px=float(np.sqrt(np.mean(all_errors**2))),pooled_median_px=float(np.median(all_errors)),
                 pooled_p95_px=float(np.percentile(all_errors,95)),max_px=float(all_errors.max()),
                 validation_rows=rows,input_sha256=hashes,opencv=cv2.__version__)
    h.save(output/'validation.json',summary)
    fields=['name','corners','prediction_rms_px','prediction_mean_px','prediction_max_px','own_pnp_rms_px','board_consistency_mm','board_consistency_deg','predicted_board_origin_z_m','utc']
    with (output/'summary.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    ordered=sorted(rows,key=lambda r:r['prediction_rms_px'],reverse=True)
    table=''.join(f"<tr><td><a href='#{r['name']}'>{r['name']}</a></td><td>{r['corners']}</td><td>{r['prediction_rms_px']:.2f}</td><td>{r['prediction_max_px']:.2f}</td><td>{r['own_pnp_rms_px']:.3f}</td><td>{r['board_consistency_mm']:.2f}</td><td>{r['board_consistency_deg']:.2f}</td></tr>" for r in ordered)
    panels=''.join(f"<section id='{r['name']}'><h2>{r['name']} · 预测 RMS {r['prediction_rms_px']:.2f} px</h2><a href='{r['name']}_overlay.jpg'><img src='{r['name']}_overlay.jpg'></a><details><summary>角点区域放大查看</summary><img src='{r['name']}_detail.jpg'></details></section>" for r in ordered)
    report=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>验证组角点预测报告</title><style>body{{font:15px/1.7 Arial,"Microsoft YaHei",sans-serif;color:#233a32;max-width:1100px;margin:30px auto;padding:20px;background:#f4f7f5}}section,header{{background:white;padding:22px;border-radius:12px;margin-bottom:20px}}img{{width:100%}}table{{width:100%;border-collapse:collapse;background:white}}td,th{{padding:9px;border-bottom:1px solid #ddd;text-align:left}}a{{color:#167763}}.green{{color:#15852b}}.red{{color:#d22}}code{{overflow-wrap:anywhere}}</style><header><h1>验证组角点预测报告</h1><p>数据：{html.escape(session.name)} · 有效 {len(samples)} 组 · 拟合 {len(ti)} 组 · 验证 {len(vi)} 组 · 方法 {method}</p><p><b class="green">绿色圆圈：实际角点</b> ／ <b class="red">红色十字：拟合组预测角点</b> ／ 黄色连线：真实像素偏差（未人为放大）。</p><p>验证组角点整体 RMS：<b>{summary['pooled_rms_px']:.2f} px</b>；中位数 {summary['pooled_median_px']:.2f} px；95 分位 {summary['pooled_p95_px']:.2f} px。</p><p>只使用拟合组建立手眼变换和固定板位姿；每张验证图只提供机械臂位姿给预测。验证图自身检测与 PnP 仅用于筛选和误差比较，不修正预测，不参与方法选择；没有最后的全量重拟合。</p><p>此处预测误差包含手眼标定、机械臂、固定板及成像模型的共同影响，不等于 GUI 的单图 PnP 重投影误差，不能沿用 1 px 阈值直接判定采集失败。偏差不能单独证明采集操作有误。</p><p>拟合组：<code>{', '.join(summary['training'])}</code><br>验证组：<code>{', '.join(summary['validation'])}</code></p><p><a href="summary.csv">CSV 汇总</a> · <a href="validation.json">完整计算与输入哈希</a> · <a href="overview.jpg">叠加图总览</a></p></header><table><tr><th>验证照片</th><th>角点数</th><th>预测 RMS / px</th><th>最大 / px</th><th>单图 PnP / px</th><th>板偏差 / mm</th><th>板偏差 / °</th></tr>{table}</table>{panels}</html>'''
    (output/'report.html').write_text(report,encoding='utf-8')
    shutil_source=pathlib.Path(__file__)
    (output/'validate_overlay.py').write_bytes(shutil_source.read_bytes())
    archive=output.with_suffix('.zip')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for p in output.iterdir():z.write(p,output.name+'/'+p.name)
    print(h.json.dumps(dict(output=str(output),archive=str(archive),training=len(ti),validation=len(vi),
                           method=method,pooled_rms_px=summary['pooled_rms_px'],rows=[{k:r[k] for k in fields} for r in ordered]),indent=2))

if __name__=='__main__':main()
