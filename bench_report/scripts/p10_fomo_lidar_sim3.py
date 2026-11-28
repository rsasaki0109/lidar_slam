# Reference-free: Sim(3) point-to-plane ICP of Leishen points onto RoboSense planes while the robot stands still.
# The rigid part absorbs the LiDAR-to-LiDAR extrinsic error; s is the RoboSense/Leishen range ratio (s > 1: the
# RoboSense measures longer). Uses no gt.txt.
import sys, json, urllib.request, re, numpy as np
from scipy.spatial import cKDTree
B='https://fomo-dataset.s3.us-west-2.amazonaws.com'
drive=sys.argv[1]; nfr=int(sys.argv[2]); skip=int(sys.argv[3]) if len(sys.argv)>3 else 0
dt=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('i','<f4'),('ring','<u2'),('t','<i8')])
get=lambda k: urllib.request.urlopen(f'{B}/{k}',timeout=120).read()
keys=lambda p,n: re.findall(r'<Key>([^<]+)</Key>',get(f'?list-type=2&prefix={p}&max-keys={n}').decode())
def load(k): b=get(k); p=np.frombuffer(b[:len(b)-len(b)%26],dt); return np.stack([p['x'],p['y'],p['z']],1).astype(float)
def rot(q):
    x,y,z,w=q; return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def expso3(w):
    th=np.linalg.norm(w); K=np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
    return np.eye(3)+K if th<1e-12 else np.eye(3)+np.sin(th)/th*K+(1-np.cos(th))/th**2*K@K
tf=json.loads(get(f'data/{drive}/calib/transforms.json')); e=[t for t in tf if t['from']=='robosense' and t['to']=='leishen'][0]
R=rot([e['orientation'][k] for k in 'xyzw']); t=np.array([e['position'][k] for k in 'xyz'])
rk=keys(f'data/{drive}/robosense/',skip+nfr)[skip:]; lk=keys(f'data/{drive}/leishen/',skip+nfr)[skip:]
rs=np.vstack([load(k) for k in rk]); ls=np.vstack([load(k) for k in lk])
rs=rs[np.linalg.norm(rs,axis=1)>2]; ls=ls[(np.linalg.norm(ls,axis=1)>2)&(np.linalg.norm(ls,axis=1)<60)]
tree=cKDTree(rs); d,idx=tree.query(rs,k=10); nb=rs[idx]; c=nb.mean(1)
C=np.einsum('nki,nkj->nij',nb-c[:,None],nb-c[:,None])/10; ev,evec=np.linalg.eigh(C); N=evec[:,:,0]
plan=(ev[:,0]<0.015**2)&(ev[:,1]>9*ev[:,0])
s=1.0
for it in range(30):
    P=ls@R.T+t           # Leishen in the RoboSense frame (rigid)
    dd,j=tree.query(P,distance_upper_bound=0.25); ok=np.isfinite(dd); ok[ok]&=plan[j[ok]]
    p,q,n=P[ok],rs[j[ok]],N[j[ok]]
    # model: the RoboSense point scaled to s*q; residual r = n.(p - s q); unknowns dw, dt, ds (p -> p + dw x p + dt)
    r=np.einsum('ni,ni->n',n,p-s*q)
    w=1/(1+(r/0.03)**2)  # Cauchy
    J=np.hstack([np.cross(p,n),n,-np.einsum('ni,ni->n',n,q)[:,None]])
    H=(J*w[:,None]).T@J; g=(J*w[:,None]).T@r; x=-np.linalg.solve(H,g)
    Rd=expso3(x[:3]); R=Rd@R; t=Rd@t+x[3:6]; s+=x[6]
    if np.abs(x).max()<1e-7: break
cov=np.linalg.inv(H)*np.sum(w*r*r)/max(1,len(r)-7)
print(f'{drive} frames {skip}-{skip+nfr}: pairs {ok.sum()}, s = {s:.5f} +- {np.sqrt(cov[6,6]):.5f}, extrinsic change '
      f'{np.round(t-np.array([e["position"][k] for k in "xyz"]),3)} m, residual rms {np.sqrt(np.mean(r*r)):.4f}')
