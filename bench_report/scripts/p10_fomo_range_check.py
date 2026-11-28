# Compare RoboSense and Leishen geometry while the robot stands still: signed point-to-plane distance of Leishen points
# to RoboSense surfaces, regressed on r_RS * (n . u). Slope = -(k - 1) if RoboSense ranges are k x too long.
import sys, json, urllib.request, re, numpy as np
from scipy.spatial import cKDTree
B='https://fomo-dataset.s3.us-west-2.amazonaws.com'
drive=sys.argv[1]; nframes=int(sys.argv[2]) if len(sys.argv)>2 else 5
dt=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('i','<f4'),('ring','<u2'),('t','<i8')])
def get(k): return urllib.request.urlopen(f'{B}/{k}',timeout=120).read()
def keys(prefix,n):
    x=get(f'?list-type=2&prefix={prefix}&max-keys={n}').decode(); return re.findall(r'<Key>([^<]+)</Key>',x)
def load(k): b=get(k); p=np.frombuffer(b[:len(b)-len(b)%26],dt); return np.stack([p['x'],p['y'],p['z']],1).astype(float)
tf=json.loads(get(f'data/{drive}/calib/transforms.json'))
e=[t for t in tf if t['from']=='robosense' and t['to']=='leishen'][0]
x,y,z,w=[e['orientation'][k] for k in 'xyzw']
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tv=np.array([e['position'][k] for k in 'xyz'])
rs=np.vstack([load(k) for k in keys(f'data/{drive}/robosense/',nframes)])
ls=np.vstack([load(k) for k in keys(f'data/{drive}/leishen/',nframes)])
ls=ls[np.linalg.norm(ls,axis=1)>1.0]; rs=rs[np.linalg.norm(rs,axis=1)>1.0]
L=ls@R.T+tv  # Leishen points in the RoboSense frame (T_robosense_leishen)
tree=cKDTree(rs); d,idx=tree.query(L,k=12,distance_upper_bound=0.3)
ok=np.isfinite(d).all(1); L=L[ok]; idx=idx[ok]
nb=rs[idx]; c=nb.mean(1); C=np.einsum('nki,nkj->nij',nb-c[:,None],nb-c[:,None])/12
ev,evec=np.linalg.eigh(C); n=evec[:,:,0]; planar=(ev[:,0]<0.02**2)&(ev[:,1]>4*ev[:,0])
r=np.linalg.norm(c,axis=1); u=c/r[:,None]; nu=np.einsum('ni,ni->n',n,u)
s=np.sign(nu); n=n*s[:,None]; nu=np.abs(nu)
res=np.einsum('ni,ni->n',L-c,n)   # >0: Leishen point beyond the RoboSense surface along the ray
m=planar&(nu>0.5)&(np.abs(res)<0.2)
X=r[m]*nu[m]; Y=res[m]
A=np.stack([X,np.ones_like(X)],1); slope,off=np.linalg.lstsq(A,Y,rcond=None)[0]
print(f'{drive}: pairs {m.sum()} of {len(L)}; residual = {slope:+.5f} * r(n.u) {off:+.4f} m  => RoboSense/Leishen range ratio ~ {1-slope:.5f}')
for lo,hi in [(1,5),(5,10),(10,20),(20,40),(40,100)]:
    q=m&(r>=lo)&(r<hi)
    if q.sum()>50: print(f'  r {lo:>3}-{hi:<3} m: n {q.sum():6d}  median residual {np.median(res[q]):+.4f} m  ({np.median(res[q]/ (r[q]*nu[q]))*100:+.3f} % of range)')
