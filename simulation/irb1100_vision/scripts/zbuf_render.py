import numpy as np
from PIL import Image
try:
    from numba import njit
except Exception:
    njit=None
def _raster(uv,z,col,W,H,bg):
    img=np.empty((H,W,3),np.float32); img[:]=bg
    zb=np.full((H,W),np.inf,np.float32)
    for i in range(len(uv)):
        a,b,c=uv[i]
        x0=int(max(0,np.floor(min(a[0],b[0],c[0])))); x1=int(min(W-1,np.ceil(max(a[0],b[0],c[0]))))
        y0=int(max(0,np.floor(min(a[1],b[1],c[1])))); y1=int(min(H-1,np.ceil(max(a[1],b[1],c[1]))))
        if x1<x0 or y1<y0: continue
        d=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(d)<1e-9: continue
        xs=np.arange(x0,x1+1)[None,:]+0.5; ys=np.arange(y0,y1+1)[:,None]+0.5
        l1=((b[1]-c[1])*(xs-c[0])+(c[0]-b[0])*(ys-c[1]))/d
        l2=((c[1]-a[1])*(xs-c[0])+(a[0]-c[0])*(ys-c[1]))/d
        l3=1-l1-l2
        m=(l1>=-1e-6)&(l2>=-1e-6)&(l3>=-1e-6)
        if not m.any(): continue
        zz=l1*z[i,0]+l2*z[i,1]+l3*z[i,2]
        sub=zb[y0:y1+1,x0:x1+1]; upd=m&(zz<sub)
        if not upd.any(): continue
        sub[upd]=zz[upd]; img[y0:y1+1,x0:x1+1][upd]=col[i]
    return img
def render_z(tris,nrm,cols,T,W,H,hfov,bg=(255,255,255),light=(0.3,-0.5,0.8)):
    inv=np.linalg.inv(T)
    P=(inv[:3,:3]@tris.reshape(-1,3).T).T+inv[:3,3]; P=P.reshape(-1,3,3)
    z=P[:,:,2]; ok=(z>0.02).all(1)
    f=(W/2)/np.tan(hfov/2)
    uv=np.stack([P[:,:,0]/np.maximum(z,1e-6)*f+W/2,P[:,:,1]/np.maximum(z,1e-6)*f+H/2],-1)
    inb=(uv[:,:,0].max(1)>=0)&(uv[:,:,0].min(1)<W)&(uv[:,:,1].max(1)>=0)&(uv[:,:,1].min(1)<H)
    sel=ok&inb
    L=np.array(light,float);L/=np.linalg.norm(L)
    nc=nrm@(T[:3,:3]@np.array([0,0,0.0]) if False else np.zeros(3))  # unused
    shade=0.5+0.5*np.abs(nrm@L)
    col=(cols*shade[:,None])[sel].astype(np.float32)*255
    img=_raster(uv[sel].astype(np.float64),z[sel].astype(np.float32),np.clip(col,0,255),W,H,np.array(bg,np.float32))
    return Image.fromarray(img.astype(np.uint8))
