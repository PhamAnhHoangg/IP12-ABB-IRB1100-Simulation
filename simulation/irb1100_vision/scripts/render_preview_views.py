import sys, numpy as np, yourdfpy, trimesh, xml.etree.ElementTree as ET
from PIL import Image, ImageDraw, ImageFont
ROOT='/home/claude/irb1100_vision/'; URDF=ROOT+'urdf/workcell.urdf'
mats={m.get('name'):[float(x) for x in m.find('color').get('rgba').split()] for m in ET.parse(URDF).getroot().findall('material')}
h=lambda fname:fname.replace('package://irb1100_vision/',ROOT).replace('../',ROOT)
u=yourdfpy.URDF.load(URDF,build_scene_graph=True,load_meshes=True,build_collision_scene_graph=False,filename_handler=h)
def pose(q): u.update_cfg(np.array(q,float))
def scene():
    tris=[];cols=[];nrm=[]
    for ln,l in u.link_map.items():
        T=u.get_transform(ln,'world')
        for v in l.visuals:
            g=v.geometry
            if g.box is not None: m=trimesh.creation.box(g.box.size)
            elif g.cylinder is not None: m=trimesh.creation.cylinder(g.cylinder.radius,g.cylinder.length,sections=24)
            elif g.mesh is not None:
                m=trimesh.load(h(g.mesh.filename),process=False); m.apply_scale(g.mesh.scale if g.mesh.scale is not None else 1)
            else: continue
            O=v.origin if v.origin is not None else np.eye(4)
            m.apply_transform(T@O)
            c=mats.get(v.material.name,[.6,.6,.6,1])[:3] if v.material and v.material.name else [.6,.6,.6]
            tris.append(m.vertices[m.faces]); nrm.append(m.face_normals); cols+=[c]*len(m.faces)
    return np.concatenate(tris),np.concatenate(nrm),np.array(cols)
def render(S,T,W,H,hfov,bg=(196,200,206)):
    tris,nrm,cols=S; inv=np.linalg.inv(T)
    P=(inv[:3,:3]@tris.reshape(-1,3).T).T+inv[:3,3]; P=P.reshape(-1,3,3)
    z=P[:,:,2]; ok=(z>0.05).all(1); f=(W/2)/np.tan(hfov/2)
    uv=np.stack([P[:,:,0]/np.maximum(z,1e-6)*f+W/2,P[:,:,1]/np.maximum(z,1e-6)*f+H/2],-1)
    Lw=np.array([0.3,-0.5,0.8]);Lw/=np.linalg.norm(Lw)
    shade=0.45+0.55*np.abs(nrm@Lw)
    idx=np.where(ok)[0]; idx=idx[np.argsort(-z.mean(1)[idx])]
    img=Image.new('RGB',(W,H),bg); d=ImageDraw.Draw(img)
    for i in idx:
        d.polygon([tuple(p) for p in uv[i]],fill=tuple(int(255*min(1,c*shade[i])) for c in cols[i]))
    return img
def lookat(eye,tgt,up=(0,0,1)):
    eye=np.array(eye,float);z=np.array(tgt,float)-eye;z/=np.linalg.norm(z)
    x=np.cross(z,up);x/=np.linalg.norm(x);y=np.cross(z,x)
    T=np.eye(4);T[:3,0]=x;T[:3,1]=y;T[:3,2]=z;T[:3,3]=eye;return T
def label(img,txt):
    d=ImageDraw.Draw(img)
    try: f=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',26)
    except: f=None
    d.text((16,12),txt,fill=(0,0,0),font=f)
if __name__=='__main__':
    q=[float(x) for x in open('/home/claude/work/home_q.txt').read().split()]; pose(q); S=scene()
    W,H=1280,720
    l=render(S,u.get_transform('cam2_left_optical_frame','world'),W,H,np.pi/2); r=render(S,u.get_transform('cam2_right_optical_frame','world'),W,H,np.pi/2)
    both=Image.new('RGB',(W*2+10,H),(255,255,255)); both.paste(l,(0,0)); both.paste(r,(W+10,0)); label(both,'ZED Mini LEFT (simulated)')
    ImageDraw.Draw(both).text((W+26,12),'ZED Mini RIGHT (simulated)',fill=(0,0,0),font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',26))
    both.save('zed_stereo_view.png')
    ov=render(S,lookat((1.5,-1.3,1.7),(0.1,0.0,0.95)),1400,900,0.9); label(ov,'Workcell overview (software render, ABB CAD arm, park pose)'); ov.save('overview.png')
    # zero pose close-up of the arm
    pose([0]*6); S0=scene()
    zc=render(S0,lookat((0.2+0.9,-0.9,0.75+0.8),(0.2+0.15,0,0.75+0.35)),1200,900,0.8); label(zc,'IRB 1100-4/0.58 zero pose, from ABB CAD'); zc.save('arm_zero_pose.png')
    pose([0.6,0.5,0.6,0,0.7,0]); S1=scene()
    zc=render(S1,lookat((0.2+0.9,-0.9,0.75+0.8),(0.2+0.15,0,0.75+0.35)),1200,900,0.8); label(zc,'example pose'); zc.save('arm_example_pose.png')
    print('done')
