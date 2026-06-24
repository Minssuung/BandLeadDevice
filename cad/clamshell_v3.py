import trimesh, numpy as np
import matplotlib; matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK KR"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
OUT="/home/minsung/dev_ws/BandLeadDevice/cad/out"
L=trimesh.load(f"{OUT}/grip_left_v3.stl"); R=trimesh.load(f"{OUT}/grip_right_v3.stl")
fig=plt.figure(figsize=(16,6))
views=[(20,-60,"닫힘(조립)",0),(20,-60,"분해(좌우 벌림)",22),(90,-90,"top",0)]
for k,(el,az,ttl,dx) in enumerate(views):
    ax=fig.add_subplot(1,3,k+1,projection="3d")
    ax.add_collection3d(Poly3DCollection((L.vertices+[-dx,0,0])[L.faces],facecolor=(.55,.8,.6),edgecolor="none",alpha=.55))
    ax.add_collection3d(Poly3DCollection((R.vertices+[dx,0,0])[R.faces],facecolor=(.4,.6,.95),edgecolor="none",alpha=.55))
    allv=np.vstack([L.vertices,R.vertices]);c=allv.mean(0);r=(allv.max(0)-allv.min(0)).max()/2
    ax.set_xlim(c[0]-r,c[0]+r);ax.set_ylim(c[1]-r,c[1]+r);ax.set_zlim(c[2]-r,c[2]+r)
    ax.view_init(el,az);ax.set_title(ttl,fontsize=10);ax.set_axis_off();ax.set_box_aspect((1,1,1))
fig.suptitle("그립 좌우 클램쉘 — 왼쪽(초록,IMU) + 오른쪽(파랑,리프트). 부품 눕히고 닫아 나사")
plt.tight_layout();plt.savefig(f"{OUT}/clamshell_v3.png",dpi=95);print("saved clamshell_v3.png")
