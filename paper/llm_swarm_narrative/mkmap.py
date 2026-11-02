import numpy as np, matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["pdf.fonttype"]=42
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from PIL import Image

# Recover the occupancy bitmap and map extent from the supplied vector PDF.
# Task coordinates below are unchanged from the original figure generator.
import fitz
from pathlib import Path
from io import BytesIO
source=fitz.open(Path(__file__).with_name('fig_map_original.pdf'))
page=source[0]
xref=page.get_images(full=True)[0][0]
image_rect=page.get_image_rects(xref)[0]
im=np.array(Image.open(BytesIO(source.extract_image(xref)['image'])).convert('L'))
scale=image_rect.width/66.6
first_blue=next(d['rect'] for d in page.get_drawings()
    if d['fill'] and abs(d['fill'][0]-0.1215686)<0.001 and d['rect'].width>12)
center=(first_blue.tl+first_blue.br)/2
ox=-6.84-(center.x-image_rect.x0)/scale
oy=1.37-(image_rect.y1-center.y)/scale
extent=[ox,ox+66.6,oy,oy+37.75]
occ=im<240

POINT = [(-6.84,1.37), (-6.31,-3.66), (-29.51,5.02),
         (-2.58,16.10), (7.84,-15.76), (12.13,2.05)]
CARRY = [((-23.67,-10.86), (-29.65,-2.85)),
         ((-15.99,  3.65), ( 12.29,-5.06)),
         (( 16.51,-12.19), (-29.65,-2.85))]
R = 1.5
BLUE="#1f6fb4"; ORANGE="#e2711d"
HOMES = [((-21.9, 9.9), "#00a5b5"), ((-8.7, -5.9), "#c2249a"),
         ((28.5, 1.9), "#2e9e3e"), ((0.6, -10.9), "#7a4fbf"),
         ((17.6, 10.7), "#c9a800")]

fig, ax = plt.subplots(figsize=(5.35, 3.03))
ax.imshow(np.flipud(occ), extent=extent, origin="lower",
          cmap="Greys", vmin=0, vmax=2.4, interpolation="nearest")

for (x0,y0),(x1,y1) in CARRY:
    ax.annotate("", xy=(x1,y1), xytext=(x0,y0),
                arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=1.7,
                                shrinkA=6.5, shrinkB=8.5, alpha=0.95),
                zorder=4)

for x,y in POINT:
    ax.add_patch(Circle((x,y), R, facecolor=BLUE, edgecolor="black",
                        lw=0.5, alpha=0.85, zorder=6))

seen=set()
for (x0,y0),(x1,y1) in CARRY:
    ax.add_patch(Circle((x0,y0), R, facecolor="white", edgecolor=ORANGE,
                        lw=1.5, zorder=6))
    if (x1,y1) not in seen:
        ax.add_patch(Circle((x1,y1), R, facecolor=ORANGE, edgecolor="black",
                            lw=0.5, alpha=0.9, zorder=7))
        seen.add((x1,y1))

for (x,y),c in HOMES:
    ax.plot(x, y, marker='s', ms=8.5, mfc=c, mec='black', mew=0.7,
            ls='none', zorder=8)

hp = [plt.Line2D([],[],marker='o',ls='none',ms=7,mfc=BLUE,mec='black',mew=0.5),
      plt.Line2D([],[],marker='o',ls='none',ms=7,mfc='white',mec=ORANGE,mew=1.5),
      plt.Line2D([],[],marker='o',ls='none',ms=7,mfc=ORANGE,mec='black',mew=0.5),
      plt.Line2D([],[],marker='s',ls='none',ms=7,mfc='#888888',mec='black',mew=0.7)]
ax.legend(hp, ["visit task", "carry: pickup", "carry: drop-off",
               "group home: 4 robots"],
          fontsize=9.5, loc="upper center", bbox_to_anchor=(0.5,-0.01), frameon=True, ncol=2, framealpha=0.9,
          borderpad=0.4, handletextpad=0.4, labelspacing=0.35)

ax.set_xlim(extent[0], extent[1]); ax.set_ylim(extent[2], extent[3])
ax.set_xticks([]); ax.set_yticks([])
for sp in ax.spines.values(): sp.set_linewidth(0.6)
ax.set_aspect("equal")
fig.tight_layout(pad=0.12)
fig.savefig(Path(__file__).with_name("fig_map.pdf"), bbox_inches="tight")
print("ok")
