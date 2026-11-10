"""Generate the conceptual interface figure without external map assets."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':10, 'pdf.fonttype':42})
fig, axes=plt.subplots(2,2,figsize=(5.35,2.9))
fig.subplots_adjust(left=.015,right=.985,bottom=.08,top=.91,wspace=.12,hspace=.5)
panels=[
    ('1  Operator command', 'Move cargo from Security\nto Admin.'),
    ('2  Typed plan', 'One pickup and drop-off leg\nplus an extra pickup assignment'),
    ('3  Assignment correction', 'Remove the extra pickup leg\nKeep the routed carrier'),
    ('4  Independent outcome check', 'Execute the effective plan\nCheck pickup, then drop-off')
]
for ax,(title,text) in zip(axes.flat,panels):
    ax.set_axis_off()
    ax.add_patch(FancyBboxPatch((.02,.06),.96,.82,boxstyle='round,pad=0.012',
                              fc='#f0f4f8',ec='#667788',lw=.8,transform=ax.transAxes))
    ax.text(.02,1.0,title,weight='bold',va='bottom',transform=ax.transAxes)
    ax.text(.5,.47,text,ha='center',va='center',linespacing=1.6,transform=ax.transAxes)
fig.savefig(Path(__file__).with_name('Fig1.pdf'),bbox_inches='tight')
