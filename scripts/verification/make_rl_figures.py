"""Render exact lesson10 numerical objective; no policy training claims."""
import importlib.util
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('ppo_numbers',root/'examples/lesson10/ppo_numbers.py')
lab=importlib.util.module_from_spec(spec)
spec.loader.exec_module(lab)
fig,axes=plt.subplots(1,2,figsize=(10,3.8),sharex=True)
ratios=[i/200 for i in range(401)]
for ax,advantage in zip(axes,[2.,-2.]):
 ax.plot(ratios,[x*advantage for x in ratios],'--',color='#8e99a6',label='Unclipped r A')
 ax.plot(ratios,[lab.clipped_surrogate(x,advantage,.2) for x in ratios],color='#1565c0',lw=2.5,label='PPO objective')
 ax.axvline(.8,color='#dd8a11',alpha=.6)
 ax.axvline(1.2,color='#dd8a11',alpha=.6)
 ax.set(title=f'Advantage A = {advantage:+.0f}',xlabel='Probability ratio r',ylabel='Objective (maximize)')
 ax.grid(alpha=.2)
 ax.legend(loc='best')
fig.suptitle('PPO-Clip: min(r A, clip(r, 0.8, 1.2) A)')
fig.tight_layout()
fig.savefig(root/'docs/assets/lesson10_ppo.png',dpi=180)
print('saved docs/assets/lesson10_ppo.png')
