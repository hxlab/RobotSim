import numpy as np
import matplotlib.pyplot as plt
import os
from matplotlib.lines import Line2D

# ─── CUSTOMIZE THIS ───────────────────────────────────────────────────────────
RUNS = [
    "500StiffnessCritDamping10Mass",
    "250StiffnessCritDamping10Mass",
    "1000StiffnessCritDamping10Mass",
    "500StiffnessCritDamping100Mass"
]

# Maps filename label -> pretty legend label
RUN_LABELS = {
    "CritDamping10Mass":   r"$\zeta=1.0$, $M=10$",
    "CritDamping1000Mass": r"$\zeta=1.0$, $M=1000$",
    "0.5Zeta10Mass":       r"$\zeta=0.5$, $M=10$",
    "1.5Zeta10Mass":       r"$\zeta=1.5$, $M=10$",
}

DATA_DIR   = "GOODDATA_EXTERNALDISTURBANCE"
OUTPUT_DIR = "figures"
# ──────────────────────────────────────────────────────────────────────────────

os.makedirs(OUTPUT_DIR, exist_ok=True)

AXIS_STYLE = {
    'x': {'color': '#e74c3c', 'label': 'x'},
    'y': {'color': '#2ecc71', 'label': 'y'},
    'z': {'color': '#3498db', 'label': 'z'},
}
DASH_STYLES = ['-', '--', '-.', ':']

def plot_topic(topic_key, ylabel, filename, scale=1.0):
    fig, ax = plt.subplots(figsize=(7, 3.5))

    for i, run in enumerate(RUNS):
        path = os.path.join(DATA_DIR, f"{run}_{topic_key}.npz")
        if not os.path.exists(path):
            print(f"Missing: {path}, skipping.")
            continue
        d = np.load(path)
        dash = DASH_STYLES[i % len(DASH_STYLES)]
        for axis in ['x', 'y', 'z']:
            ax.plot(d['t'], d[axis] * scale,   # <── scale applied here
                    color=AXIS_STYLE[axis]['color'],
                    linestyle=dash,
                    linewidth=1.5)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle='--', alpha=0.4)

    # Custom legend handles
    axis_handles = [
        Line2D([0], [0], color=AXIS_STYLE[a]['color'], lw=2, label=a)
        for a in ['x', 'y', 'z']
    ]
    run_handles = [
        Line2D([0], [0], color='black', lw=2,
               linestyle=DASH_STYLES[i % len(DASH_STYLES)],
               label=RUN_LABELS.get(run, run))   # fallback to raw name if not found
        for i, run in enumerate(RUNS)
    ]

    leg1 = ax.legend(handles=axis_handles, title="Axis",
                     loc='upper right',
                     fontsize=8)
    leg2 = ax.legend(handles=run_handles, title="Run",
                     loc='lower left', bbox_to_anchor=(0.0, 1.01),
                     ncol=2, fontsize=8, borderaxespad=0,
                     handlelength=3.5)
    
    ax.add_artist(leg1)

    # bbox_inches='tight' expands the saved canvas to include all artists
    out = os.path.join(OUTPUT_DIR, filename)
    fig.savefig(out, dpi=300, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.close()

plot_topic('ee_wrench',    'Force (N)',    'ee_wrench.png')
plot_topic('ee_pos_error', 'Error (mm)',   'ee_pos_error.png', scale=1000.0)