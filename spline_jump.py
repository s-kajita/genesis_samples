import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicHermiteSpline

# 境界条件
t1, x1, v1 = 0.0, 0.5, 0.0
t2, x2, v2 = 1.0, 0.5, 0.5   # lift off
t3, x3, v3 = 1.1, 0.5, -0.5  # touch down
t4, x4, v4 = 1.5, 0.5, 0.5   # lift off
t5, x5, v5 = 1.6, 0.5, -0.5  # touch down
t6, x6, v6 = 2.6, 0.5, 0.0

# 3次スプライン補間
spline = CubicHermiteSpline(
    [t1, t2, t3, t4, t5, t6],
    [x1, x2, x3, x4, x5, x6],
    [v1, v2, v3, v4, v5, v6]
)

# 時刻配列
Dtime = 0.02  # [s]  = 50Hz
N = int(t6 / Dtime)
t = np.array(range(N+1),float)*Dtime

# 位置、速度、加速度
x = spline(t)
v = spline(t, 1)
a = spline(t, 2)

# グラフ表示
fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

axes[0].plot(t, x)
axes[0].plot([t1, t2], [x1, x2], 'ro')
axes[0].set_ylabel('Position [m]')
axes[0].grid()

axes[1].plot(t, v)
axes[1].plot([t1, t2], [v1, v2], 'ro')
axes[1].set_ylabel('Velocity [m/s]')
axes[1].grid()

axes[2].plot(t, a)
axes[2].set_ylabel('Acceleration [m/s²]')
axes[2].set_xlabel('Time [s]')
axes[2].grid()

plt.tight_layout()
plt.show()