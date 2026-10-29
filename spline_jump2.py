import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicHermiteSpline

Ttrans = 1.0
Tjump = 0.1
Tsup  = 0.4
Gacc = 9.8    # m/s^2
Vz = Gacc*Tjump/2.0   # liftoff speed
Zliftoff = -0.03  # liftoff height = touchdown height

T = [0.]
X = [0.]
V = [0.]

# the first lift off
T.append(T[-1]+Ttrans)
X.append(Zliftoff)
V.append(Vz)

Njump = 5
for n in range(Njump):
    # touch down
    T.append(T[-1]+Tjump)
    X.append(Zliftoff)
    V.append(-Vz)
    if n < Njump-1:
        # support for next jump
        T.append(T[-1]+Tsup)
        X.append(Zliftoff)
        V.append(Vz)
    else:
        # finish jump
        T.append(T[-1]+Ttrans)
        X.append(0.)
        V.append(0.)
   
# 3次スプライン補間
spline = CubicHermiteSpline(
    T,X,V
)

# 時刻配列
Dtime = 0.02  # [s]  = 50Hz
N = int(T[-1] / Dtime)
t = np.array(range(N+1),float)*Dtime

# 位置、速度、加速度
x = spline(t)
v = spline(t, 1)
a = spline(t, 2)

# グラフ表示
fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

axes[0].plot(t, x)
axes[0].plot(T, X, 'ro')
axes[0].set_ylabel('Position [m]')
axes[0].grid()

axes[1].plot(t, v)
axes[1].plot(T, V, 'ro')
axes[1].set_ylabel('Velocity [m/s]')
axes[1].grid()

axes[2].plot(t, a)
axes[2].set_ylabel('Acceleration [m/s²]')
axes[2].set_xlabel('Time [s]')
axes[2].grid()

plt.tight_layout()
plt.show()