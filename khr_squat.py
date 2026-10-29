import genesis as gs
import numpy as np
from scipy.interpolate import CubicHermiteSpline

# 連続ジャンプのための境界条件
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
    T,
    X,
    V,
)

# 時刻配列
Dtime = 0.01  # [s]  = 100Hz
N = int(T[-1] / Dtime)
time = np.array(range(N+1),float)*Dtime

# ジャンプパターン
z_pattern = spline(time)


# Genesisの初期化
gs.init(backend=gs.cpu)

# シーンの作成
scene = gs.Scene(
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(1.0, -1.0, 1.0),
        camera_lookat=(0.0, 0.0, 0.25),
    ),
)

# 床の追加
scene.add_entity(gs.morphs.Plane())

# KHR-3HV
base_pos  = [0.0, 0.0, 0.256]     #本体の初期高さ
base_quat = [1.0, 0.0, 0.0, 0.0] #本体の初期姿勢
robot = scene.add_entity(
    gs.morphs.URDF(
        file="../assets/khr3hv/urdf/khr3hv_12dof.urdf",  #頭の位置が正しく表示される
        pos=base_pos,
        quat=base_quat,
    ),
    #dofs_armature=np.array([0.01]*22),
)

# シーンの構築
scene.build()

base_link = robot.get_link("base_link")
r_foot = robot.get_link("r_foot")
l_foot = robot.get_link("l_foot")

print("r_foot:",r_foot.get_pos().cpu().numpy())


joint_names = [
    "l_hip_yaw",
    "l_hip_roll",
    "l_hip_pitch",
    "l_knee_pitch",
    "l_ankle_pitch",
    "l_ankle_roll",

    "r_hip_yaw",
    "r_hip_roll",
    "r_hip_pitch",
    "r_knee_pitch",
    "r_ankle_pitch",
    "r_ankle_roll",
]

leg_dofs = [
    robot.get_joint(name).dofs_idx_local[0]
    for name in joint_names
]

# --------------------------------------------------
# 5. 左右足部の目標姿勢
# --------------------------------------------------

left_pos_ini = np.array([0.0168, 0.0177, 0.024])
right_pos_ini = np.array([0.0168, -0.0177, 0.024])

left_quat = np.array([1.0, 0.0, 0.0, 0.0])
right_quat = np.array([1.0, 0.0, 0.0, 0.0])

q_init = robot.get_qpos().clone()

for z in z_pattern:
    qpos = q_init.clone()
    qpos[2] += z    # set hip z motion
    print("z=",z,", qpos[2]=",qpos[2])

    q_ik = robot.inverse_kinematics_multilink(
        links=[l_foot, r_foot],
        poss=[left_pos_ini, right_pos_ini],
        quats=[left_quat, right_quat],
        init_qpos=qpos,
        dofs_idx_local=leg_dofs,
        max_solver_iters=100,
    )
    robot.set_qpos(q_ik)          # IKで求めた関節位置をロボットに設定
    scene.visualizer.update()     # ビューアの更新

input("Press Enter to exit...")
