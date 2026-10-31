import genesis as gs
import numpy as np
from scipy.interpolate import CubicHermiteSpline
import matplotlib.pyplot as plt

# 連続ジャンプのための境界条件
Ttrans = 1.0
Tjump = 0.05
Tsup  = 0.3
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

Njump = 4
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
z_pos = spline(time)
z_vel = spline(time,1)
z_acc = spline(time,2)

'''
# グラフ表示
fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

axes[0].plot(time, z_pos)
axes[0].plot(T, X, 'ro')
axes[0].set_ylabel('Position [m]')
axes[0].grid()

axes[1].plot(time, z_vel)
axes[1].plot(T, V, 'ro')
axes[1].set_ylabel('Velocity [m/s]')
axes[1].grid()

axes[2].plot(time, z_acc)
axes[2].set_ylabel('Acceleration [m/s²]')
axes[2].set_xlabel('Time [s]')
axes[2].grid()

plt.tight_layout()
plt.show()
'''

#-------------------- Genesisの初期化 ----------------
gs.init(backend=gs.cpu)


CameraPos = (0.7, -0.7, 0.7)
CameraLookAt = (0.0, 0.0, 0.25)

# シーンの作成
scene = gs.Scene(
    show_viewer=True,

    sim_options=gs.options.SimOptions(
        dt=Dtime,
    ),

    viewer_options=gs.options.ViewerOptions(
        camera_pos= CameraPos,
        camera_lookat= CameraLookAt,
    ),
)

#-------------------- 録画用カメラの追加 ----------------
camera = scene.add_camera(
    res=(1280, 720),
    pos= CameraPos,
    lookat= CameraLookAt,
    fov=40,
    GUI=False,
)

# 床の追加
scene.add_entity(gs.morphs.Plane())

# KHR-3HV
base_pos  = [0.0, 0.0, 0.256]     #本体の初期高さ
base_quat = [1.0, 0.0, 0.0, 0.0] #本体の初期姿勢
robot = scene.add_entity(
    gs.morphs.URDF(
        file="../assets/khr3hv/urdf/khr3hv_12dof.urdf", 
        pos =base_pos,
        quat=base_quat,
    ),
)

# シーンの構築
scene.build()
robot.set_dofs_armature([0]*6+[0.01]*12)

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

############ 制御ゲインの設定 ############
# 位置ゲインの設定
robot.set_dofs_kp(
    kp             = [25.0]*12,
    dofs_idx_local = leg_dofs,
)
# 速度ゲインの設定
robot.set_dofs_kv(
    kv             = [0.5]*12,
    dofs_idx_local = leg_dofs,
)

q_log = []

#------------------ パターン生成とアニメーション -------------------
for z in z_pos:
    qpos = q_init.clone()
    qpos[0] += 0.006  # 腰水平位置調整
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
    q_log.append(q_ik)        # 関節角の保存

#------------------- 動力学シミュレーション ---------------------
print("Dynamic simulation")

camera.start_recording()

for i,q in enumerate(q_log):
    robot.control_dofs_position(q[1:])
    scene.step()
    # 動画のフレームを記録
    if i % 3 == 0:
        camera.render()
for n in range(100):
    scene.step()
    # 動画のフレームを記録
    if n % 3 == 0:
        camera.render()

input("Press Enter to exit...")# 録画終了とファイル保存
camera.stop_recording(
    save_to_filename="khr_dynamics.mp4",
    fps=int(100/3),
)

