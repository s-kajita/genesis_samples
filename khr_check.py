import genesis as gs
import numpy as np
from scipy.interpolate import CubicHermiteSpline
import matplotlib.pyplot as plt


#-------------------- Genesisの初期化 ----------------
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
        file="../assets/khr3hv/urdf/khr3hv_d.urdf",  #頭の位置が正しく表示される
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


q_init = robot.get_qpos().clone()

input("Press Enter to exit...")
