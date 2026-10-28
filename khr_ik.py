import genesis as gs
import numpy as np

# Genesisの初期化
gs.init(backend=gs.cpu)

# シーンの作成
scene = gs.Scene(
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(2.0, -2.0, 1.5),
        camera_lookat=(0.0, 0.0, 0.5),
    ),
)

# 床の追加
scene.add_entity(gs.morphs.Plane())

# Pandaロボットの読み込み
'''
robot = scene.add_entity(
    gs.morphs.MJCF(
        file="xml/franka_emika_panda/panda.xml"
    )
)
'''
# KHR-3HV
base_pos  = [0.0, 0.0, 0.25]     #本体の初期高さ
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

left_pos = np.array([0.0168, 0.0177, 0.0383])
#right_pos = np.array([0.0168, -0.0177, 0.0383])
right_pos = np.array([0.0168, -0.0177, 0.07])

left_quat = np.array([1.0, 0.0, 0.0, 0.0])
right_quat = np.array([1.0, 0.0, 0.0, 0.0])

q_init = robot.get_qpos().clone()

q_ik = robot.inverse_kinematics_multilink(
    links=[l_foot, r_foot],
    poss=[left_pos, right_pos],
    quats=[left_quat, right_quat],
    init_qpos=q_init,
    dofs_idx_local=leg_dofs,
    max_solver_iters=100,
)

input("Press Enter to exit...")


robot.set_qpos(q_ik)          # IKで求めた関節位置をロボットに設定
scene.visualizer.update()     # ビューアの更新
input("Press Enter to exit...")
