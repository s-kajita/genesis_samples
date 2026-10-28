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
robot = scene.add_entity(
    gs.morphs.MJCF(
        file="xml/franka_emika_panda/panda.xml"
    )
)

# シーンの構築
scene.build()

# 手先リンクを取得
ee_link = robot.get_link("hand")

# 初期位置の取得（FK）
initial_pos = ee_link.get_pos().cpu().numpy()
print("Initial EE position:", initial_pos)
initial_quat = ee_link.get_quat().cpu().numpy()
print("Initial EE quaternion:", initial_quat)


# 目標手先位置 [m]
#target_pos = np.array([0.55, 0.0, 0.35])
target_pos = np.array([0,0,0.7])

# 目標手先姿勢（クォータニオン w,x,y,z）
#target_quat = np.array([0.0, 1.0, 0.0, 0.0])
target_quat = initial_quat

# 逆運動学（IK）の計算
q_ik = robot.inverse_kinematics(
    link=ee_link,
    pos=target_pos,
    quat=target_quat,
)

print("IK joint positions:")
print(q_ik)

# IKで求めた関節位置をロボットに設定
robot.set_qpos(q_ik)

# FKによる手先位置の検証
actual_pos = ee_link.get_pos().cpu().numpy()

print("Target position:", target_pos)
print("Actual position:", actual_pos)
print("Position error:",
      np.linalg.norm(actual_pos - target_pos))

input("Press Enter to move")
# ビューアの更新
scene.visualizer.update()

input("Press Enter to exit...")