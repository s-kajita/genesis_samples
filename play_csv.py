import numpy as np
import genesis as gs

q_pattern = np.loadtxt('patterns/jump2_test.csv', delimiter=',')

# 時刻配列
Dtime = 0.01  # [s]  = 100Hz
N = len(q_pattern)
time = np.array(range(N+1),float)*Dtime

#-------------------- Genesisの初期化 ----------------
gs.init(backend=gs.cpu, logging_level="warning")


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

# 床の追加
scene.add_entity(gs.morphs.Plane())

# KHR-3HV
base_pos  = [0.0, 0.0, 0.256]     #本体の初期高さ
base_quat = [1.0, 0.0, 0.0, 0.0] #本体の初期姿勢
robot = scene.add_entity(
    gs.morphs.URDF(
        file="../assets/khr3hv/urdf/khr3hv_d.urdf", 
        pos =base_pos,
        quat=base_quat,
    ),
)

# シーンの構築
scene.build()
robot.set_dofs_armature([0]*6+[0.01]*22)

base_link = robot.get_link("base_link")
r_foot = robot.get_link("r_foot")
l_foot = robot.get_link("l_foot")

print("r_foot:",r_foot.get_pos().cpu().numpy())

joint_names = [
    "c_chest_yaw",
    "l_shoulder_pitch",
    "l_shoulder_roll",
    "l_elbow_yaw",
    "l_elbow_pitch",

    "c_head_yaw",
    "r_shoulder_pitch",
    "r_shoulder_roll",
    "r_elbow_yaw",
    "r_elbow_pitch",
   
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

joint2robot = np.argsort(leg_dofs)

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
    kp             = [25.0]*22,
    dofs_idx_local = leg_dofs,
)
# 速度ゲインの設定
robot.set_dofs_kv(
    kv             = [0.5]*22,
    dofs_idx_local = leg_dofs,
)


#------------------- 動力学シミュレーション ---------------------
print("Dynamic simulation")

#camera.start_recording()
base_q = np.array([0.,0.,0., 0.,0.,0.])

for i,q in enumerate(q_pattern):
    full_q = np.concatenate([base_q,q[joint2robot]])
    robot.control_dofs_position(full_q)
    scene.step()
    # 動画のフレームを記録
    #if i % 3 == 0:
    #    camera.render()
for n in range(50):
    scene.step()
    # 動画のフレームを記録
    #if n % 3 == 0:
    #    camera.render()

