# 导入Isaac Lab仿真工具包，包含URDF文件处理等功能
import isaaclab.sim as sim_utils
# 导入Isaac Lab中的执行器配置类
from isaaclab.actuators import ImplicitActuatorCfg
# 导入Isaac Lab中关节结构体的基础配置类
from isaaclab.assets.articulation import ArticulationCfg

# 从本项目资源模块导入资产目录路径
from whole_body_tracking.assets import ASSET_DIR

# 定义5020型号电机的转动惯量参数
ARMATURE_5020 = 0.003609725
# 定义7520-14型号电机的转动惯量参数
ARMATURE_7520_14 = 0.010177520
# 定义7520-22型号电机的转动惯量参数
ARMATURE_7520_22 = 0.025101925
# 定义4010型号电机的转动惯量参数
ARMATURE_4010 = 0.00425

# 计算自然频率，设置为10Hz（弧度/秒）
NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
# 设置阻尼比，数值为临界阻尼的2倍
DAMPING_RATIO = 2.0

# 刚度系数
# 根据转动惯量和自然频率计算5020型号电机的刚度系数
STIFFNESS_5020 = ARMATURE_5020 * NATURAL_FREQ**2
# 根据转动惯量和自然频率计算7520-14型号电机的刚度系数
STIFFNESS_7520_14 = ARMATURE_7520_14 * NATURAL_FREQ**2
# 根据转动惯量和自然频率计算7520-22型号电机的刚度系数
STIFFNESS_7520_22 = ARMATURE_7520_22 * NATURAL_FREQ**2
# 根据转动惯量和自然频率计算4010型号电机的刚度系数
STIFFNESS_4010 = ARMATURE_4010 * NATURAL_FREQ**2

# 阻尼系数
# 根据转动惯量、阻尼比和自然频率计算5020型号电机的阻尼系数
DAMPING_5020 = 2.0 * DAMPING_RATIO * ARMATURE_5020 * NATURAL_FREQ
# 根据转动惯量、阻尼比和自然频率计算7520-14型号电机的阻尼系数
DAMPING_7520_14 = 2.0 * DAMPING_RATIO * ARMATURE_7520_14 * NATURAL_FREQ
# 根据转动惯量、阻尼比和自然频率计算7520-22型号电机的阻尼系数
DAMPING_7520_22 = 2.0 * DAMPING_RATIO * ARMATURE_7520_22 * NATURAL_FREQ
# 根据转动惯量、阻尼比和自然频率计算4010型号电机的阻尼系数
DAMPING_4010 = 2.0 * DAMPING_RATIO * ARMATURE_4010 * NATURAL_FREQ

# 创建Avada机器人的URDF配置对象，定义机器人模型的物理属性和初始状态
Avada_CYLINDER_CFG = ArticulationCfg(
    # 定义机器人的生成配置，包括URDF文件路径和物理属性
    spawn=sim_utils.UrdfFileCfg(
        # 设置基座是否固定（False表示不固定，允许自由移动）
        fix_base=False,
        # 启用圆柱体替换功能，将URDF中的圆柱体替换为胶囊体以提高碰撞检测性能
        replace_cylinders_with_capsules=True,
        # 指定URDF文件路径，使用ASSET_DIR变量确保路径正确
        asset_path=f"{ASSET_DIR}/avada4/urdf/avada4.urdf", 
        # 激活接触传感器，用于检测碰撞
        activate_contact_sensors=True,
        # 定义刚体物理属性配置
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            # 不禁用重力（即启用重力）
            disable_gravity=False,
            # 不保留加速度
            retain_accelerations=False,
            # 线性阻尼系数，影响物体线性运动时的阻力
            linear_damping=0.0,
            # 角阻尼系数，影响物体旋转时的阻力
            angular_damping=0.0,
            # 最大线性速度限制
            max_linear_velocity=1000.0,
            # 最大角速度限制
            max_angular_velocity=1000.0,
            # 最大穿透速度限制，控制物体穿透后的分离速度，增加此值以防止脚掌陷入地面
            max_depenetration_velocity=1.0,
        ),
        # 定义关节根节点属性
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # 启用自碰撞检测，防止机器人不同部分相互穿透
            enabled_self_collisions=True, 
            # 位置求解迭代次数，影响物理模拟精度
            solver_position_iteration_count=8, 
            # 速度求解迭代次数，影响物理模拟精度
            solver_velocity_iteration_count=4
        ),
        # 定义关节驱动配置
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            # 定义PD增益配置，设置刚度和阻尼为0（默认值，可能被具体执行器覆盖）
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    # 定义机器人初始状态配置
    init_state=ArticulationCfg.InitialStateCfg(
        # 初始位置设为(0,0,0.9)，高度约0.86米
        pos=(0.0, 0.0, 0.86),
        # 初始化各关节位置，设置腿部和手臂的初始姿态
        joint_pos={
            # 所有髋部俯仰关节的初始角度为-0.312弧度
            ".*_hip_pitch_joint": 0.0,
            # 所有膝盖关节的初始角度为0.669弧度
            ".*_knee_joint": 0.0,
            # 所有踝部俯仰关节的初始角度为-0.363弧度
            ".*_ankle_pitch_joint": 0.0,
            # 所有肘关节的初始角度为0.6弧度
            ".*_elbow_joint": 0.0,
            # 左肩滚转关节的初始角度为0.2弧度
            "left_shoulder_roll_joint": 0.0,
            # 左肩俯仰关节的初始角度为0.2弧度
            "left_shoulder_pitch_joint": 0.0,
            # 右肩滚转关节的初始角度为-0.2弧度
            "right_shoulder_roll_joint": 0.0,
            # 右肩俯仰关节的初始角度为0.2弧度
            "right_shoulder_pitch_joint": 0.0,
        },
        # 初始化所有关节速度为0
        joint_vel={".*": 0.0},
    ),
    # 软性关节位置限制因子，设置为90%的硬件限制范围，增加安全余量
    soft_joint_pos_limit_factor=0.9,
    # 定义各类执行器配置
    actuators={
        # 定义腿部执行器配置
        "legs": ImplicitActuatorCfg(
            # 定义腿部关节名称表达式，匹配所有相关的腿部关节
            joint_names_expr=[
                ".*_hip_yaw_joint",      # 髋部偏航关节
                ".*_hip_roll_joint",     # 髋部滚动关节
                ".*_hip_pitch_joint",    # 髋部俯仰关节
                ".*_knee_joint",         # 膝关节
            ],
            # 定义腿部执行器的力矩限制
            effort_limit_sim={
                ".*_hip_yaw_joint": 800,    # 髋部偏航关节力矩限制为88.0 Nm
                ".*_hip_roll_joint": 800,  # 髋部滚动关节力矩限制为139.0 Nm
                ".*_hip_pitch_joint": 800,  # 髋部俯仰关节力矩限制为88.0 Nm
                ".*_knee_joint": 800,     # 膝关节力矩限制为139.0 Nm
            },
            # 定义腿部执行器的速度限制
            velocity_limit_sim={
                ".*_hip_yaw_joint": 120,    # 髋部偏航关节速度限制为32.0 rad/s
                ".*_hip_roll_joint": 120.0,   # 髋部滚动关节速度限制为20.0 rad/s
                ".*_hip_pitch_joint": 120.0,  # 髋部俯仰关节速度限制为32.0 rad/s
                ".*_knee_joint": 120.0,      # 膝关节速度限制为20.0 rad/s
            },
            # 定义腿部执行器的刚度系数
            stiffness={
                ".*_hip_pitch_joint": 400,  # 髋部俯仰关节使用7520-14型号刚度
                ".*_hip_roll_joint": 400,   # 髋部滚动关节使用7520-22型号刚度
                ".*_hip_yaw_joint": 400,    # 髋部偏航关节使用7520-14型号刚度
                ".*_knee_joint": 600,       # 膝关节使用7520-22型号刚度
            },
            # 定义腿部执行器的阻尼系数
            damping={
                ".*_hip_pitch_joint": 5,    # 髋部俯仰关节使用7520-14型号阻尼
                ".*_hip_roll_joint": 5,     # 髋部滚动关节使用7520-22型号阻尼
                ".*_hip_yaw_joint": 5,      # 髋部偏航关节使用7520-14型号阻尼
                ".*_knee_joint": 8,         # 膝关节使用7520-22型号阻尼
            },
            # 定义腿部执行器的转动惯量
            armature={
                ".*_hip_pitch_joint": ARMATURE_7520_14,   # 髋部俯仰关节使用7520-14型号转动惯量
                ".*_hip_roll_joint": ARMATURE_7520_22,    # 髋部滚动关节使用7520-22型号转动惯量
                ".*_hip_yaw_joint": ARMATURE_7520_14,     # 髋部偏航关节使用7520-14型号转动惯量
                ".*_knee_joint": ARMATURE_7520_22,        # 膝关节使用7520-22型号转动惯量
            },
        ),
        # 定义足部执行器配置
        "feet": ImplicitActuatorCfg(
            # 足部执行器力矩限制为50.0 Nm
            effort_limit_sim=100.0,
            # 足部执行器速度限制为37.0 rad/s
            velocity_limit_sim=100.0,
            # 定义足部关节名称表达式，匹配踝部俯仰和滚动关节
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            # 足部执行器刚度设为5020型号的2倍
            stiffness=100.0,
            # 足部执行器阻尼设为5020型号的2倍
            damping=4.0,
            # 足部执行器转动惯量设为5020型号的2倍
            armature=4.0 * ARMATURE_5020,
        ),
        # 定义腰部执行器配置
        "waist": ImplicitActuatorCfg(
            # 腰部执行器力矩限制为50 Nm
            effort_limit_sim=800,
            # 腰部执行器速度限制为37.0 rad/s
            velocity_limit_sim=100.0,
            # 定义腰部关节名称表达式，匹配腰部滚动和俯仰关节
            joint_names_expr=["waist_roll_joint", "waist_pitch_joint"],
            # 腰部执行器刚度设为5020型号的2倍
            stiffness=800,
            # 腰部执行器阻尼设为5020型号的2倍
            damping=5,
            # 腰部执行器转动惯量设为5020型号的2倍
            armature=2.0 * ARMATURE_5020,
        ),
        # 定义腰部偏航执行器配置
        "waist_yaw": ImplicitActuatorCfg(
            # 腰部偏航执行器力矩限制为88 Nm
            effort_limit_sim=800,
            # 腰部偏航执行器速度限制为32.0 rad/s
            velocity_limit_sim=120.0,
            # 定义腰部偏航关节名称表达式
            joint_names_expr=["waist_yaw_joint"],
            # 腰部偏航执行器使用7520-14型号刚度
            stiffness=800,
            # 腰部偏航执行器使用7520-14型号阻尼
            damping=10,
            # 腰部偏航执行器使用7520-14型号转动惯量
            armature=ARMATURE_7520_14,
        ),
        # 定义手臂执行器配置
        "arms": ImplicitActuatorCfg(
            # 定义手臂关节名称表达式，匹配所有相关手臂关节
            joint_names_expr=[
                ".*_shoulder_pitch_joint",  # 肩部俯仰关节
                ".*_shoulder_roll_joint",   # 肩部滚动关节
                ".*_shoulder_yaw_joint",    # 肩部偏航关节
                ".*_elbow_joint",           # 肘关节
                ".*_wrist_roll_joint",      # 腕部滚动关节
                ".*_wrist_pitch_joint",     # 腕部俯仰关节
                ".*_wrist_yaw_joint",       # 腕部偏航关节
            ],
            # 定义手臂执行器的力矩限制
            effort_limit_sim={
                ".*_shoulder_pitch_joint": 400,  # 肩部俯仰关节力矩限制为400 Nm
                ".*_shoulder_roll_joint": 400,   # 肩部滚动关节力矩限制为400 Nm
                ".*_shoulder_yaw_joint": 400,    # 肩部偏航关节力矩限制为400 Nm
                ".*_elbow_joint": 400,           # 肘关节力矩限制为400 Nm
                ".*_wrist_roll_joint": 400,      # 腕部滚动关节力矩限制为400 Nm
                ".*_wrist_pitch_joint": 400,      # 腕部俯仰关节力矩限制为5.0 Nm
                ".*_wrist_yaw_joint": 400,        # 腕部偏航关节力矩限制为5.0 Nm
            },
            # 定义手臂执行器的速度限制
            velocity_limit_sim={
                ".*_shoulder_pitch_joint": 100.0,  # 肩部俯仰关节速度限制为100.0 rad/s
                ".*_shoulder_roll_joint": 100.0,   # 肩部滚动关节速度限制为100.0 rad/s
                ".*_shoulder_yaw_joint": 100.0,    # 肩部偏航关节速度限制为100.0 rad/s
                ".*_elbow_joint": 100.0,           # 肘关节速度限制为100.0 rad/s
                ".*_wrist_roll_joint": 100.0,      # 腕部滚动关节速度限制为100.0 rad/s
                ".*_wrist_pitch_joint": 100.0,     # 腕部俯仰关节速度限制为22.0 rad/s
                ".*_wrist_yaw_joint": 100.0,       # 腕部偏航关节速度限制为22.0 rad/s
            },
            # 定义手臂执行器的刚度系数
            stiffness={
                ".*_shoulder_pitch_joint": 100,  # 肩部俯仰关节使用5020型号刚度
                ".*_shoulder_roll_joint": 100,   # 肩部滚动关节使用5020型号刚度
                ".*_shoulder_yaw_joint": 100,    # 肩部偏航关节使用5020型号刚度
                ".*_elbow_joint": 100,           # 肘关节使用5020型号刚度
                ".*_wrist_roll_joint": 100,      # 腕部滚动关节使用5020型号刚度
                ".*_wrist_pitch_joint": 100,     # 腕部俯仰关节使用4010型号刚度
                ".*_wrist_yaw_joint": 100,       # 腕部偏航关节使用4010型号刚度
            },
            # 定义手臂执行器的阻尼系数
            damping={
                ".*_shoulder_pitch_joint": 6,    # 肩部俯仰关节使用5020型号阻尼
                ".*_shoulder_roll_joint": 6,     # 肩部滚动关节使用5020型号阻尼
                ".*_shoulder_yaw_joint": 6,      # 肩部偏航关节使用5020型号阻尼
                ".*_elbow_joint": 6,             # 肘关节使用5020型号阻尼
                ".*_wrist_roll_joint": 6,        # 腕部滚动关节使用5020型号阻尼
                ".*_wrist_pitch_joint": 6,       # 腕部俯仰关节使用4010型号阻尼
                ".*_wrist_yaw_joint": 6,         # 腕部偏航关节使用4010型号阻尼
            },
            # 定义手臂执行器的转动惯量
            armature={
                ".*_shoulder_pitch_joint": ARMATURE_5020,   # 肩部俯仰关节使用5020型号转动惯量
                ".*_shoulder_roll_joint": ARMATURE_5020,    # 肩部滚动关节使用5020型号转动惯量
                ".*_shoulder_yaw_joint": ARMATURE_5020,     # 肩部偏航关节使用5020型号转动惯量
                ".*_elbow_joint": ARMATURE_5020,            # 肘关节使用5020型号转动惯量
                ".*_wrist_roll_joint": ARMATURE_5020,       # 腕部滚动关节使用5020型号转动惯量
                ".*_wrist_pitch_joint": ARMATURE_4010,      # 腕部俯仰关节使用4010型号转动惯量
                ".*_wrist_yaw_joint": ARMATURE_4010,        # 腕部偏航关节使用4010型号转动惯量
            },
        ),
    },
)

# 为Avada机器人创建动作缩放字典，用于将标准化动作映射到实际力矩输出
Avada_ACTION_SCALE = {}
# 遍历所有执行器配置
for a in Avada_CYLINDER_CFG.actuators.values():
    # 获取执行器的力矩限制
    e = a.effort_limit_sim
    # 获取执行器的刚度系数
    s = a.stiffness
    # 获取执行器关联的关节名称表达式
    names = a.joint_names_expr
    # 如果力矩限制不是字典形式，则将其转换为字典（每个关节使用相同的力矩限制）
    if not isinstance(e, dict):
        e = {n: e for n in names}
    # 如果刚度系数不是字典形式，则将其转换为字典（每个关节使用相同的刚度）
    if not isinstance(s, dict):
        s = {n: s for n in names}
    # 对于每个关节名称
    for n in names:
        # 如果该关节在力矩限制和刚度字典中都存在且刚度非零
        if n in e and n in s and s[n]:
            # 计算动作缩放因子，使用公式 0.25 * 力矩限制 / 刚度
            Avada_ACTION_SCALE[n] = 0.25 * e[n] / s[n]