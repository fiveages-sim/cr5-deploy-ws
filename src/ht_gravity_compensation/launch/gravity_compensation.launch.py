"""Self-contained gravity compensation launch for Panthera HT.

Starts robot_state_publisher + ros2_control_node with this package's controller
configuration, then spawns the gravity compensation controller.

No existing package is modified: the URDF is expanded from the description
package xacro at launch time and all controller parameters come from this
package (config + launch-generated overrides).

Arm joint names / gripper joint / max torque are parsed from the description
package's xacro/ros2_control/interfaces.xacro (no hard-coded joint lists).

Usage:
  # mock hardware (no motors, for validation)
  ros2 launch ht_gravity_compensation gravity_compensation.launch.py

  # real Panthera HT dual arm
  ros2 launch ht_gravity_compensation gravity_compensation.launch.py hardware:=real

  # single / left / right arm (controller joints are derived from `type`)
  ros2 launch ht_gravity_compensation gravity_compensation.launch.py type:=single hardware:=real

  # optional RViz (default off) and custom namespace
  ros2 launch ht_gravity_compensation gravity_compensation.launch.py rviz:=true namespace:=my_ns
"""

import os
import tempfile
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_DESCRIPTION_PACKAGE = "panthera_ht_description"

# interfaces.xacro 解析失败时的回退值（关节名均为裸名，无 left_/right_ 前缀）
FALLBACK_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
FALLBACK_GRIPPER_JOINT = "gripper_joint"
FALLBACK_ARM_MAX_EFFORT = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]

# 悬停软刚度（真机不再导出 kp/kd 命令接口时 use_pd 自动回退，此处为硬参数模式保留）
HOLD_KP = [0.01]
HOLD_KD = [0.1]


def _parse_arm_interfaces(description_package):
    """从描述包 xacro 解析关节信息与力矩限幅。

    - interfaces.xacro：关节名（${prefix} 模板名）
    - robot.xacro：hardware 段 <param name="max_torques"> CSV（每臂 7 值，取前 6 个臂关节）

    Returns:
        (arm_joints, gripper_joint, arm_max_efforts)：均为裸名 / 裸值
        （无 left_/right_ 前缀；前缀由 _joints_for_type 按 type 生成）。
    """
    base_dir = os.path.join(
        get_package_share_directory(description_package), "xacro", "ros2_control")
    try:
        interfaces_file = os.path.join(base_dir, "interfaces.xacro")
        tree = ET.parse(interfaces_file)
    except (OSError, ET.ParseError, Exception) as e:
        print(f"[WARN] Failed to parse interfaces.xacro from '{description_package}': "
              f"{e}; using fallback joint lists")
        return (FALLBACK_ARM_JOINTS, FALLBACK_GRIPPER_JOINT, FALLBACK_ARM_MAX_EFFORT)

    joints_in_order = []
    for joint_el in tree.getroot().iter("joint"):
        name = joint_el.get("name")
        if not name:
            continue
        # xacro 宏模板名 "${prefix}joint1" → 去掉前缀模板得到裸名
        name = name.replace("${prefix}", "").strip()
        if not name:
            continue
        joints_in_order.append(name)

    # max_torques 已从 interfaces.xacro 移至 robot.xacro hardware 段（CSV，每臂 7 值）
    max_efforts_default = list(FALLBACK_ARM_MAX_EFFORT)
    try:
        robot_file = os.path.join(base_dir, "robot.xacro")
        robot_tree = ET.parse(robot_file)
        for param in robot_tree.getroot().iter("param"):
            if param.get("name") == "max_torques" and param.text:
                values = [float(v.strip()) for v in param.text.split(",") if v.strip()]
                if len(values) >= 6:
                    max_efforts_default = values[:6]
                break
    except (OSError, ET.ParseError, ValueError, Exception) as e:
        print(f"[WARN] Failed to parse max_torques from robot.xacro: "
              f"{e}; using fallback {FALLBACK_ARM_MAX_EFFORT}")

    arm_joints = []
    max_efforts = []
    gripper_joint = None
    for name in joints_in_order:
        if "gripper" in name:
            if gripper_joint is None:
                gripper_joint = name
        else:
            arm_joints.append(name)
            max_efforts.append(
                max_efforts_default[len(arm_joints) - 1]
                if len(arm_joints) - 1 < len(max_efforts_default) else 10.0)

    if not arm_joints or gripper_joint is None:
        print("[WARN] interfaces.xacro parse yielded no arm joints; using fallback")
        return (FALLBACK_ARM_JOINTS, FALLBACK_GRIPPER_JOINT, FALLBACK_ARM_MAX_EFFORT)

    print(f"[INFO] Parsed from {interfaces_file} (+robot.xacro max_torques): "
          f"arm joints={arm_joints} gripper={gripper_joint} max_effort={max_efforts}")
    return arm_joints, gripper_joint, max_efforts


def _joints_for_type(robot_type, arm_joints, gripper_joint):
    """按臂组合类型生成带前缀的关节列表。"""
    if robot_type == "dual":
        joints = [p + j for p in ("left_", "right_") for j in arm_joints]
        holds = ["left_" + gripper_joint, "right_" + gripper_joint]
    else:
        prefix = {"single": "", "left": "left_", "right": "right_"}.get(robot_type, "")
        joints = [prefix + j for j in arm_joints]
        holds = [prefix + gripper_joint]
    return joints, holds


def _prefixed_config(yaml_path, namespace, controller_overrides=None):
    """读取 yaml，把顶层节点键加上 namespace 前缀，并合并控制器覆盖参数。

    launch_ros 对 yaml 文件按节点全名匹配顶层键（如 `drag_controller/controller_manager`），
    匹配成功后剥离键前缀、把参数以裸名传给节点——controller_manager 源码正是以
    `gravity_compensation_controller.type`、`update_rate` 等裸名读取。
    注意：不能直接传 dict，dict 的键会原样成为参数名（带前缀，CM 读不到）。

    该文件同时会被 CM 作为控制器节点的 --params-file 加载（见 CM 日志 node arguments），
    因此把控制器覆盖参数（joints/hold_joints/use_pd/robot_description 等）合并进
    `gravity_compensation_controller` 键，控制器 configure 时就能从自身参数读到。
    """
    with open(yaml_path) as f:
        data = yaml.safe_load(f) or {}
    if namespace:
        prefixed = {}
        for node_key, node_params in data.items():
            key = node_key.lstrip("/")
            if key.startswith(namespace + "/"):
                prefixed[node_key] = node_params
            else:
                prefixed[f"{namespace}/{key}"] = node_params
        data = prefixed
    if controller_overrides:
        ctrl_key = (
            f"{namespace}/gravity_compensation_controller"
            if namespace else "gravity_compensation_controller")
        ctrl_node = data.setdefault(ctrl_key, {})
        ctrl_params = ctrl_node.setdefault("ros__parameters", {})
        ctrl_params.update(controller_overrides)
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="ht_gravity_compensation_cm_")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, allow_unicode=True)
    return path


def _write_controller_params_file(joints, holds, use_pd, max_effort, hold_kp, hold_kd, urdf, namespace=""):
    """生成控制器覆盖参数 dict（不再写独立文件）。

    覆盖参数直接合并进 CM 配置 yaml（_prefixed_config 的 controller_overrides），
    因为 Jazzy spawner 的 -p 文件不会出现在控制器节点参数中（实测）。
    """
    params = {
        "joints": joints,
        "hold_joints": holds,
        "use_pd": use_pd,
        "robot_description": urdf,
    }
    if use_pd:
        params["hold_kp"] = hold_kp
        params["hold_kd"] = hold_kd
        params["max_effort"] = max_effort
    return params


def launch_setup(context, *args, **kwargs):
    robot = context.launch_configurations["robot"]
    robot_type = context.launch_configurations["type"]
    hardware = context.launch_configurations["hardware"]
    control_mode = context.launch_configurations["control_mode"]
    use_sim_time = context.launch_configurations["use_sim_time"] == "true"
    use_pd_arg = context.launch_configurations["use_pd"]
    namespace = context.launch_configurations["namespace"]
    # 空串 / "/" 都表示全局命名空间（launch CLI 无法传真正空串，用 "/" 即可）
    if namespace in ("", "/"):
        namespace = ""
    description_package = context.launch_configurations["description_package"]
    rviz_enabled = context.launch_configurations["rviz"] == "true"

    # Real ht_ros2_control hardware exposes kp/kd command interfaces; mock does not.
    use_pd = (use_pd_arg == "auto" and hardware == "real") or use_pd_arg == "true"

    # 关节名 / 夹爪名 / 力矩限幅从描述包 interfaces.xacro 解析（支持 description_package 参数）
    arm_joints, gripper_joint, arm_max_efforts = _parse_arm_interfaces(description_package)

    # Expand the ros2_control URDF from the description package (read-only reuse).
    xacro_file = os.path.join(
        get_package_share_directory(description_package),
        "xacro", "ros2_control", "robot.xacro")
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "type": robot_type,
            "ros2_control_hardware_type": hardware,
            "control_mode": control_mode,
        },
    )
    urdf = doc.toprettyxml(indent="  ")

    joints, holds = _joints_for_type(robot_type, arm_joints, gripper_joint)
    arm_count = 2 if robot_type == "dual" else 1

    nodes = []
    nodes.append(LogInfo(
        msg=f"[ht_gravity_compensation] robot={robot} type={robot_type} "
            f"hardware={hardware} control_mode={control_mode} use_pd={use_pd} "
            f"namespace='{namespace}' rviz={rviz_enabled} "
            f"description_package={description_package}"))
    nodes.append(LogInfo(
        msg=f"[ht_gravity_compensation] joints={joints} hold_joints={holds} "
            f"max_effort={arm_max_efforts}"))

    # robot_state_publisher publishes TF and /<ns>/robot_description from the same URDF.
    # TransformBroadcaster 硬编码发布到绝对名 "/tf" 和 "/tf_static"（不受节点 namespace 影响），
    # 因此显式 remap 到 "/<ns>/tf" 绝对名（注意：目标必须是绝对路径，否则会再叠加节点 ns）。
    rsp_remappings = []
    if namespace:
        rsp_remappings = [
            ("/tf", f"/{namespace}/tf"),
            ("/tf_static", f"/{namespace}/tf_static"),
        ]
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{
            "robot_description": urdf,
            "publish_frequency": 100.0,
            "use_tf_static": True,
            "use_sim_time": use_sim_time,
        }],
        remappings=rsp_remappings,
    ))

    # ros2_control_node: controller configuration from THIS package.
    # 配置 yaml 的顶层键需带 namespace 前缀才能匹配 ns 下的 CM 节点；
    # 控制器覆盖参数（joints/use_pd/robot_description 等）合并进同一个 yaml，
    # 因为该文件会同时作为控制器节点的 --params-file 加载。
    # robot_description 不通过 CM 参数传入——CM 默认订阅相对话题 "robot_description"
    # （RSP 在 ns 内发布同名相对话题，自动匹配）。
    config_yaml = os.path.join(
        get_package_share_directory("ht_gravity_compensation"),
        "config", "gravity_compensation.yaml")
    controller_params = _write_controller_params_file(
        joints=joints,
        holds=holds,
        use_pd=use_pd,
        max_effort=arm_max_efforts * arm_count if use_pd else [],
        hold_kp=HOLD_KP * len(joints) if use_pd else [],
        hold_kd=HOLD_KD * len(joints) if use_pd else [],
        urdf=urdf,
        namespace=namespace,
    )
    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            _prefixed_config(config_yaml, namespace, controller_params),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    ))

    # spawner 的 controller manager 绝对路径（namespace 下为 /<ns>/controller_manager）
    controller_manager_name = (
        f"{namespace}/controller_manager" if namespace else "/controller_manager")

    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_name],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    ))

    # 控制器 spawner：参数已通过 CM 配置 yaml 注入，无需 -p 文件。
    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gravity_compensation_controller",
            "--controller-manager", controller_manager_name,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    ))

    # Optional RViz (default off). Runs inside the same namespace so relative
    # topic names in the config ("tf", "robot_description") follow the robot.
    if rviz_enabled:
        rviz_config = os.path.join(
            get_package_share_directory("ht_gravity_compensation"),
            "config", "gravity_compensation.rviz")
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=namespace,
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="panthera_ht",
                              description="Robot name (informational only)"),
        DeclareLaunchArgument("description_package", default_value=DEFAULT_DESCRIPTION_PACKAGE,
                              description="Robot description package providing "
                                          "xacro/ros2_control/robot.xacro and interfaces.xacro"),
        DeclareLaunchArgument("type", default_value="dual",
                              description="Arm composition: single | left | right | dual",
                              choices=["single", "left", "right", "dual"]),
        DeclareLaunchArgument("hardware", default_value="mock_components",
                              description="real | mock_components | gz | isaac",
                              choices=["real", "mock_components", "gz", "isaac"]),
        DeclareLaunchArgument("control_mode", default_value="full_control",
                              description="Hardware interface control mode (full_control | pd_control)",
                              choices=["full_control", "pd_control"]),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_pd", default_value="auto",
                              description="Write kp/kd command interfaces: auto | true | false"),
        DeclareLaunchArgument("namespace", default_value="drag_controller",
                              description="ROS namespace for robot_state_publisher, "
                                          "controller_manager and RViz (use '/' for global)"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Launch RViz2 (true|false)"),
        OpaqueFunction(function=launch_setup),
    ])
