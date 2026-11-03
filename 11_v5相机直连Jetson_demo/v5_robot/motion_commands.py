"""V5 笛卡尔运动指令构造模块。

控制器在总线上把 BlendL 表示成 MoveBlend 容器里的一条 ``insert_line`` 记录，
因此每一次机械臂运动都必须是「一个 MoveBlend 容器 + 若干条 BlendL 记录」的组合。
本模块刻意不提供 MoveAbsJ（关节空间运动），避免在同一 demo 进程里混用两套运动规划器。

生成的指令字符串由机械臂网关原样下发给控制器执行。
"""
from __future__ import annotations

from collections.abc import Iterable, Sequence


def blend_l(target: Sequence[float], speed: str,
            zone: tuple[float, float] = (0.0, 0.0)) -> str:
    """构造一条 BlendL（直线插补）指令字符串。

    参数：
        target —— 7 个数值的 robottarget：[x, y, z, qx, qy, qz, qw]。
                  位置单位 m，姿态为四元数；具体坐标系由控制器侧的约定决定。
        speed  —— 速度档位名（控制器定义的速度枚举，如 "v25"），不能为空且不能含空白字符
        zone   —— 过渡区参数二元组 (zone1, zone2)，默认 (0.0, 0.0) 表示不启用过渡

    返回：
        形如 ``{MoveBlend --type=insert_line --robottarget_value={...} --zone={...} --speed=...}``
        的指令字符串。
    """
    # 统一转成 float 并锁定为 7 个分量，避免后续字符串格式化出错
    values = tuple(float(value) for value in target)
    # robottarget 必须是 [x, y, z, qx, qy, qz, qw] 共 7 个数值
    if len(values) != 7:
        raise ValueError(f"BlendL needs a 7-value robottarget, got {len(values)}")
    # 速度名会被原样拼进指令，含空白字符会破坏控制器侧的解析
    if not speed or any(ch.isspace() for ch in speed):
        raise ValueError(f"invalid BlendL speed: {speed!r}")
    # 位置分量保留 9 位小数，避免科学计数法或精度截断影响定位
    pose = ",".join(f"{value:.9f}" for value in values)
    return (
        "{MoveBlend --type=insert_line --robottarget_value={"
        + pose
        + f"}} --zone={{{float(zone[0]):.1f},{float(zone[1]):.1f}}} --speed={speed}}}"
    )


def move_blend(targets: Iterable[Sequence[float]], speed: str,
               zone: tuple[float, float] = (0.0, 0.0)) -> list[str]:
    """把一串目标点打包成一个完整的 MoveBlend 程序（内部只含 BlendL 段）。

    生成结构固定：``first_insert`` 开头 → 逐条 BlendL → ``start`` 启动执行。
    控制器按先后顺序执行全部 BlendL 段，因此入参的点位顺序就是机械臂的运动顺序。

    参数：
        targets —— 可迭代的 7 值 robottarget 序列（会被完整物化成列表）
        speed   —— 速度档位名，透传给每条 BlendL
        zone    —— 过渡区参数，透传给每条 BlendL

    返回：
        指令字符串列表，需要按顺序逐条下发给控制器。
    """
    # 先物化入参：既方便判空，也避免生成器被重复消费
    points = list(targets)
    # 一个目标点都没有时不该下发任何运动指令
    if not points:
        raise ValueError("MoveBlend needs at least one BlendL target")
    return [
        "{MoveBlend --type=first_insert}",
        *(blend_l(target, speed, zone) for target in points),
        "{MoveBlend --type=start}",
    ]
