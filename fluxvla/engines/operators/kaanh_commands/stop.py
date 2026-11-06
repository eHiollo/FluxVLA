"""Stop local action sending and explicitly confirm follower/motion shutdown."""


def configure_parser(parser):
    """stop takes no arguments."""


def execute(robot, args, output):
    output('本地动作下发已取消，正在请求退出 follower 并停止控制器当前运动命令……')
    try:
        state = robot.stop_all_actions()
    except Exception as exc:
        raise RuntimeError(f'本地下发已取消，但未确认机器人停止：{exc}。请在现场控制器确认停止。') from exc
    output('stop 完成：follower 已退出，Stopped 反馈已确认；控制台保留。')
    return state
