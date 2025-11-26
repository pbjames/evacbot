def clamp(value: float, min_v: float, max_v: float):
    return min(max_v, max(value, min_v))


def print_help():
    print(
        """
You can control the drone with your computer keyboard:
    k:     move forward
    j:     move backward
    h:     turn right
    l:     turn left
    up:    increase the target altitude
    down:  decrease the target altitude
    right: strafe right
    left:  strafe left
"""
    )
