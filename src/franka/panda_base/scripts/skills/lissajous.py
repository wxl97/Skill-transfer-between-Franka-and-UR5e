import math

def lissajous(x0, amplitude_multi, A_x, freq_x, t, phase):
    """
        计算李萨如曲线
    """
    return x0 + amplitude_multi * A_x * math.sin(2 * math.pi * freq_x * t + phase)
