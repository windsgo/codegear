import math

'''
渐开线函数和反渐开线函数
引用时直接：from invfun import inv,inv_1
'''

def inv(alphar):  # 渐开线函数，输入为弧度值
    return math.tan(alphar) - alphar


def inv_1(inva, alpha0=math.radians(20), err=1e-3):  # 输出为弧度制
    def __f(x):
        return math.tan(x) - x - inva

    def __f1(x):
        return 1/(math.cos(x)**2) - 1

    x = [alpha0]
    err_e = abs(x[0])
    n = 0
    while(err_e > err):
        x.append(x[-1] - __f(x[-1]) / __f1(x[-1]))
        err_e = abs(x[-1] - x[-2])
        n += 1
        if n > 5e6:
            raise ValueError('反渐开线函数迭代失败，请检查输入值')

    return x[-1]