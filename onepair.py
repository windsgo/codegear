import math
from invfun import inv, inv_1
import os
import sys

class PrintToTxt:  # with HiddenPrints(self.hidelog):
    def __init__(self, filename='Default.txt', activated=True):
        # activated参数表示当前修饰类是否被激活
        self.filename = filename
        self.activated = activated
        self.original_stdout = None

    def open(self):
        sys.stdout.close()
        sys.stdout = self.original_stdout

    def close(self):
        self.original_stdout = sys.stdout
        sys.stdout = open(self.filename, 'w',encoding=sys.getfilesystemencoding())

    def __enter__(self):
        if self.activated:
            self.close()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.activated:
            self.open()

class HiddenPrints:  # with HiddenPrints(self.hidelog):
    def __init__(self, activated=True):
        # activated参数表示当前修饰类是否被激活
        self.activated = activated
        self.original_stdout = None

    def open(self):
        sys.stdout.close()
        sys.stdout = self.original_stdout

    def close(self):
        self.original_stdout = sys.stdout
        sys.stdout = open(os.devnull, 'w')
        # 这里的os.devnull实际上就是Linux系统中的“/dev/null”
        # /dev/null会使得发送到此目标的所有数据无效化，就像“被删除”一样
        # 这里使用/dev/null对sys.stdout输出流进行重定向

    def __enter__(self):
        if self.activated:
            self.close()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.activated:
            self.open()


'''
齿轮和行星齿轮
'''


class BasicGears(object):  # 齿轮设计基础参数
    def __init__(self, alpha, h_above_std, c_std, z0=25, x0=0, ha0_std=1.3):
        self.alpha = math.radians(alpha)
        self.h_above_std = h_above_std
        self.c_std = c_std
        self.zmin = math.floor(
            2 * h_above_std / math.sin(math.radians(alpha))**2)
        # 插齿刀参数
        self.z0 = z0
        self.x0 = x0
        self.ha0_std = ha0_std


class Gear(object):  # 单对齿轮啮合副 已知x1，x2
    def __init__(self, m, z1, z2, x1=0, x2=0, types='wai', basicgears=BasicGears(20, 1, 0.25), hidelog=False):  # wai / nei
        self.hidelog = hidelog
        self.types = types
        self.alpha = basicgears.alpha
        self.h_above_std = basicgears.h_above_std
        self.c_std = basicgears.c_std
        self.z0 = basicgears.z0
        self.x0 = basicgears.x0
        self.ha0_std = basicgears.ha0_std

        self.m = m
        self.z1 = z1  # 小齿轮
        self.z2 = z2
        self.d1 = m*z1
        self.d2 = m*z2
        self.db1 = math.cos(self.alpha)*self.d1
        self.db2 = math.cos(self.alpha)*self.d2
        self.x1 = x1
        self.x2 = x2
        self.u = z2/z1
        if types == 'wai':
            self.x_segma = x2 + x1
            self.z_segma = z2 + z1
            self.a = (self.d1+self.d2)/2  # 标准中心距
            # print('外啮合')
        else:  # nei
            self.x_segma = x2 - x1
            self.z_segma = z2 - z1
            self.a = (self.d2-self.d1)/2  # 标准中心距
            # print('内啮合，内齿轮插齿刀参数：z0=%d,x0=%d,ha0_std=%f.' %
            #       (self.z0, self.x0, self.ha0_std))
        self.alpha_work = inv_1(
            2*self.x_segma*math.tan(self.alpha)/self.z_segma+inv(self.alpha))
        self.a_work = math.cos(self.alpha)*self.a / \
            math.cos(self.alpha_work)  # 实际中心距
        self.y = (self.a_work - self.a)/m
        self.delta_y = self.x_segma - self.y

    def calc(self):  # 计算，外部调用
        if self.types == 'wai':
            print('外啮合')
        else:
            print('内啮合，内齿轮插齿刀参数：z0=%d, x0=%d, ha0_std=%f.' %
                  (self.z0, self.x0, self.ha0_std))
        print('m=%d, z1=%d, z2=%d, x1=%f, x2=%f.' %
              (self.m, self.z1, self.z2, self.x1, self.x2))
        self.__GetDiameterAbove()
        self.__GetAlphaAbove()
        self.__GetYetaMax()
        self.__GetContactRatio()

    def examine(self, k=0.25):  # 检验类函数，外部调用
        self.__ExamineBottomCut()
        self.__ExamineTopCut()
        self.__ExamineTransitionCurveInterference()
        self.__ExamineToothProfileInterference()
        self.__ExamineToothTipThickness(k)

        self.examineResult = bool(self.BottomCut[0]*self.TopCut[0] * self.TransitionCurveInterference[0]
                                  * self.ToothProfileInterference[0] * self.ToothTipThickness[0])
        return self.examineResult

    def revise(self, da1_safe):  # 用于改正行星轮齿顶圆直径和齿顶压力角（选择较小的安全直径），重新计算用到齿顶压力角的性能参数，外部调用，传参：小齿轮安全齿顶圆直径
        self.da1 = da1_safe
        self.alpha_a1 = math.acos(self.db1/self.da1)
        self.__GetYetaMax()
        self.__GetContactRatio()

    def __GetDiameterAbove(self, select=0, cuttype='gun'):
        alpha = self.alpha
        # alpha_work = self.alpha_work
        ha_std = self.h_above_std
        c_std = self.c_std

        if cuttype == 'cha':  # 插齿
            pass

        elif cuttype == 'gun':  # 滚齿法（对于内齿轮仍然采用插齿计算）
            hf1 = (ha_std + c_std - self.x1)*self.m  # 小齿轮齿根高
            self.df1 = df1 = self.d1 - 2*hf1  # 小齿轮齿根圆直径
            if self.types == 'wai':  # 外啮合计算
                ha1 = (ha_std + self.x1 - self.delta_y)*self.m
                self.da1 = da1 = self.d1 + 2*ha1
                ha2 = (ha_std + self.x2 - self.delta_y)*self.m
                self.da2 = da2 = self.d2 + 2*ha2
            elif self.types == 'nei':  # 内啮合计算: 小齿轮滚齿，大齿轮插齿
                self.da2 = da2 = df1 + 2*self.a_work + 2*c_std*self.m

                self.da0 = da0 = self.m * \
                    (self.z0 + 2*self.ha0_std + 2*self.x0)  # 插齿刀齿顶圆直径
                self.alpha_work0 = alpha_work0 = inv_1(inv(
                    alpha) + 2*(self.x2-self.x0)*math.tan(alpha)/(self.z2-self.z0))  # 插内齿啮合角
                self.a_work0 = a_work0 = 0.5*self.m * \
                    (self.z2-self.z0)*math.cos(alpha) / \
                    math.cos(alpha_work0)  # 插内齿中心距

                self.df2 = df2 = da0 + 2*a_work0  # 大（内）齿轮齿根圆直径
                self.da1 = da1 = df2 - 2*self.a_work - 2*c_std*self.m

                da2 = 2*self.a_work + df1 + 2*c_std*self.m

        print('齿顶圆直径:\t', [da1, da2])

    def __GetAlphaAbove(self, select=0):  # 计算select的齿顶圆压力角
        # self.__GetDiameterAbove()
        self.alpha_a1 = alpha_a1 = math.acos(self.db1/self.da1)
        self.alpha_a2 = alpha_a2 = math.acos(self.db2/self.da2)
        if self.types == 'nei':
            self.alpha_a0 = math.acos(
                self.m*self.z0*math.cos(self.alpha)/self.da0)  # 插齿刀齿顶压力角
            self.ta0 = math.tan(self.alpha_a0)  # 插齿刀齿顶压力角正切值
        print('齿顶圆压力角:\t', [alpha_a1, alpha_a2])

    def __GetYetaMax(self, select=0):  # 计算select的齿轮的最大滑动率
        u = self.u  # 两齿传动比
        self.taw = taw = math.tan(self.alpha_work)  # 啮合角正切值
        self.ta1 = ta1 = math.tan(self.alpha_a1)  # 齿顶圆压力角正切值
        self.ta2 = ta2 = math.tan(self.alpha_a2)

        if self.types == 'wai':
            self.yeta1 = (1 + u) * (ta2 - taw)/((1 + u) * taw - u * ta2)
            self.yeta2 = (1 + u) * (ta1 - taw)/((1 + u) * taw - ta1)
        elif self.types == 'nei':
            self.yeta1 = (1 - u) * (ta2 - taw)/((1 - u) * taw + u * ta2)
            self.yeta2 = (u - 1) * (ta1 - taw)/((u - 1) * taw + ta1)

        print('最大滑动率:\t', [self.yeta1, self.yeta2])

    def __GetContactRatio(self, select=0):  # 重合度
        flag = 1 if self.types == 'wai' else -1  # wai:1 nei:-1
        self.ContactRatio = 0.5/math.pi * \
            (self.z1*(self.ta1-self.taw) + flag*self.z2*(self.ta2-self.taw))
        print('重合度:\t', self.ContactRatio)

    '''
    校验类函数：指标符合返回值为1，不符合返回值为0，无须检验类返回值为-1
    返回一个列表 0位：总体符合情况
    1位：小齿轮符合情况
    2位：大齿轮符合情况
    同名值存储信息，如：self.TopCut = [0,-1,0]表示内啮合时大齿轮（内齿轮）产生泛成顶切
    '''

    def __ExamineBottomCut(self):
        ha_std = self.h_above_std
        alpha = self.alpha
        zmin = 2*ha_std/(math.sin(alpha)**2)
        x1min = ha_std*(zmin-self.z1)/zmin
        x2min = ha_std*(zmin-self.z2)/zmin
        BottomCut = [-1, int(self.x1 >= x1min), int(self.x2 >= x2min)]
        BottomCut[0] = BottomCut[1]*BottomCut[2]
        self.BottomCut = BottomCut
        print('BottomCut:\t', BottomCut)
        return BottomCut

    def __ExamineTopCut(self):  # 校验内齿轮加工时，是否产生泛成顶切
        if self.types == 'wai':
            self.TopCut = TopCut = [-1, -1, -1]
        else:  # 'nei'
            l = self.z0/self.z2
            r = 1 - self.ta2/math.tan(self.alpha_work0)
            self.TopCut = TopCut = [1, -1, 1] if l >= r else [0, -1, 0]
            with HiddenPrints(self.hidelog):
                print('l,r:\t', [l, r])
        print('Topcut:\t', TopCut)
        return TopCut

    def __ExamineTransitionCurveInterference(self):  # 校验过度曲线干涉
        z1 = self.z1
        z2 = self.z2
        alpha = self.alpha
        taw = self.taw
        ta1 = self.ta1
        ta2 = self.ta2
        ha_std = self.h_above_std
        # 小齿轮滚刀加工，公式一致

        l1 = z2*ta2 - (z2 - z1)*taw
        r1 = z1*math.tan(alpha) - 4*(ha_std - self.x1)/math.sin(2*alpha)

        # 大齿轮分滚齿or插齿（内齿轮）
        if self.types == 'wai':
            l2 = z1*ta1 - (z1 - z2)*taw
            r2 = z2*math.tan(alpha) - 4*(ha_std - self.x2)/math.sin(2*alpha)
        else:  # 'nei'
            l2 = self.z0*self.ta0 + (z2-self.z0)*math.tan(self.alpha_work0)
            r2 = z1*ta1 + (z2-z1)*taw
        with HiddenPrints(self.hidelog):
            print('l1,r1,l2,r2:\t', [l1, r1, l2, r2])
        TransitionCurveInterference = [0, 0, 0]
        TransitionCurveInterference[1] = int(l1 >= r1)
        TransitionCurveInterference[2] = int(l2 >= r2)
        TransitionCurveInterference[0] = TransitionCurveInterference[1] * \
            TransitionCurveInterference[2]
        TransitionCurveInterference.append([l1-r1])
        self.TransitionCurveInterference = TransitionCurveInterference
        print('TransitionCurveInterference:\t', TransitionCurveInterference)
        return TransitionCurveInterference

    def __ExamineToothProfileInterference(self):  # 校验齿廓重叠干涉 (内啮合需校核)
        if self.types == 'wai':
            self.ToothProfileInterference = ToothProfileInterference = [
                -1, -1, -1]
        else:  # 'nei'
            ra1 = 0.5 * self.da1
            ra2 = 0.5 * self.da2
            aw = self.a_work
            delta1 = math.acos((ra2**2 - ra1**2 - aw**2) / (2*ra1*aw))
            delta2 = math.acos((ra2**2 - ra1**2 + aw**2) / (2*ra2*aw))
            l = self.z1*(delta1+inv(self.alpha_a1)) - self.z2*(delta2 +
                                                               inv(self.alpha_a2)) + inv(self.alpha_work)*(self.z2-self.z1)
            r = 0
            with HiddenPrints(self.hidelog):
                print('l,r:', [l, r])
            self.ToothProfileInterference = ToothProfileInterference = [
                1, 1, 1] if l >= r else [0, 0, 0]
        print('ToothProfileInterference:\t', ToothProfileInterference)

    def __ExamineToothTipThickness(self, k=0.25):  # 齿顶厚验算
        def __sa(da, x, z, alpha_a):
            return da*((math.pi + 4*x*math.tan(self.alpha))/(2*z) + inv(self.alpha) - inv(alpha_a))
        r = k * self.m
        self.sa1 = l1 = __sa(self.da1, self.x1, self.z1, self.alpha_a1)
        ret1 = int(l1 >= r)
        if self.types == 'wai':
            self.sa2 = l2 = __sa(self.da2, self.x2, self.z2, self.alpha_a2)
            ret2 = int(l2 >= r)
            ret0 = ret1 * ret2
        else:  # 'nei'
            self.sa2 = l2 = -1
            ret2 = -1  # 内齿轮无须校验
            ret0 = ret1
        self.ToothTipThickness = ToothTipThickness = [ret0, ret1, ret2]
        with HiddenPrints(self.hidelog):
            print('l1,l2,r:\t', [l1, l2, r])
        print('ToothTipThickness:\t', ToothTipThickness)

    def Printitem(self):  # 打印所有参数表
        print('\n'.join(['%s\t%s' % item for item in self.__dict__.items()]))


class PlanetaryGear(object):
    def __init__(self, m, za, zc, zb, xa, xc, xb, basicgears=BasicGears(20, 1, 0.25), hidelog=False):
        self.m = m
        self.za = za
        self.zc = zc
        self.zb = zb
        self.xa = xa
        self.xc = xc
        self.xb = xb
        self.gear_ac = Gear(m, za, zc, xa, xc, 'wai', basicgears, hidelog)
        self.gear_cb = Gear(m, zc, zb, xc, xb, 'nei', basicgears, hidelog)

    def calc(self):
        print('---------------Planetary calc-------------------')
        print('>>Starting calculate...')
        self.gear_ac.calc()
        print('- - - - - - - - - - - - - - - - - - - - - - - - -')
        self.gear_cb.calc()
        print('===============Planetary calced=================\n')

    def revise(self):
        print('---------------Planetary revise-----------------')
        if self.gear_ac.da2 > self.gear_cb.da1:
            print('修改外啮合...')
            self.gear_ac.revise(self.gear_cb.da1)
            print('已修改-外-啮合副的行星轮齿顶圆直径')
        elif self.gear_ac.da2 < self.gear_cb.da1:
            print('修改内啮合...')
            self.gear_cb.revise(self.gear_ac.da2)
            print('已修改-内-啮合副的行星轮齿顶圆直径')

        else:
            print('行星轮齿顶圆一致')
            pass
        print('daa,dac,dab:\t', [self.gear_ac.da1,
                                 self.gear_ac.da2, self.gear_cb.da2])

        print('===============Planetary revised===============\n')

    def examine(self, kac=0.25, kcb=0.25):
        print('---------------Planetary examine---------------')
        print('外啮合:')
        r1 = self.gear_ac.examine(kac)
        print('- - - - - - - - - - - - - - - - - - - - - - - - -')
        print('内啮合:')
        r2 = self.gear_cb.examine(kcb)
        self.examineResult = r1 & r2
        print('===============Planetary examined==============\n')
        return(self.examineResult)

    def start(self, examine_kac=0.25, examine_kcb=0.25):
        self.calc()
        self.revise()
        return self.examine(examine_kac, examine_kcb)


if __name__ == '__main__':
    basegear = BasicGears(20, 1, 0.25)
    gear = Gear(3, 42, 45, 0.5, 1.007, 'nei', basegear, hidelog=False)
    gear.calc()
    gear.examine()
    # gear.Printitem()

    # print(gear.GetDiameterAbove(gear))
