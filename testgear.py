from invfun import inv, inv_1
from onepair import BasicGears, Gear, PlanetaryGear, HiddenPrints, PrintToTxt
import math
import numpy as np
# test revise

# z1:太阳轮 z2:内齿轮
def GetZ3(z1, z2, alpha_work_cb=20, alpha_work_ac_range=[23, 27]):
    if z2 <= z1:
        raise ValueError('太阳轮齿数过大')
    elif z2 % z1 != 0:
        raise ValueError('无法组成整数传动比')
    else:
        z3_std = (z2 - z1)/2
        if z3_std % 1 != 0:
            print('标准行星轮齿数不为整数,z3_std=%.2f' % z3_std)
        z3_std = int(z3_std)  # 取整
        z3_range_p = [(z3_std - zx) for zx in range(z3_std)]
        findflag = 0
        z3_group = []
        alpha_work_ac_group = []
        for z3x in z3_range_p:
            z_segma_ac = z1 + z3x
            z_segma_cb = z2 - z3x
            alpha_work_ac_tmp = math.degrees(
                math.acos(z_segma_ac/z_segma_cb*math.cos(math.radians(alpha_work_cb))))
            if (alpha_work_ac_tmp >= alpha_work_ac_range[0]) and (alpha_work_ac_tmp <= alpha_work_ac_range[1]):

                findflag = 1
                z3_group.append(z3x)
                alpha_work_ac_group.append(alpha_work_ac_tmp)
            else:
                if findflag == 1:
                    break

        return z3_group, alpha_work_ac_group


def ChooseX(m, z1, z3, z2, alpha_work_ac, alpha_work_cb=None, basegear=BasicGears(20, 1, 0.25)):
    if alpha_work_cb == None:
        alpha_work_cb = 20
    # x3_list_ori = np.arange(-1,2,0.02)
    x3_list_ori = [0]
    z_segma_ac = z1+z3
    z_segma_cb = z2-z3
    x_segma_ac = z_segma_ac*(inv(math.radians(alpha_work_ac)) -
                             inv(basegear.alpha))/(2*math.tan(basegear.alpha))
    x_segma_cb = z_segma_cb*(inv(math.radians(alpha_work_cb)) -
                             inv(basegear.alpha))/(2*math.tan(basegear.alpha))

    x_list = []
    for x3 in x3_list_ori:
        x1 = x_segma_ac - x3
        x2 = x_segma_cb + x3
        with HiddenPrints(False):
            pg = PlanetaryGear(m, z1, z3, z2, x1, x3, x2,
                               basegear, hidelog=False)
            ret = pg.start(0.25, 0.25)
        print(pg.gear_cb.TransitionCurveInterference)
        if ret:
            x_list.append([x1, x3, x2])
    print(z3, len(x_list))
    return pg


if __name__ == '__main__':
    basegear = BasicGears(20, 1, 0.25, 58, 0, 1.25)
    '''
    gear = Gear(3, 42, 45, 0.5, 1.007, 'nei', basegear,hidelog=True)
    gear.calc()
    gear.examine(0.25)
    '''

    hh = 0.001
    m = 1.75
    z1 = 18
    z2 = 108
    z3 = 44
    z_segma_13 = z1+z3
    z_segma_32 = z2-z3
    alpha = math.radians(20)
    with PrintToTxt('pg.txt',True):
        print('1:sun 3:planet 2:internal; D_Yeta:最大滑动率差; CRatio:重合度; A_w:啮合角')
        print('z1=%d,z3=%d,z2=%d,\tm=%f'%(z1,z3,z2,m))
        for x2 in np.arange(0.82, 0.84, hh):
            for x3 in np.arange(0.68, 0.70, hh):

                try:
                    x_segma_32 = x2-x3
                    alpha_work_32 = inv_1(
                        x_segma_32*2*math.tan(alpha)/z_segma_32+inv(alpha))
                    alpha_work_13 = math.acos(
                        z_segma_13/z_segma_32*math.cos(alpha_work_32))
                    x_segma_13 = z_segma_13 * \
                        (inv(alpha_work_13)-inv(alpha))/(2*math.tan(alpha))
                    x1 = x_segma_13 - x3
                    with HiddenPrints():
                        planet = PlanetaryGear(
                            m, z1, z3, z2, x1, x3, x2, basegear, hidelog=False)
                        ret = planet.start(0.25, 0.25)
                    if ret:
                        dy1 = abs(planet.gear_ac.yeta1-planet.gear_ac.yeta2)
                        dy2 = abs(planet.gear_cb.yeta1-planet.gear_cb.yeta2)
                        ep1 = planet.gear_ac.ContactRatio
                        ep2 = planet.gear_cb.ContactRatio

                        if (dy1 < 0.13 and dy2 < 0.13) and ep1 > 1.3 and ep2 > 1.3:
                            print('x1:%.4f,x3:%.3f,x2:%.3f,\tD_Yeta:%.4f,%.4f,\tCRatio:%.4f,%.4f\tA_w:%.3f,%.3f' % (
                                x1, x3, x2, dy1, dy2, ep1, ep2, math.degrees(planet.gear_ac.alpha_work), math.degrees(planet.gear_cb.alpha_work)))
                except:
                    pass

    # z1 = 18
    # z3_group,alpha_work_ac_group=GetZ3(z1,108,20,[23,27])
    # ChooseX(1.75,z1,z3_group[0],108,alpha_work_ac_group[0],20,basegear)
    # pg=ChooseX(1.75,18,45,108,20,20,basegear)
