package odin.detour.utils;

/**
 * Detour通用数学工具函数
 * 翻译自UE5 DetourCommon.h和DetourCommon.cpp
 * 
 * @author UE5NavMesh4J
 */
public final class DetourCommon {
    
    /** 私有构造函数防止实例化 */
    private DetourCommon() {
        throw new UnsupportedOperationException("工具类不能被实例化");
    }
    
    // ========== 基础数学函数 ==========
    
    /**
     * 交换两个值
     * @param a 值A
     * @param b 值B
     * @return 数组[a', b']，其中a' = b, b' = a
     */
    public static double[] dtSwap(double a, double b) {
        return new double[]{b, a};
    }
    
    /**
     * 返回两个值中的最小值
     * @param a 值A
     * @param b 值B
     * @return 较小的值
     */
    public static double dtMin(double a, double b) {
        return a < b ? a : b;
    }
    
    /**
     * 返回两个值中的最大值
     * @param a 值A
     * @param b 值B
     * @return 较大的值
     */
    public static double dtMax(double a, double b) {
        return a > b ? a : b;
    }
    
    /**
     * 返回绝对值
     * @param a 值
     * @return 绝对值
     */
    public static double dtAbs(double a) {
        return a < 0 ? -a : a;
    }
    
    /**
     * 返回平方值
     * @param a 值
     * @return 平方值
     */
    public static double dtSqr(double a) {
        return a * a;
    }
    
    /**
     * 将值限制在指定范围内
     * @param v 要限制的值
     * @param mn 最小值
     * @param mx 最大值
     * @return 限制后的值
     */
    public static double dtClamp(double v, double mn, double mx) {
        return v < mn ? mn : (v > mx ? mx : v);
    }
    
    /**
     * 向下取整
     * @param x 值
     * @return 向下取整的结果
     */
    public static double dtFloor(double x) {
        return Math.floor(x);
    }
    
    /**
     * 向上取整
     * @param x 值
     * @return 向上取整的结果
     */
    public static double dtCeil(double x) {
        return Math.ceil(x);
    }
    
    /**
     * 正弦函数
     * @param x 弧度值
     * @return 正弦值
     */
    public static double dtSin(double x) {
        return Math.sin(x);
    }
    
    /**
     * 余弦函数
     * @param x 弧度值
     * @return 余弦值
     */
    public static double dtCos(double x) {
        return Math.cos(x);
    }
    
    /**
     * 反正切函数
     * @param y Y值
     * @param x X值
     * @return 弧度值
     */
    public static double dtAtan2(double y, double x) {
        return Math.atan2(y, x);
    }
    
    /**
     * 平方根函数
     * @param x 值
     * @return 平方根
     */
    public static double dtSqrt(double x) {
        return Math.sqrt(x);
    }
    
    /**
     * 取模函数
     * @param x 被除数
     * @param y 除数
     * @return 余数
     */
    public static double dtfMod(double x, double y) {
        return x % y;
    }
    
    /**
     * 线性插值
     * @param a 起始值
     * @param b 结束值
     * @param t 插值因子 [0, 1]
     * @return 插值结果
     */
    public static double dtLerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    
    // ========== 向量操作函数 ==========
    
    /**
     * 计算两个向量的叉积 (v1 x v2)
     * @param dest 结果向量 [(x, y, z)]
     * @param v1 向量1 [(x, y, z)]
     * @param v2 向量2 [(x, y, z)]
     */
    public static void dtVcross(double[] dest, double[] v1, double[] v2) {
        dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
        dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
        dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
    }
    
    /**
     * 计算两个向量的点积 (v1 . v2)
     * @param v1 向量1 [(x, y, z)]
     * @param v2 向量2 [(x, y, z)]
     * @return 点积结果
     */
    public static double dtVdot(double[] v1, double[] v2) {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    }
    
    /**
     * 缩放向量加法 (v1 + (v2 * s))
     * @param dest 结果向量 [(x, y, z)]
     * @param v1 基础向量 [(x, y, z)]
     * @param v2 要缩放并添加到v1的向量 [(x, y, z)]
     * @param s v2的缩放因子
     */
    public static void dtVmad(double[] dest, double[] v1, double[] v2, double s) {
        dest[0] = v1[0] + v2[0] * s;
        dest[1] = v1[1] + v2[1] * s;
        dest[2] = v1[2] + v2[2] * s;
    }
    
    /**
     * 两个向量之间的线性插值 (v1 向 v2)
     * @param dest 结果向量 [(x, y, z)]
     * @param v1 起始向量
     * @param v2 目标向量
     * @param t 插值因子 [限制: 0 <= value <= 1.0]
     */
    public static void dtVlerp(double[] dest, double[] v1, double[] v2, double t) {
        dest[0] = v1[0] + (v2[0] - v1[0]) * t;
        dest[1] = v1[1] + (v2[1] - v1[1]) * t;
        dest[2] = v1[2] + (v2[2] - v1[2]) * t;
    }
    
    /**
     * 向量加法 (v1 + v2)
     * @param dest 结果向量 [(x, y, z)]
     * @param v1 基础向量 [(x, y, z)]
     * @param v2 要添加到v1的向量 [(x, y, z)]
     */
    public static void dtVadd(double[] dest, double[] v1, double[] v2) {
        dest[0] = v1[0] + v2[0];
        dest[1] = v1[1] + v2[1];
        dest[2] = v1[2] + v2[2];
    }
    
    /**
     * 向量减法 (v1 - v2)
     * @param dest 结果向量 [(x, y, z)]
     * @param v1 基础向量 [(x, y, z)]
     * @param v2 要从v1减去的向量 [(x, y, z)]
     */
    public static void dtVsub(double[] dest, double[] v1, double[] v2) {
        dest[0] = v1[0] - v2[0];
        dest[1] = v1[1] - v2[1];
        dest[2] = v1[2] - v2[2];
    }
    
    /**
     * 向量缩放 (v * t)
     * @param dest 结果向量 [(x, y, z)]
     * @param v 要缩放的向量 [(x, y, z)]
     * @param t 缩放因子
     */
    public static void dtVscale(double[] dest, double[] v, double t) {
        dest[0] = v[0] * t;
        dest[1] = v[1] * t;
        dest[2] = v[2] * t;
    }
    
    /**
     * 从指定向量中选择每个元素的最小值
     * @param mn 向量（将用结果更新） [(x, y, z)]
     * @param v 向量 [(x, y, z)]
     */
    public static void dtVmin(double[] mn, double[] v) {
        mn[0] = dtMin(mn[0], v[0]);
        mn[1] = dtMin(mn[1], v[1]);
        mn[2] = dtMin(mn[2], v[2]);
    }
    
    /**
     * 从指定向量中选择每个元素的最大值
     * @param mx 向量（将用结果更新） [(x, y, z)]
     * @param v 向量 [(x, y, z)]
     */
    public static void dtVmax(double[] mx, double[] v) {
        mx[0] = dtMax(mx[0], v[0]);
        mx[1] = dtMax(mx[1], v[1]);
        mx[2] = dtMax(mx[2], v[2]);
    }
    
    /**
     * 设置向量元素为指定值
     * @param dest 结果向量 [(x, y, z)]
     * @param x 向量的x值
     * @param y 向量的y值
     * @param z 向量的z值
     */
    public static void dtVset(double[] dest, double x, double y, double z) {
        dest[0] = x;
        dest[1] = y;
        dest[2] = z;
    }
    
    /**
     * 向量复制
     * @param dest 结果 [(x, y, z)]
     * @param a 要复制的向量 [(x, y, z)]
     */
    public static void dtVcopy(double[] dest, double[] a) {
        dest[0] = a[0];
        dest[1] = a[1];
        dest[2] = a[2];
    }
    
    /**
     * 计算向量的标量长度
     * @param v 向量 [(x, y, z)]
     * @return 向量的标量长度
     */
    public static double dtVlen(double[] v) {
        return dtSqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
    
    /**
     * 计算向量标量长度的平方 (len * len)
     * @param v 向量 [(x, y, z)]
     * @return 向量标量长度的平方
     */
    public static double dtVlenSqr(double[] v) {
        return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    }
    
    /**
     * 计算两点之间的距离（使用float数组）
     * @param v1 第一个点 [x, y, z]
     * @param v2 第二个点 [x, y, z]
     * @return 两点之间的距离
     */
    public static float dtVdist(float[] v1, float[] v2) {
        float dx = v2[0] - v1[0];
        float dy = v2[1] - v1[1];
        float dz = v2[2] - v1[2];
        return (float)Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    /**
     * 返回两点之间距离的平方
     * @param v1 点1 [(x, y, z)]
     * @param v2 点2 [(x, y, z)]
     * @return 两点之间距离的平方
     */
    public static double dtVdistSqr(double[] v1, double[] v2) {
        double dx = v2[0] - v1[0];
        double dy = v2[1] - v1[1];
        double dz = v2[2] - v1[2];
        return dx * dx + dy * dy + dz * dz;
    }
    
    /**
     * 计算指定点在xz平面上的距离
     * @param v1 点1 [(x, y, z)]
     * @param v2 点2 [(x, y, z)]
     * @return xz平面上点之间的距离
     */
    public static double dtVdist2D(double[] v1, double[] v2) {
        double dx = v2[0] - v1[0];
        double dz = v2[2] - v1[2];
        return dtSqrt(dx * dx + dz * dz);
    }
    
    /**
     * 计算指定点在xz平面上距离的平方
     * @param v1 点1 [(x, y, z)]
     * @param v2 点2 [(x, y, z)]
     * @return xz平面上点之间距离的平方
     */
    public static double dtVdist2DSqr(double[] v1, double[] v2) {
        double dx = v2[0] - v1[0];
        double dz = v2[2] - v1[2];
        return dx * dx + dz * dz;
    }
    
    /**
     * 规范化向量
     * @param v 要规范化的向量 [(x, y, z)]
     */
    public static void dtVnormalize(double[] v) {
        double d = 1.0 / dtSqrt(dtSqr(v[0]) + dtSqr(v[1]) + dtSqr(v[2]));
        v[0] *= d;
        v[1] *= d;
        v[2] *= d;
    }
    
    /**
     * 检查两个向量是否相等
     * @param a 向量A [(x, y, z)]
     * @param b 向量B [(x, y, z)]
     * @return 如果向量相同则返回true
     */
    public static boolean dtVisEqual(int[] a, int[] b) {
        return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
    }
    
    /**
     * 执行指定点的"粗略"位置检查
     * @param p0 点1 [(x, y, z)]
     * @param p1 点2 [(x, y, z)]
     * @return 如果点被认为在同一位置则返回true
     */
    public static boolean dtVequal(double[] p0, double[] p1) {
        double thr = dtSqr(1.0 / 16384.0);
        double d = dtVdistSqr(p0, p1);
        return d < thr;
    }
    
    /**
     * 计算两个向量在xz平面上的点积 (u . v)
     * @param u 向量1 [(x, y, z)]
     * @param v 向量2 [(x, y, z)]
     * @return xz平面上的点积
     */
    public static double dtVdot2D(double[] u, double[] v) {
        return u[0] * v[0] + u[2] * v[2];
    }
    
    /**
     * 计算两个向量的xz平面2D垂直积 (uz*vx - ux*vz)
     * @param u 左侧向量 [(x, y, z)]
     * @param v 右侧向量 [(x, y, z)]
     * @return xz平面上的垂直积
     */
    public static double dtVperp2D(double[] u, double[] v) {
        return u[2] * v[0] - u[0] * v[2];
    }
    
    // ========== 计算几何函数 ==========
    
    /**
     * 计算三角形ABC的有符号xz平面面积，或直线AB到点C的关系
     * @param a 顶点A [(x, y, z)]
     * @param b 顶点B [(x, y, z)]
     * @param c 顶点C [(x, y, z)]
     * @return 三角形的有符号xz平面面积
     */
    public static double dtTriArea2D(double[] a, double[] b, double[] c) {
        double abx = b[0] - a[0];
        double abz = b[2] - a[2];
        double acx = c[0] - a[0];
        double acz = c[2] - a[2];
        return acx * abz - abx * acz;
    }
    
    /**
     * 确定两个轴对齐边界框是否重叠
     * @param amin 盒子A的最小边界 [(x, y, z)]
     * @param amax 盒子A的最大边界 [(x, y, z)]
     * @param bmin 盒子B的最小边界 [(x, y, z)]
     * @param bmax 盒子B的最大边界 [(x, y, z)]
     * @return 如果两个AABB重叠则返回true
     */
    public static boolean dtOverlapQuantBounds(int[] amin, int[] amax, int[] bmin, int[] bmax) {
        boolean overlap = true;
        overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
        overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
        overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
        return overlap;
    }
    
    /**
     * 确定两个轴对齐边界框是否重叠
     * @param amin 盒子A的最小边界 [(x, y, z)]
     * @param amax 盒子A的最大边界 [(x, y, z)]
     * @param bmin 盒子B的最小边界 [(x, y, z)]
     * @param bmax 盒子B的最大边界 [(x, y, z)]
     * @return 如果两个AABB重叠则返回true
     */
    public static boolean dtOverlapBounds(double[] amin, double[] amax, double[] bmin, double[] bmax) {
        boolean overlap = true;
        overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
        overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
        overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
        return overlap;
    }
    
    /**
     * 计算从指定参考点到三角形的最近点
     * @param closest 三角形上的最近点
     * @param p 要测试的参考点 [(x, y, z)]
     * @param a 三角形ABC的顶点A [(x, y, z)]
     * @param b 三角形ABC的顶点B [(x, y, z)]
     * @param c 三角形ABC的顶点C [(x, y, z)]
     */
    public static void dtClosestPtPointTriangle(double[] closest, double[] p, 
                                               double[] a, double[] b, double[] c) {
        // 检查点是否在顶点A的外围
        double[] ab = new double[3];
        double[] ac = new double[3];
        double[] ap = new double[3];
        
        dtVsub(ab, b, a);
        dtVsub(ac, c, a);
        dtVsub(ap, p, a);
        
        double d1 = dtVdot(ab, ap);
        double d2 = dtVdot(ac, ap);
        if (d1 <= 0.0 && d2 <= 0.0) {
            dtVcopy(closest, a); // 重心坐标 (1,0,0)
            return;
        }
        
        // 检查点是否在顶点B的外围
        double[] bp = new double[3];
        dtVsub(bp, p, b);
        double d3 = dtVdot(ab, bp);
        double d4 = dtVdot(ac, bp);
        if (d3 >= 0.0 && d4 <= d3) {
            dtVcopy(closest, b); // 重心坐标 (0,1,0)
            return;
        }
        
        // 检查点是否在边AB上
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            double v = d1 / (d1 - d3);
            dtVmad(closest, a, ab, v); // 重心坐标 (1-v,v,0)
            return;
        }
        
        // 检查点是否在顶点C的外围
        double[] cp = new double[3];
        dtVsub(cp, p, c);
        double d5 = dtVdot(ab, cp);
        double d6 = dtVdot(ac, cp);
        if (d6 >= 0.0 && d5 <= d6) {
            dtVcopy(closest, c); // 重心坐标 (0,0,1)
            return;
        }
        
        // 检查点是否在边AC上
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            double w = d2 / (d2 - d6);
            dtVmad(closest, a, ac, w); // 重心坐标 (1-w,0,w)
            return;
        }
        
        // 检查点是否在边BC上
        double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            double[] bc = new double[3];
            dtVsub(bc, c, b);
            dtVmad(closest, b, bc, w); // 重心坐标 (0,1-w,w)
            return;
        }
        
        // 点在三角形内部
        double denom = 1.0 / (va + vb + vc);
        double v = vb * denom;
        double w = vc * denom;
        dtVmad(closest, a, ab, v);
        dtVmad(closest, closest, ac, w); // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
    }
    
    /**
     * 确定指定点是否在xz平面上的凸多边形内
     * @param pt 要检查的点 [(x, y, z)]
     * @param verts 多边形顶点 [(x, y, z) * nverts]
     * @param nverts 顶点数量 [限制: >= 3]
     * @return 如果点在多边形内则返回true
     */
    public static boolean dtPointInPolygon(double[] pt, double[] verts, int nverts) {
        // TODO: 用单纯的符号
        boolean c = false;
        int i, j;
        for (i = 0, j = nverts - 1; i < nverts; j = i++) {
            int vi = i * 3;
            int vj = j * 3;
            if (((verts[vi + 2] > pt[2]) != (verts[vj + 2] > pt[2])) &&
                (pt[0] < (verts[vj] - verts[vi]) * (pt[2] - verts[vi + 2]) / (verts[vj + 2] - verts[vi + 2]) + verts[vi])) {
                c = !c;
            }
        }
        return c;
    }
    
    /**
     * 计算下一个2的幂
     * @param v 值
     * @return 大于等于v的最小2的幂
     */
    public static int dtNextPow2(int v) {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v++;
        return v;
    }
    
    /**
     * 计算整数的以2为底的对数
     * @param v 值
     * @return 以2为底的对数
     */
    public static int dtIlog2(int v) {
        int r;
        int shift;
        r = (v > 0xffff ? 1 : 0) << 4; v >>= r;
        shift = (v > 0xff ? 1 : 0) << 3; v >>= shift; r |= shift;
        shift = (v > 0xf ? 1 : 0) << 2; v >>= shift; r |= shift;
        shift = (v > 0x3 ? 1 : 0) << 1; v >>= shift; r |= shift;
        r |= (v >> 1);
        return r;
    }
    
    /**
     * 对齐到最近的4字节边界
     * @param x 值
     * @return 对齐后的值
     */
    public static int dtAlign(int x) {
        return (x + 3) & ~3;
    }
} 