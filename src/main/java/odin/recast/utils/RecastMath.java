package odin.recast.utils;

import static odin.recast.config.RecastConstants.RC_PI;

/**
 * Recast数学工具类
 * 翻译自UE5 Recast.h中的数学函数
 * 
 * @author UE5NavMesh4J
 */
public final class RecastMath {
    
    /** 私有构造函数防止实例化 */
    private RecastMath() {
        throw new UnsupportedOperationException("工具类不能被实例化");
    }
    
    // 基础数学函数
    
    public static float rcSin(float x) {
        return (float) Math.sin(x);
    }
    
    public static double rcSin(double x) {
        return Math.sin(x);
    }
    
    public static float rcCos(float x) {
        return (float) Math.cos(x);
    }
    
    public static double rcCos(double x) {
        return Math.cos(x);
    }
    
    public static float rcFloor(float x) {
        return (float) Math.floor(x);
    }
    
    public static double rcFloor(double x) {
        return Math.floor(x);
    }
    
    public static float rcCeil(float x) {
        return (float) Math.ceil(x);
    }
    
    public static double rcCeil(double x) {
        return Math.ceil(x);
    }
    
    public static float rcAbs(float x) {
        return Math.abs(x);
    }
    
    public static double rcAbs(double x) {
        return Math.abs(x);
    }
    
    public static float rcSqrt(float x) {
        return (float) Math.sqrt(x);
    }
    
    public static double rcSqrt(double x) {
        return Math.sqrt(x);
    }
    
    // 模板函数的Java实现
    
    public static <T extends Comparable<T>> T rcMin(T a, T b) {
        return a.compareTo(b) < 0 ? a : b;
    }
    
    public static float rcMin(float a, float b) {
        return a < b ? a : b;
    }
    
    public static double rcMin(double a, double b) {
        return a < b ? a : b;
    }
    
    public static int rcMin(int a, int b) {
        return a < b ? a : b;
    }
    
    public static <T extends Comparable<T>> T rcMax(T a, T b) {
        return a.compareTo(b) > 0 ? a : b;
    }
    
    public static float rcMax(float a, float b) {
        return a > b ? a : b;
    }
    
    public static double rcMax(double a, double b) {
        return a > b ? a : b;
    }
    
    public static int rcMax(int a, int b) {
        return a > b ? a : b;
    }
    
    public static <T extends Comparable<T>> T rcAbs(T a) {
        return a.compareTo((T) Integer.valueOf(0)) < 0 ? 
            (T) Integer.valueOf(-((Integer) a)) : a;
    }
    
    public static int rcAbs(int a) {
        return a < 0 ? -a : a;
    }
    
    public static <T> T rcSqr(T a) {
        if (a instanceof Float) {
            float val = (Float) a;
            return (T) Float.valueOf(val * val);
        } else if (a instanceof Double) {
            double val = (Double) a;
            return (T) Double.valueOf(val * val);
        } else if (a instanceof Integer) {
            int val = (Integer) a;
            return (T) Integer.valueOf(val * val);
        }
        throw new UnsupportedOperationException("不支持的类型");
    }
    
    public static float rcSqr(float a) {
        return a * a;
    }
    
    public static double rcSqr(double a) {
        return a * a;
    }
    
    public static int rcSqr(int a) {
        return a * a;
    }
    
    public static <T extends Comparable<T>> T rcClamp(T v, T mn, T mx) {
        return v.compareTo(mn) < 0 ? mn : (v.compareTo(mx) > 0 ? mx : v);
    }
    
    public static float rcClamp(float v, float mn, float mx) {
        return v < mn ? mn : (v > mx ? mx : v);
    }
    
    public static double rcClamp(double v, double mn, double mx) {
        return v < mn ? mn : (v > mx ? mx : v);
    }
    
    public static int rcClamp(int v, int mn, int mx) {
        return v < mn ? mn : (v > mx ? mx : v);
    }
    
    // 向量数学函数
    
    /**
     * 向量叉积
     * @param dest 结果向量 [3]
     * @param v1 向量1 [3]
     * @param v2 向量2 [3]
     */
    public static void rcVcross(float[] dest, float[] v1, float[] v2) {
        dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
        dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
        dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
    }
    
    /**
     * 向量点积
     * @param v1 向量1 [3]
     * @param v2 向量2 [3]
     * @return 点积结果
     */
    public static float rcVdot(float[] v1, float[] v2) {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    }
    
    /**
     * 向量乘法和加法：dest = v1 + v2 * s
     * @param dest 结果向量 [3]
     * @param v1 向量1 [3]
     * @param v2 向量2 [3]
     * @param s 标量
     */
    public static void rcVmad(float[] dest, float[] v1, float[] v2, float s) {
        dest[0] = v1[0] + v2[0] * s;
        dest[1] = v1[1] + v2[1] * s;
        dest[2] = v1[2] + v2[2] * s;
    }
    
    /**
     * 向量加法：dest = v1 + v2
     * @param dest 结果向量 [3]
     * @param v1 向量1 [3]
     * @param v2 向量2 [3]
     */
    public static void rcVadd(float[] dest, float[] v1, float[] v2) {
        dest[0] = v1[0] + v2[0];
        dest[1] = v1[1] + v2[1];
        dest[2] = v1[2] + v2[2];
    }
    
    /**
     * 向量减法：dest = v1 - v2
     * @param dest 结果向量 [3]
     * @param v1 向量1 [3]
     * @param v2 向量2 [3]
     */
    public static void rcVsub(float[] dest, float[] v1, float[] v2) {
        dest[0] = v1[0] - v2[0];
        dest[1] = v1[1] - v2[1];
        dest[2] = v1[2] - v2[2];
    }
    
    /**
     * 计算最小边界：mn = min(mn, v)
     * @param mn 最小边界 [3]
     * @param v 向量 [3]
     */
    public static void rcVmin(float[] mn, float[] v) {
        mn[0] = rcMin(mn[0], v[0]);
        mn[1] = rcMin(mn[1], v[1]);
        mn[2] = rcMin(mn[2], v[2]);
    }
    
    /**
     * 计算最大边界：mx = max(mx, v)
     * @param mx 最大边界 [3]
     * @param v 向量 [3]
     */
    public static void rcVmax(float[] mx, float[] v) {
        mx[0] = rcMax(mx[0], v[0]);
        mx[1] = rcMax(mx[1], v[1]);
        mx[2] = rcMax(mx[2], v[2]);
    }
    
    /**
     * 向量复制：dest = v
     * @param dest 目标向量 [3]
     * @param v 源向量 [3]
     */
    public static void rcVcopy(float[] dest, float[] v) {
        dest[0] = v[0];
        dest[1] = v[1];
        dest[2] = v[2];
    }
    
    /**
     * 计算两向量间距离
     * @param v1 向量1 [3]
     * @param v2 向量2 [3]
     * @return 距离
     */
    public static float rcVdist(float[] v1, float[] v2) {
        float dx = v2[0] - v1[0];
        float dy = v2[1] - v1[1];
        float dz = v2[2] - v1[2];
        return rcSqrt(dx * dx + dy * dy + dz * dz);
    }
    
    /**
     * 计算两向量间距离的平方
     * @param v1 向量1 [3]
     * @param v2 向量2 [3]
     * @return 距离的平方
     */
    public static float rcVdistSqr(float[] v1, float[] v2) {
        float dx = v2[0] - v1[0];
        float dy = v2[1] - v1[1];
        float dz = v2[2] - v1[2];
        return dx * dx + dy * dy + dz * dz;
    }
    
    /**
     * 向量归一化
     * @param v 要归一化的向量 [3]
     */
    public static void rcVnormalize(float[] v) {
        float d = rcSqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (d > 1e-6f) {
            d = 1.0f / d;
            v[0] *= d;
            v[1] *= d;
            v[2] *= d;
        }
    }
} 