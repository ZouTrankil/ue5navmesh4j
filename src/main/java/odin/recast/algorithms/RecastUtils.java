package odin.recast.algorithms;

import odin.recast.core.RecastHeightfield.Heightfield;
import odin.recast.core.RecastHeightfield.CompactHeightfield;
import odin.recast.core.RecastSpan.Span;
import odin.recast.core.RecastSpan.SpanPool;
import odin.recast.config.RecastConstants;

import static odin.recast.utils.RecastMath.*;
import static odin.recast.config.RecastConstants.*;

/**
 * Recast基础工具类
 * 翻译自UE5 Recast.cpp中的工具函数
 * 
 * @author UE5NavMesh4J
 */
public final class RecastUtils {
    
    /** 私有构造函数防止实例化 */
    private RecastUtils() {
        throw new UnsupportedOperationException("工具类不能被实例化");
    }
    
    /**
     * 检查两个边界框是否重叠
     * @param amin 边界框A的最小值 [3]
     * @param amax 边界框A的最大值 [3]
     * @param bmin 边界框B的最小值 [3] 
     * @param bmax 边界框B的最大值 [3]
     * @return true如果重叠
     */
    public static boolean overlapBounds(float[] amin, float[] amax, float[] bmin, float[] bmax) {
        boolean overlap = true;
        overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
        overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
        overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
        return overlap;
    }
    
    /**
     * 检查两个区间是否重叠
     * @param amin 区间A的最小值
     * @param amax 区间A的最大值
     * @param bmin 区间B的最小值
     * @param bmax 区间B的最大值
     * @return true如果重叠
     */
    public static boolean overlapInterval(int amin, int amax, int bmin, int bmax) {
        if (amax < bmin) return false;
        if (amin > bmax) return false;
        return true;
    }
    
    /**
     * 计算三角形的法线
     * @param v0 顶点0 [3]
     * @param v1 顶点1 [3] 
     * @param v2 顶点2 [3]
     * @param norm 输出法线 [3]
     */
    public static void calcTriNormal(float[] v0, float[] v1, float[] v2, float[] norm) {
        float[] e0 = new float[3];
        float[] e1 = new float[3];
        
        rcVsub(e0, v1, v0);
        rcVsub(e1, v2, v0);
        rcVcross(norm, e0, e1);
        rcVnormalize(norm);
    }
    
    /**
     * 计算顶点数组的边界框
     * @param verts 顶点数组 [(x, y, z) * nv]
     * @param nv 顶点数量
     * @param bmin 输出最小边界 [3]
     * @param bmax 输出最大边界 [3]
     */
    public static void calcBounds(float[] verts, int nv, float[] bmin, float[] bmax) {
        // 计算边界框
        rcVcopy(bmin, verts);
        rcVcopy(bmax, verts);
        
        for (int i = 1; i < nv; i++) {
            int offset = i * 3;
            float[] v = {verts[offset], verts[offset + 1], verts[offset + 2]};
            rcVmin(bmin, v);
            rcVmax(bmax, v);
        }
    }
    
    /**
     * 根据边界框和单元格大小计算网格尺寸
     * @param bmin 最小边界 [3]
     * @param bmax 最大边界 [3]
     * @param cs 单元格大小
     * @param w 输出宽度
     * @param h 输出高度
     * @return 网格尺寸 [宽度, 高度]
     */
    public static int[] calcGridSize(float[] bmin, float[] bmax, float cs) {
        int w = (int)((bmax[0] - bmin[0]) / cs + 0.5f);
        int h = (int)((bmax[2] - bmin[2]) / cs + 0.5f);
        return new int[]{w, h};
    }
    
    /**
     * 获取方向的X偏移
     * @param dir 方向 (0-3)
     * @return X偏移
     */
    public static int getDirOffsetX(int dir) {
        int[] offset = {-1, 0, 1, 0};
        return (dir >= 0 && dir < 4) ? offset[dir] : 0;
    }
    
    /**
     * 获取方向的Y偏移
     * @param dir 方向 (0-3)
     * @return Y偏移
     */
    public static int getDirOffsetY(int dir) {
        int[] offset = {0, 1, 0, -1};
        return (dir >= 0 && dir < 4) ? offset[dir] : 0;
    }
    
    /**
     * 创建高度字段
     * @param ctx 构建上下文
     * @param hf 高度字段
     * @param width 宽度
     * @param height 高度
     * @param bmin 最小边界 [3]
     * @param bmax 最大边界 [3]
     * @param cs 单元格大小
     * @param ch 单元格高度
     * @return true如果成功创建
     */
    public static boolean createHeightfield(RecastContext ctx, Heightfield hf, 
                                          int width, int height,
                                          float[] bmin, float[] bmax,
                                          float cs, float ch) {
        try {
            hf.init(width, height, bmin, bmax, cs, ch);
            return true;
        } catch (Exception e) {
            if (ctx != null) {
                ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_ERROR, 
                       "创建高度字段失败: %s", e.getMessage());
            }
            return false;
        }
    }
    
    /**
     * 重置高度字段
     * @param hf 高度字段
     */
    public static void resetHeightfield(Heightfield hf) {
        if (hf.spans != null) {
            for (int i = 0; i < hf.width * hf.height; i++) {
                hf.spans[i] = null;
            }
        }
        
        // 重置span池
        hf.freelist = null;
        hf.pools = null;
    }
    
    /**
     * 标记可行走的三角形
     * @param ctx 构建上下文
     * @param walkableSlopeAngle 可行走坡度角（度）
     * @param verts 顶点数组 [(x, y, z) * nv]
     * @param nv 顶点数量
     * @param tris 三角形索引 [(a, b, c) * nt]
     * @param nt 三角形数量
     * @param areas 输出区域id数组 [nt]
     */
    public static void markWalkableTriangles(RecastContext ctx, float walkableSlopeAngle,
                                           float[] verts, int nv,
                                           int[] tris, int nt,
                                           byte[] areas) {
        float walkableThr = rcCos(walkableSlopeAngle / 180.0f * (float)RC_PI);
        markWalkableTrianglesCos(ctx, walkableThr, verts, nv, tris, nt, areas);
    }
    
    /**
     * 使用余弦值标记可行走的三角形
     * @param ctx 构建上下文
     * @param walkableSlopeCos 可行走坡度余弦值
     * @param verts 顶点数组 [(x, y, z) * nv]
     * @param nv 顶点数量
     * @param tris 三角形索引 [(a, b, c) * nt]
     * @param nt 三角形数量
     * @param areas 输出区域id数组 [nt]
     */
    public static void markWalkableTrianglesCos(RecastContext ctx, float walkableSlopeCos,
                                              float[] verts, int nv,
                                              int[] tris, int nt,
                                              byte[] areas) {
        float[] norm = new float[3];
        
        for (int i = 0; i < nt; i++) {
            int triOffset = i * 3;
            int[] tri = {tris[triOffset], tris[triOffset + 1], tris[triOffset + 2]};
            
            float[] v0 = {verts[tri[0] * 3], verts[tri[0] * 3 + 1], verts[tri[0] * 3 + 2]};
            float[] v1 = {verts[tri[1] * 3], verts[tri[1] * 3 + 1], verts[tri[1] * 3 + 2]};
            float[] v2 = {verts[tri[2] * 3], verts[tri[2] * 3 + 1], verts[tri[2] * 3 + 2]};
            
            calcTriNormal(v0, v1, v2, norm);
            
            // 检查三角形是否可行走
            if (norm[1] > walkableSlopeCos) {
                areas[i] = (byte)RC_WALKABLE_AREA;
            }
        }
    }
    
    /**
     * 清除不可行走的三角形
     * @param ctx 构建上下文
     * @param walkableSlopeAngle 可行走坡度角（度）
     * @param verts 顶点数组 [(x, y, z) * nv]
     * @param nv 顶点数量
     * @param tris 三角形索引 [(a, b, c) * nt]
     * @param nt 三角形数量
     * @param areas 区域id数组 [nt]
     */
    public static void clearUnwalkableTriangles(RecastContext ctx, float walkableSlopeAngle,
                                              float[] verts, int nv,
                                              int[] tris, int nt,
                                              byte[] areas) {
        float walkableThr = rcCos(walkableSlopeAngle / 180.0f * (float)RC_PI);
        float[] norm = new float[3];
        
        for (int i = 0; i < nt; i++) {
            int triOffset = i * 3;
            int[] tri = {tris[triOffset], tris[triOffset + 1], tris[triOffset + 2]};
            
            float[] v0 = {verts[tri[0] * 3], verts[tri[0] * 3 + 1], verts[tri[0] * 3 + 2]};
            float[] v1 = {verts[tri[1] * 3], verts[tri[1] * 3 + 1], verts[tri[1] * 3 + 2]};
            float[] v2 = {verts[tri[2] * 3], verts[tri[2] * 3 + 1], verts[tri[2] * 3 + 2]};
            
            calcTriNormal(v0, v1, v2, norm);
            
            // 检查三角形是否不可行走
            if (norm[1] <= walkableThr) {
                areas[i] = (byte)RC_NULL_AREA;
            }
        }
    }
    
    /**
     * 计算高度字段中的span数量
     * @param ctx 构建上下文
     * @param hf 高度字段
     * @return span数量
     */
    public static int getHeightFieldSpanCount(RecastContext ctx, Heightfield hf) {
        int w = hf.width;
        int h = hf.height;
        int spanCount = 0;
        
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                Span s = hf.getSpan(x, y);
                while (s != null) {
                    if (s.data.area != RC_NULL_AREA) {
                        spanCount++;
                    }
                    s = s.next;
                }
            }
        }
        
        return spanCount;
    }
    
    /**
     * 计算所有三角形的法线
     * @param verts 顶点数组 [(x, y, z) * nv]
     * @param nv 顶点数量
     * @param tris 三角形索引 [(a, b, c) * nt]
     * @param nt 三角形数量
     * @param norms 输出法线数组 [(x, y, z) * nt]
     */
    public static void calcTriNormals(float[] verts, int nv, int[] tris, int nt, float[] norms) {
        for (int i = 0; i < nt; i++) {
            int triOffset = i * 3;
            int[] tri = {tris[triOffset], tris[triOffset + 1], tris[triOffset + 2]};
            
            float[] v0 = {verts[tri[0] * 3], verts[tri[0] * 3 + 1], verts[tri[0] * 3 + 2]};
            float[] v1 = {verts[tri[1] * 3], verts[tri[1] * 3 + 1], verts[tri[1] * 3 + 2]};
            float[] v2 = {verts[tri[2] * 3], verts[tri[2] * 3 + 1], verts[tri[2] * 3 + 2]};
            
            float[] norm = new float[3];
            calcTriNormal(v0, v1, v2, norm);
            
            norms[i * 3] = norm[0];
            norms[i * 3 + 1] = norm[1];
            norms[i * 3 + 2] = norm[2];
        }
    }
    
    /**
     * 裁剪多边形
     * @param in 输入多边形顶点 [(x, y, z) * n]
     * @param n 顶点数量
     * @param out 输出多边形顶点 [(x, y, z) * 最大12个]
     * @param pnx 裁剪平面法线X
     * @param pnz 裁剪平面法线Z
     * @param pd 裁剪平面距离
     * @return 输出顶点数量
     */
    public static int clipPoly(float[] in, int n, float[] out, 
                              float pnx, float pnz, float pd) {
        float[] d = new float[12];
        
        for (int i = 0; i < n; i++) {
            d[i] = pnx * in[i * 3] + pnz * in[i * 3 + 2] + pd;
        }
        
        int m = 0;
        for (int i = 0, j = n - 1; i < n; j = i, i++) {
            boolean ina = d[j] >= 0;
            boolean inb = d[i] >= 0;
            
            if (ina != inb) {
                float s = d[j] / (d[j] - d[i]);
                out[m * 3] = in[j * 3] + (in[i * 3] - in[j * 3]) * s;
                out[m * 3 + 1] = in[j * 3 + 1] + (in[i * 3 + 1] - in[j * 3 + 1]) * s;
                out[m * 3 + 2] = in[j * 3 + 2] + (in[i * 3 + 2] - in[j * 3 + 2]) * s;
                m++;
            }
            
            if (inb) {
                out[m * 3] = in[i * 3];
                out[m * 3 + 1] = in[i * 3 + 1];
                out[m * 3 + 2] = in[i * 3 + 2];
                m++;
            }
        }
        
        return m;
    }
} 