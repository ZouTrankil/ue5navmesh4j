package odin.recast.algorithms;

import odin.recast.core.RecastHeightfield.Heightfield;
import odin.recast.core.RecastSpan.Span;
import odin.recast.core.RecastSpan.SpanPool;
import odin.recast.core.RecastSpan.SpanCache;
import odin.recast.config.RecastEnums.TimerLabel;
import odin.recast.config.RecastEnums.RasterizationFlags;

import static odin.recast.utils.RecastMath.*;
import static odin.recast.config.RecastConstants.*;
import static odin.recast.algorithms.RecastUtils.*;

/**
 * Recast光栅化算法
 * 翻译自UE5 RecastRasterization.cpp
 * 
 * 负责将三角形网格转换为体素化的高度字段
 * 
 * @author UE5NavMesh4J
 */
public final class RecastRasterization {
    
    /** 私有构造函数防止实例化 */
    private RecastRasterization() {
        throw new UnsupportedOperationException("工具类不能被实例化");
    }
    
    /**
     * 从高度字段中分配一个span
     * @param hf 高度字段
     * @return 分配的span，如果内存不足则返回null
     */
    private static Span allocSpan(Heightfield hf) {
        // 如果空闲列表为空，分配新的span池
        if (hf.freelist == null) {
            // 创建新的span池
            SpanPool pool = new SpanPool();
            
            // 将池添加到链表中
            pool.next = hf.pools;
            hf.pools = pool;
            
            // 将新span添加到空闲列表
            Span freelist = hf.freelist;
            for (int i = RC_SPANS_PER_POOL - 1; i >= 0; i--) {
                pool.items[i].next = freelist;
                freelist = pool.items[i];
            }
            hf.freelist = freelist;
        }
        
        // 从空闲列表头部取出一个span
        Span span = hf.freelist;
        hf.freelist = hf.freelist.next;
        return span;
    }
    
    /**
     * 释放span到空闲列表
     * @param hf 高度字段
     * @param span 要释放的span
     */
    private static void freeSpan(Heightfield hf, Span span) {
        if (span == null) return;
        
        // 将span添加到空闲列表头部
        span.next = hf.freelist;
        hf.freelist = span;
    }
    
    /**
     * 添加span到高度字段
     * @param hf 高度字段
     * @param x X坐标
     * @param y Y坐标
     * @param smin span最小高度
     * @param smax span最大高度
     * @param area 区域id
     * @param flagMergeThr 合并阈值
     */
    private static void addSpan(Heightfield hf, int x, int y,
                               int smin, int smax,
                               int area, int flagMergeThr) {
        
        // 分配新span
        Span s = allocSpan(hf);
        if (s == null) return;
        
        s.data.smin = smin;
        s.data.smax = smax;
        s.data.area = area;
        s.next = null;
        
        // 如果单元格为空，直接添加第一个span
        Span[] spans = hf.spans[y * hf.width + x];
        if (spans == null || spans.length == 0) {
            hf.spans[y * hf.width + x] = new Span[]{s};
            return;
        }
        
        Span prev = null;
        Span cur = spans[0];
        
        // 插入并合并span
        while (cur != null) {
            if (cur.data.smin > s.data.smax) {
                // 当前span在新span上方，跳出循环
                break;
            } else if (cur.data.smax < s.data.smin) {
                // 当前span在新span下方，继续向上
                prev = cur;
                cur = cur.next;
            } else {
                // 重叠，需要合并
                
                // 对于高度接近的span，优先选择可行走区域
                if (rcAbs(s.data.smax - cur.data.smax) <= flagMergeThr) {
                    s.data.area = rcMax(s.data.area, cur.data.area);
                } else {
                    // 如果当前span会成为顶部，使用其区域
                    if (cur.data.smax > s.data.smax) {
                        s.data.area = cur.data.area;
                    }
                }
                
                // 合并高度区间
                if (cur.data.smin < s.data.smin) {
                    s.data.smin = cur.data.smin;
                }
                if (cur.data.smax > s.data.smax) {
                    s.data.smax = cur.data.smax;
                }
                
                // 移除当前span
                Span next = cur.next;
                freeSpan(hf, cur);
                if (prev != null) {
                    prev.next = next;
                } else {
                    spans[0] = next;
                }
                cur = next;
            }
        }
        
        // 插入新span
        if (prev != null) {
            s.next = prev.next;
            prev.next = s;
        } else {
            s.next = spans[0];
            spans[0] = s;
        }
    }
    
    /**
     * 光栅化单个三角形
     * @param v0 三角形顶点0 [3]
     * @param v1 三角形顶点1 [3]
     * @param v2 三角形顶点2 [3]
     * @param area 区域id
     * @param hf 高度字段
     * @param bmin 边界框最小值 [3]
     * @param bmax 边界框最大值 [3]
     * @param cs 单元格大小
     * @param ics 单元格大小的倒数
     * @param ich 单元格高度的倒数
     * @param flagMergeThr 合并阈值
     * @param rasterizationFlags 光栅化标志
     * @param rasterizationMasks 光栅化掩码
     */
    private static void rasterizeTriangle(float[] v0, float[] v1, float[] v2,
                                        int area, Heightfield hf,
                                        float[] bmin, float[] bmax,
                                        float cs, float ics, float ich,
                                        int flagMergeThr,
                                        int rasterizationFlags,
                                        int[] rasterizationMasks) {
        
        int w = hf.width;
        int h = hf.height;
        float[] tmin = new float[3];
        float[] tmax = new float[3];
        float by = bmax[1] - bmin[1];
        
        // 计算三角形的边界框
        System.arraycopy(v0, 0, tmin, 0, 3);
        System.arraycopy(v0, 0, tmax, 0, 3);
        rcVmin(tmin, v1);
        rcVmin(tmin, v2);
        rcVmax(tmax, v1);
        rcVmax(tmax, v2);
        
        // 检查三角形是否与高度字段边界框重叠
        if (!overlapBounds(bmin, bmax, tmin, tmax)) {
            return;
        }
        
        // 计算三角形在网格上的足迹
        int x0 = (int)((tmin[0] - bmin[0]) * ics);
        int y0 = (int)((tmin[2] - bmin[2]) * ics);
        int x1 = (int)((tmax[0] - bmin[0]) * ics);
        int y1 = (int)((tmax[2] - bmin[2]) * ics);
        
        x0 = rcClamp(x0, 0, w - 1);
        y0 = rcClamp(y0, 0, h - 1);
        x1 = rcClamp(x1, 0, w - 1);
        y1 = rcClamp(y1, 0, h - 1);
        
        // 将三角形裁剪到所有接触的网格单元格中
        float[] in = new float[7 * 3];
        float[] out = new float[7 * 3];
        float[] inrow = new float[7 * 3];
        
        for (int y = y0; y <= y1; y++) {
            // 将多边形裁剪到行
            rcVcopy(in, 0, v0, 0, 3);
            rcVcopy(in, 3, v1, 0, 3);
            rcVcopy(in, 6, v2, 0, 3);
            int nvrow = 3;
            
            float cz = bmin[2] + y * cs;
            nvrow = clipPoly(in, nvrow, out, 0, 1, -cz);
            if (nvrow < 3) continue;
            nvrow = clipPoly(out, nvrow, inrow, 0, -1, cz + cs);
            if (nvrow < 3) continue;
            
            for (int x = x0; x <= x1; x++) {
                // 将多边形裁剪到列
                int nv = nvrow;
                float cx = bmin[0] + x * cs;
                nv = clipPoly(inrow, nv, out, 1, 0, -cx);
                if (nv < 3) continue;
                nv = clipPoly(out, nv, in, -1, 0, cx + cs);
                if (nv < 3) continue;
                
                // 计算span的最小值和最大值
                float smin = in[1];
                float smax = in[1];
                for (int i = 1; i < nv; i++) {
                    smin = rcMin(smin, in[i * 3 + 1]);
                    smax = rcMax(smax, in[i * 3 + 1]);
                }
                smin -= bmin[1];
                smax -= bmin[1];
                
                // 跳过超出高度字段边界框的span
                if (smax < 0.0f) continue;
                if (smin > by) continue;
                
                // 将span限制在高度字段边界框内
                if (smin < 0.0f) smin = 0;
                if (smax > by) smax = by;
                
                // 将span对齐到高度字段高度网格
                int ismin = rcClamp((int)rcFloor(smin * ich), 0, RC_SPAN_MAX_HEIGHT);
                int ismax = rcClamp((int)rcCeil(smax * ich), ismin + 1, RC_SPAN_MAX_HEIGHT);
                
                // 检查是否需要投影到底部
                boolean projectToBottom = (rasterizationFlags & RasterizationFlags.RC_PROJECT_TO_BOTTOM.getValue()) != 0;
                if (rasterizationMasks != null) {
                    projectToBottom = projectToBottom && (rasterizationMasks[x + y * w] != 0);
                }
                
                if (projectToBottom) {
                    ismin = 0;
                }
                
                addSpan(hf, x, y, ismin, ismax, area, flagMergeThr);
            }
        }
    }
    
    /**
     * 向数组复制向量
     * @param dst 目标数组
     * @param dstOffset 目标偏移
     * @param src 源数组
     * @param srcOffset 源偏移
     * @param length 复制长度
     */
    private static void rcVcopy(float[] dst, int dstOffset, float[] src, int srcOffset, int length) {
        System.arraycopy(src, srcOffset, dst, dstOffset, length);
    }
    
    /**
     * 添加span到高度字段
     * @param ctx 构建上下文
     * @param hf 高度字段
     * @param x X坐标
     * @param y Y坐标
     * @param smin span最小高度
     * @param smax span最大高度
     * @param area 区域id
     * @param flagMergeThr 合并阈值
     */
    public static void addSpan(RecastContext ctx, Heightfield hf, int x, int y,
                              int smin, int smax, int area, int flagMergeThr) {
        addSpan(hf, x, y, smin, smax, area, flagMergeThr);
    }
    
    /**
     * 添加多个span到高度字段
     * @param ctx 构建上下文
     * @param hf 高度字段
     * @param flagMergeThr 合并阈值
     * @param cachedSpans 缓存的span数据
     * @param nspans span数量
     */
    public static void addSpans(RecastContext ctx, Heightfield hf, int flagMergeThr,
                               SpanCache[] cachedSpans, int nspans) {
        for (int i = 0; i < nspans; i++) {
            SpanCache cache = cachedSpans[i];
            addSpan(hf, cache.x, cache.y, cache.data.smin, cache.data.smax, 
                   cache.data.area, flagMergeThr);
        }
    }
    
    /**
     * 光栅化单个三角形
     * @param ctx 构建上下文
     * @param v0 三角形顶点0 [3]
     * @param v1 三角形顶点1 [3]
     * @param v2 三角形顶点2 [3]
     * @param area 区域id
     * @param solid 高度字段
     * @param flagMergeThr 合并阈值
     * @param rasterizationFlags 光栅化标志
     * @param rasterizationMasks 光栅化掩码
     */
    public static void rasterizeTriangle(RecastContext ctx, 
                                       float[] v0, float[] v1, float[] v2,
                                       int area, Heightfield solid,
                                       int flagMergeThr, 
                                       int rasterizationFlags,
                                       int[] rasterizationMasks) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_RASTERIZE_TRIANGLES);
        }
        
        float ics = 1.0f / solid.cs;
        float ich = 1.0f / solid.ch;
        
        rasterizeTriangle(v0, v1, v2, area, solid, solid.bmin, solid.bmax,
                         solid.cs, ics, ich, flagMergeThr, 
                         rasterizationFlags, rasterizationMasks);
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_RASTERIZE_TRIANGLES);
        }
    }
    
    /**
     * 光栅化多个三角形（int索引）
     * @param ctx 构建上下文
     * @param verts 顶点数组 [(x, y, z) * nv]
     * @param nv 顶点数量
     * @param tris 三角形索引数组 [(a, b, c) * nt]
     * @param areas 区域id数组 [nt]
     * @param nt 三角形数量
     * @param solid 高度字段
     * @param flagMergeThr 合并阈值
     * @param rasterizationFlags 光栅化标志
     * @param rasterizationMasks 光栅化掩码
     */
    public static void rasterizeTriangles(RecastContext ctx, 
                                        float[] verts, int nv,
                                        int[] tris, byte[] areas, int nt,
                                        Heightfield solid, 
                                        int flagMergeThr,
                                        int rasterizationFlags,
                                        int[] rasterizationMasks) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_RASTERIZE_TRIANGLES);
        }
        
        float ics = 1.0f / solid.cs;
        float ich = 1.0f / solid.ch;
        
        // 光栅化三角形
        for (int i = 0; i < nt; i++) {
            int triOffset = i * 3;
            float[] v0 = {verts[tris[triOffset] * 3], 
                         verts[tris[triOffset] * 3 + 1], 
                         verts[tris[triOffset] * 3 + 2]};
            float[] v1 = {verts[tris[triOffset + 1] * 3], 
                         verts[tris[triOffset + 1] * 3 + 1], 
                         verts[tris[triOffset + 1] * 3 + 2]};
            float[] v2 = {verts[tris[triOffset + 2] * 3], 
                         verts[tris[triOffset + 2] * 3 + 1], 
                         verts[tris[triOffset + 2] * 3 + 2]};
            
            // 跳过空区域的三角形
            if (areas[i] == RC_NULL_AREA) continue;
            
            rasterizeTriangle(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax,
                             solid.cs, ics, ich, flagMergeThr,
                             rasterizationFlags, rasterizationMasks);
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_RASTERIZE_TRIANGLES);
        }
    }
    
    /**
     * 光栅化多个三角形（short索引）
     * @param ctx 构建上下文
     * @param verts 顶点数组 [(x, y, z) * nv]
     * @param nv 顶点数量
     * @param tris 三角形索引数组 [(a, b, c) * nt]
     * @param areas 区域id数组 [nt]
     * @param nt 三角形数量
     * @param solid 高度字段
     * @param flagMergeThr 合并阈值
     * @param rasterizationFlags 光栅化标志
     * @param rasterizationMasks 光栅化掩码
     */
    public static void rasterizeTriangles(RecastContext ctx, 
                                        float[] verts, int nv,
                                        short[] tris, byte[] areas, int nt,
                                        Heightfield solid, 
                                        int flagMergeThr,
                                        int rasterizationFlags,
                                        int[] rasterizationMasks) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_RASTERIZE_TRIANGLES);
        }
        
        float ics = 1.0f / solid.cs;
        float ich = 1.0f / solid.ch;
        
        // 光栅化三角形
        for (int i = 0; i < nt; i++) {
            int triOffset = i * 3;
            float[] v0 = {verts[tris[triOffset] * 3], 
                         verts[tris[triOffset] * 3 + 1], 
                         verts[tris[triOffset] * 3 + 2]};
            float[] v1 = {verts[tris[triOffset + 1] * 3], 
                         verts[tris[triOffset + 1] * 3 + 1], 
                         verts[tris[triOffset + 1] * 3 + 2]};
            float[] v2 = {verts[tris[triOffset + 2] * 3], 
                         verts[tris[triOffset + 2] * 3 + 1], 
                         verts[tris[triOffset + 2] * 3 + 2]};
            
            // 跳过空区域的三角形
            if (areas[i] == RC_NULL_AREA) continue;
            
            rasterizeTriangle(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax,
                             solid.cs, ics, ich, flagMergeThr,
                             rasterizationFlags, rasterizationMasks);
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_RASTERIZE_TRIANGLES);
        }
    }
    
    /**
     * 计算高度字段中的span数量
     * @param ctx 构建上下文
     * @param hf 高度字段
     * @return span数量
     */
    public static int countSpans(RecastContext ctx, Heightfield hf) {
        if (hf.width > 0xffff || hf.height > 0xffff) {
            return 0;
        }
        
        int numSpans = 0;
        
        // 计算已分配的span数量
        for (SpanPool pool = hf.pools; pool != null; pool = pool.next) {
            numSpans += RC_SPANS_PER_POOL;
        }
        
        // 减去空闲span数量
        for (Span s = hf.freelist; s != null; s = s.next) {
            numSpans--;
        }
        
        return numSpans;
    }
    
    /**
     * 缓存高度字段中的所有span
     * @param ctx 构建上下文
     * @param hf 高度字段
     * @param cachedSpans 输出缓存数组
     */
    public static void cacheSpans(RecastContext ctx, Heightfield hf, SpanCache[] cachedSpans) {
        int cachedIndex = 0;
        
        for (int iz = 0; iz < hf.height; iz++) {
            for (int ix = 0; ix < hf.width; ix++) {
                Span s = hf.getSpan(ix, iz);
                while (s != null) {
                    if (cachedIndex < cachedSpans.length) {
                        cachedSpans[cachedIndex] = new SpanCache();
                        cachedSpans[cachedIndex].x = ix;
                        cachedSpans[cachedIndex].y = iz;
                        cachedSpans[cachedIndex].data = s.data;
                        cachedIndex++;
                    }
                    s = s.next;
                }
            }
        }
    }
} 