package odin.recast.algorithms;

import odin.recast.core.RecastHeightfield.Heightfield;
import odin.recast.core.RecastHeightfield.CompactHeightfield;
import odin.recast.core.RecastHeightfield.CompactCell;
import odin.recast.core.RecastHeightfield.CompactSpan;
import odin.recast.core.RecastSpan.Span;
import odin.recast.config.RecastEnums.TimerLabel;

import static odin.recast.utils.RecastMath.*;
import static odin.recast.config.RecastConstants.*;
import static odin.recast.algorithms.RecastUtils.*;

/**
 * Recast紧凑高度字段构建算法
 * 翻译自UE5 Recast.cpp中的buildCompactHeightfield相关函数
 * 
 * 负责将普通高度字段转换为紧凑高度字段，为后续算法做准备
 * 
 * @author UE5NavMesh4J
 */
public final class RecastCompactHeightfield {
    
    /** 私有构造函数防止实例化 */
    private RecastCompactHeightfield() {
        throw new UnsupportedOperationException("工具类不能被实例化");
    }
    
    /**
     * 构建紧凑高度字段
     * 
     * 这只是完全构建紧凑高度字段过程的开始。
     * 可能会应用各种过滤器，然后构建距离字段和区域。
     * 
     * @param ctx 构建上下文
     * @param walkableHeight 可行走高度
     * @param walkableClimb 可行走攀爬高度
     * @param hf 源高度字段
     * @param chf 输出紧凑高度字段
     * @return true如果成功构建
     */
    public static boolean buildCompactHeightfield(RecastContext ctx, 
                                                 int walkableHeight, 
                                                 int walkableClimb,
                                                 Heightfield hf, 
                                                 CompactHeightfield chf) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_BUILD_COMPACTHEIGHTFIELD);
        }
        
        // 提前检查是否有可行走的span
        int spanCount = getHeightFieldSpanCount(ctx, hf);
        if (spanCount == 0) {
            if (ctx != null) {
                ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_WARNING, 
                       "buildCompactHeightfield: 没有可行走的span，退出");
                ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_COMPACTHEIGHTFIELD);
            }
            return false;
        }
        
        int w = hf.width;
        int h = hf.height;
        
        // 填充头信息
        chf.width = w;
        chf.height = h;
        chf.spanCount = spanCount;
        chf.walkableHeight = walkableHeight;
        chf.walkableClimb = walkableClimb;
        chf.maxRegions = 0;
        
        rcVcopy(chf.bmin, hf.bmin);
        rcVcopy(chf.bmax, hf.bmax);
        chf.bmax[1] += walkableHeight * hf.ch;
        chf.cs = hf.cs;
        chf.ch = hf.ch;
        
        // 分配单元格数组
        try {
            chf.cells = new CompactCell[w * h];
            for (int i = 0; i < w * h; i++) {
                chf.cells[i] = new CompactCell();
            }
            
            chf.spans = new CompactSpan[spanCount];
            for (int i = 0; i < spanCount; i++) {
                chf.spans[i] = new CompactSpan();
            }
            
            chf.areas = new byte[spanCount];
            for (int i = 0; i < spanCount; i++) {
                chf.areas[i] = (byte)RC_NULL_AREA;
            }
        } catch (OutOfMemoryError e) {
            if (ctx != null) {
                ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_ERROR, 
                       "buildCompactHeightfield: 内存不足，无法分配数组");
                ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_COMPACTHEIGHTFIELD);
            }
            return false;
        }
        
        // 填充单元格和span
        int idx = 0;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                Span s = hf.getSpan(x, y);
                
                // 如果此单元格没有span，保持数据为index=0, count=0
                if (s == null) continue;
                
                CompactCell c = chf.cells[x + y * w];
                c.index = idx;
                c.count = 0;
                
                while (s != null) {
                    if (s.data.area != RC_NULL_AREA) {
                        int bot = s.data.smax;
                        int top = s.next != null ? s.next.data.smin : RC_SPAN_MAX_HEIGHT;
                        
                        chf.spans[idx].y = rcClamp(bot, 0, 0xffff);
                        chf.spans[idx].h = rcClamp(top - bot, 0, 0xff);
                        chf.areas[idx] = (byte)s.data.area;
                        
                        idx++;
                        c.count++;
                    }
                    s = s.next;
                }
            }
        }
        
        // 寻找邻居连接
        int MAX_LAYERS = RC_NOT_CONNECTED - 1;
        int tooHighNeighbor = 0;
        
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                CompactCell c = chf.cells[x + y * w];
                
                for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                    CompactSpan s = chf.spans[i];
                    
                    for (int dir = 0; dir < 4; dir++) {
                        setConnection(s, dir, RC_NOT_CONNECTED);
                        
                        int nx = x + getDirOffsetX(dir);
                        int ny = y + getDirOffsetY(dir);
                        
                        // 首先检查邻居单元格是否在范围内
                        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
                            continue;
                        }
                        
                        // 遍历邻居单元格中的span
                        CompactCell nc = chf.cells[nx + ny * w];
                        for (int k = nc.index, nk = nc.index + nc.count; k < nk; k++) {
                            CompactSpan ns = chf.spans[k];
                            int bot = rcMax(s.y, ns.y);
                            int top = rcMin(s.y + s.h, ns.y + ns.h);
                            
                            // 检查两个span是否重叠
                            if ((top - bot) >= walkableHeight && 
                                rcAbs(ns.y - s.y) <= walkableClimb) {
                                
                                // 标记连接
                                int lidx = k - nc.index;
                                if (lidx < 0 || lidx > MAX_LAYERS) {
                                    tooHighNeighbor = rcMax(tooHighNeighbor, lidx);
                                    continue;
                                }
                                setConnection(s, dir, lidx);
                                break;
                            }
                        }
                    }
                }
            }
        }
        
        if (tooHighNeighbor > MAX_LAYERS) {
            if (ctx != null) {
                ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_WARNING, 
                       "buildCompactHeightfield: 超过最大层数 %d (最大: %d)", 
                       tooHighNeighbor, MAX_LAYERS);
            }
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_COMPACTHEIGHTFIELD);
        }
        
        return true;
    }
    
    /**
     * 设置紧凑span的连接
     * @param s 紧凑span
     * @param dir 方向
     * @param i 连接索引
     */
    private static void setConnection(CompactSpan s, int dir, int i) {
        int shift = dir * 6;
        int con = s.con;
        s.con = (con & ~(0x3f << shift)) | ((i & 0x3f) << shift);
    }
    
    /**
     * 获取紧凑span的连接
     * @param s 紧凑span
     * @param dir 方向
     * @return 连接索引
     */
    public static int getConnection(CompactSpan s, int dir) {
        int shift = dir * 6;
        return (s.con >> shift) & 0x3f;
    }
    
    /**
     * 获取紧凑高度字段中相邻span的索引
     * @param chf 紧凑高度字段
     * @param x X坐标
     * @param y Y坐标
     * @param i span索引
     * @param dir 方向
     * @return 邻居span的索引，如果没有邻居则返回-1
     */
    public static int getNeighborIndex(CompactHeightfield chf, int x, int y, int i, int dir) {
        CompactSpan s = chf.spans[i];
        int connection = getConnection(s, dir);
        
        if (connection == RC_NOT_CONNECTED) {
            return -1;
        }
        
        int nx = x + getDirOffsetX(dir);
        int ny = y + getDirOffsetY(dir);
        
        if (nx < 0 || ny < 0 || nx >= chf.width || ny >= chf.height) {
            return -1;
        }
        
        CompactCell nc = chf.cells[nx + ny * chf.width];
        return nc.index + connection;
    }
    
    /**
     * 构建距离字段
     * @param ctx 构建上下文
     * @param chf 紧凑高度字段
     * @return true如果成功构建
     */
    public static boolean buildDistanceField(RecastContext ctx, CompactHeightfield chf) {
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_BUILD_DISTANCEFIELD);
        }
        
        // 分配距离数组
        try {
            chf.dist = new int[chf.spanCount];
        } catch (OutOfMemoryError e) {
            if (ctx != null) {
                ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_ERROR, 
                       "buildDistanceField: 内存不足，无法分配距离数组");
                ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_DISTANCEFIELD);
            }
            return false;
        }
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_BUILD_DISTANCEFIELD_DIST);
        }
        
        // 计算距离字段
        calculateDistanceField(chf);
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_DISTANCEFIELD_DIST);
            ctx.startTimer(TimerLabel.RC_TIMER_BUILD_DISTANCEFIELD_BLUR);
        }
        
        // 模糊距离字段
        blurDistanceField(chf);
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_DISTANCEFIELD_BLUR);
            ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_DISTANCEFIELD);
        }
        
        return true;
    }
    
    /**
     * 计算距离字段
     * @param chf 紧凑高度字段
     */
    private static void calculateDistanceField(CompactHeightfield chf) {
        int w = chf.width;
        int h = chf.height;
        
        // 初始化距离
        for (int i = 0; i < chf.spanCount; i++) {
            chf.dist[i] = 0xffff;
        }
        
        // 标记边界距离为0
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                    CompactSpan s = chf.spans[i];
                    int area = chf.areas[i] & 0xFF;
                    
                    int neighborCount = 0;
                    for (int dir = 0; dir < 4; dir++) {
                        if (getConnection(s, dir) != RC_NOT_CONNECTED) {
                            neighborCount++;
                        }
                    }
                    
                    if (neighborCount != 4) {
                        chf.dist[i] = 0;
                    } else if (area == RC_NULL_AREA) {
                        chf.dist[i] = 0;
                    }
                }
            }
        }
        
        // 传播距离
        int[] queue = new int[chf.spanCount];
        int queueHead = 0;
        int queueTail = 0;
        
        // 将边界添加到队列
        for (int i = 0; i < chf.spanCount; i++) {
            if (chf.dist[i] == 0) {
                queue[queueTail++] = i;
            }
        }
        
        while (queueHead < queueTail) {
            int idx = queue[queueHead++];
            
            // 找到这个span的坐标
            int x = -1, y = -1;
            for (int cy = 0; cy < h && y == -1; cy++) {
                for (int cx = 0; cx < w && x == -1; cx++) {
                    CompactCell c = chf.cells[cx + cy * w];
                    if (idx >= c.index && idx < c.index + c.count) {
                        x = cx;
                        y = cy;
                        break;
                    }
                }
            }
            
            if (x == -1 || y == -1) continue;
            
            CompactCell c = chf.cells[x + y * w];
            int spanIndex = idx - c.index;
            CompactSpan s = chf.spans[idx];
            
            for (int dir = 0; dir < 4; dir++) {
                if (getConnection(s, dir) == RC_NOT_CONNECTED) continue;
                
                int nx = x + getDirOffsetX(dir);
                int ny = y + getDirOffsetY(dir);
                CompactCell nc = chf.cells[nx + ny * w];
                int nni = nc.index + getConnection(s, dir);
                
                if (chf.dist[nni] > chf.dist[idx] + 2) {
                    chf.dist[nni] = chf.dist[idx] + 2;
                    queue[queueTail++] = nni;
                }
            }
        }
    }
    
    /**
     * 模糊距离字段
     * @param chf 紧凑高度字段
     */
    private static void blurDistanceField(CompactHeightfield chf) {
        int w = chf.width;
        int h = chf.height;
        
        int[] tmp = new int[chf.spanCount];
        System.arraycopy(chf.dist, 0, tmp, 0, chf.spanCount);
        
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                    CompactSpan s = chf.spans[i];
                    
                    int totalDist = tmp[i];
                    int totalWeight = 1;
                    
                    for (int dir = 0; dir < 4; dir++) {
                        if (getConnection(s, dir) == RC_NOT_CONNECTED) continue;
                        
                        int nx = x + getDirOffsetX(dir);
                        int ny = y + getDirOffsetY(dir);
                        CompactCell nc = chf.cells[nx + ny * w];
                        int nni = nc.index + getConnection(s, dir);
                        
                        totalDist += tmp[nni];
                        totalWeight++;
                    }
                    
                    chf.dist[i] = (totalDist + totalWeight / 2) / totalWeight;
                }
            }
        }
    }
} 