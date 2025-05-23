package odin.recast.algorithms;

import odin.recast.core.RecastHeightfield.Heightfield;
import odin.recast.core.RecastSpan.Span;
import odin.recast.config.RecastEnums.TimerLabel;
import odin.recast.config.RecastEnums.NeighborSlopeFilterMode;

import static odin.recast.utils.RecastMath.*;
import static odin.recast.config.RecastConstants.*;
import static odin.recast.algorithms.RecastUtils.*;

/**
 * Recast过滤算法
 * 翻译自UE5 RecastFilter.cpp
 * 
 * 负责过滤和清理光栅化后的高度字段，去除不合适的span
 * 
 * @author UE5NavMesh4J
 */
public final class RecastFilter {
    
    /** 私有构造函数防止实例化 */
    private RecastFilter() {
        throw new UnsupportedOperationException("工具类不能被实例化");
    }
    
    /**
     * 过滤低矮悬挂的可行走障碍物
     * 
     * 允许形成可行走区域，这些区域可以流过低矮物体（如路缘）和上楼梯等结构。
     * 如果两个相邻的span满足条件：rcAbs(currentSpan.smax - neighborSpan.smax) < walkableClimb，
     * 则它们被视为可行走的。
     * 
     * @param ctx 构建上下文
     * @param walkableClimb 可行走攀爬高度
     * @param solid 高度字段
     */
    public static void filterLowHangingWalkableObstacles(RecastContext ctx, 
                                                        int walkableClimb, 
                                                        Heightfield solid) {
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_FILTER_LOW_OBSTACLES);
        }
        
        int w = solid.width;
        int h = solid.height;
        
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                Span ps = null;
                boolean previousWalkable = false;
                int previousArea = RC_NULL_AREA;
                
                Span s = solid.getSpan(x, y);
                while (s != null) {
                    boolean walkable = s.data.area != RC_NULL_AREA;
                    
                    // 如果当前span不可行走，但下方有可行走的span，
                    // 并且高度差在可攀爬范围内，则标记为可行走
                    if (!walkable && previousWalkable) {
                        if (rcAbs(s.data.smax - ps.data.smax) <= walkableClimb) {
                            s.data.area = previousArea;
                        }
                    }
                    
                    // 复制可行走标志，避免传播到多个不可行走对象之外
                    previousWalkable = walkable;
                    previousArea = s.data.area;
                    
                    ps = s;
                    s = s.next;
                }
            }
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_FILTER_LOW_OBSTACLES);
        }
    }
    
    /**
     * 过滤边缘span的实现
     * @param ctx 构建上下文
     * @param walkableHeight 可行走高度
     * @param walkableClimb 可行走攀爬高度
     * @param neighborSlopeFilterMode 邻居坡度过滤模式
     * @param maxStepFromWalkableSlope 可行走坡度的最大步高
     * @param ch 单元格高度
     * @param filterLedgeSpansAtY 过滤的Y坐标
     * @param solid 高度字段
     */
    private static void filterLedgeSpansImpl(RecastContext ctx, 
                                           int walkableHeight, 
                                           int walkableClimb,
                                           NeighborSlopeFilterMode neighborSlopeFilterMode, 
                                           float maxStepFromWalkableSlope, 
                                           float ch,
                                           int filterLedgeSpansAtY, 
                                           Heightfield solid) {
        
        int w = solid.width;
        int h = solid.height;
        int MAX_HEIGHT = RC_SPAN_MAX_HEIGHT;
        
        int maxStepFor2CellsVx = (int)rcCeil(2 * maxStepFromWalkableSlope / ch);
        
        // 标记边界span
        for (int x = 0; x < w; x++) {
            Span s = solid.getSpan(x, filterLedgeSpansAtY);
            while (s != null) {
                // 跳过不可行走的span
                if (s.data.area == RC_NULL_AREA) {
                    s = s.next;
                    continue;
                }
                
                int bot = s.data.smax;
                int top = s.next != null ? s.next.data.smin : MAX_HEIGHT;
                
                // 寻找邻居的最小高度
                int minh = MAX_HEIGHT;
                
                // 可访问邻居的最小和最大高度
                int asmin = s.data.smax;
                int asmax = s.data.smax;
                
                for (int dir = 0; dir < 4; dir++) {
                    int dx = x + getDirOffsetX(dir);
                    int dy = filterLedgeSpansAtY + getDirOffsetY(dir);
                    
                    // 跳过超出边界的邻居
                    if (dx < 0 || dy < 0 || dx >= w || dy >= h) {
                        minh = rcMin(minh, -walkableClimb - bot);
                        continue;
                    }
                    
                    // 从负无穷到第一个span
                    Span ns = solid.getSpan(dx, dy);
                    int nbot = -walkableClimb;
                    int ntop = ns != null ? ns.data.smin : MAX_HEIGHT;
                    
                    // 如果span之间的间隙太小，跳过邻居
                    if (rcMin(top, ntop) - rcMax(bot, nbot) > walkableHeight) {
                        minh = rcMin(minh, nbot - bot);
                    }
                    
                    // 其余的span
                    while (ns != null) {
                        nbot = ns.data.smax;
                        ntop = ns.next != null ? ns.next.data.smin : MAX_HEIGHT;
                        
                        // 如果span之间的间隙太小，跳过邻居
                        if (rcMin(top, ntop) - rcMax(bot, nbot) > walkableHeight) {
                            minh = rcMin(minh, nbot - bot);
                            
                            // 寻找可访问邻居的最小/最大高度
                            if (neighborSlopeFilterMode != NeighborSlopeFilterMode.RC_SLOPE_FILTER_NONE && 
                                rcAbs(nbot - bot) <= walkableClimb) {
                                if (nbot < asmin) asmin = nbot;
                                if (nbot > asmax) asmax = nbot;
                            }
                        }
                        ns = ns.next;
                    }
                }
                
                // 如果到任何邻居span的跌落小于walkableClimb，当前span接近边缘
                if (minh < -walkableClimb) {
                    s.data.area = RC_NULL_AREA;
                }
                // 如果所有邻居之间的差异太大，我们处于陡坡，标记span为边缘
                else if (neighborSlopeFilterMode == NeighborSlopeFilterMode.RC_SLOPE_FILTER_RECAST && 
                         (asmax - asmin) > walkableClimb) {
                    s.data.area = RC_NULL_AREA;
                }
                // 比较最小值和最大值之间的步高与2个span的均匀坡度的最大步高
                else if (neighborSlopeFilterMode == NeighborSlopeFilterMode.RC_SLOPE_FILTER_USE_HEIGHT_FROM_WALKABLE_SLOPE && 
                         (asmax - asmin) > maxStepFor2CellsVx) {
                    s.data.area = RC_NULL_AREA;
                }
                
                s = s.next;
            }
        }
    }
    
    /**
     * 过滤边缘span
     * 
     * 边缘是指一个或多个邻居的最大值距离当前span的最大值超过walkableClimb的span。
     * 这种方法消除了保守体素化过高估计的影响，使得生成的网格不会在边缘悬崖上有悬挂区域。
     * 
     * @param ctx 构建上下文
     * @param walkableHeight 可行走高度
     * @param walkableClimb 可行走攀爬高度
     * @param neighborSlopeFilterMode 邻居坡度过滤模式
     * @param maxStepFromWalkableSlope 可行走坡度的最大步高
     * @param ch 单元格高度
     * @param solid 高度字段
     */
    public static void filterLedgeSpans(RecastContext ctx, 
                                       int walkableHeight, 
                                       int walkableClimb,
                                       NeighborSlopeFilterMode neighborSlopeFilterMode, 
                                       float maxStepFromWalkableSlope, 
                                       float ch,
                                       Heightfield solid) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_FILTER_BORDER);
        }
        
        int h = solid.height;
        
        // 标记边界span
        for (int y = 0; y < h; y++) {
            filterLedgeSpansImpl(ctx, walkableHeight, walkableClimb, 
                               neighborSlopeFilterMode, maxStepFromWalkableSlope, ch, 
                               y, solid);
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_FILTER_BORDER);
        }
    }
    
    /**
     * 过滤边缘span（指定范围）
     * @param ctx 构建上下文
     * @param walkableHeight 可行走高度
     * @param walkableClimb 可行走攀爬高度
     * @param neighborSlopeFilterMode 邻居坡度过滤模式
     * @param maxStepFromWalkableSlope 可行走坡度的最大步高
     * @param ch 单元格高度
     * @param yStart 开始Y坐标
     * @param maxYProcess 最大处理Y范围
     * @param solid 高度字段
     */
    public static void filterLedgeSpans(RecastContext ctx, 
                                       int walkableHeight, 
                                       int walkableClimb,
                                       NeighborSlopeFilterMode neighborSlopeFilterMode, 
                                       float maxStepFromWalkableSlope, 
                                       float ch,
                                       int yStart, 
                                       int maxYProcess,
                                       Heightfield solid) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_FILTER_BORDER);
        }
        
        int h = rcMin(yStart + maxYProcess, solid.height);
        
        for (int y = yStart; y < h; y++) {
            filterLedgeSpansImpl(ctx, walkableHeight, walkableClimb, 
                               neighborSlopeFilterMode, maxStepFromWalkableSlope, ch, 
                               y, solid);
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_FILTER_BORDER);
        }
    }
    
    /**
     * 过滤低高度的可行走span
     * 
     * 对于此过滤器，span上方的净空是从span的最大值到下一个更高span的最小值的距离（同一网格列）。
     * 
     * @param ctx 构建上下文
     * @param walkableHeight 可行走高度
     * @param solid 高度字段
     */
    public static void filterWalkableLowHeightSpans(RecastContext ctx, 
                                                   int walkableHeight, 
                                                   Heightfield solid) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_FILTER_WALKABLE);
        }
        
        int w = solid.width;
        int h = solid.height;
        int MAX_HEIGHT = RC_SPAN_MAX_HEIGHT;
        
        // 移除没有足够空间让代理站立的span的可行走标志
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                Span s = solid.getSpan(x, y);
                while (s != null) {
                    int bot = s.data.smax;
                    int top = s.next != null ? s.next.data.smin : MAX_HEIGHT;
                    
                    if ((top - bot) < walkableHeight) {
                        s.data.area = RC_NULL_AREA;
                    }
                    
                    s = s.next;
                }
            }
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_FILTER_WALKABLE);
        }
    }
    
    /**
     * 过滤低高度span序列
     * 
     * UE5特有的实现：只在有效span下方留下单个低span，
     * 或者在它们之间留下walkableHeight空间后
     * 
     * @param ctx 构建上下文
     * @param walkableHeight 可行走高度
     * @param solid 高度字段
     */
    public static void filterWalkableLowHeightSpansSequences(RecastContext ctx, 
                                                            int walkableHeight, 
                                                            Heightfield solid) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_FILTER_WALKABLE);
        }
        
        int w = solid.width;
        int h = solid.height;
        
        final int MAX_SPANS = 64;
        CompactSpanData[] spanList = new CompactSpanData[MAX_SPANS];
        for (int i = 0; i < MAX_SPANS; i++) {
            spanList[i] = new CompactSpanData();
        }
        
        // 移除没有足够空间让代理站立的span的可行走标志
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                // 构建紧凑span列表，需要从上到下迭代
                int numSpans = 0;
                Span s = solid.getSpan(x, y);
                while (s != null && numSpans < MAX_SPANS) {
                    int bot = s.data.smax;
                    int top = s.next != null ? s.next.data.smin : RC_SPAN_MAX_HEIGHT;
                    
                    spanList[numSpans].y = rcClamp(bot, 0, RC_SPAN_MAX_HEIGHT);
                    spanList[numSpans].h = rcClamp(top - bot, 0, 0xff);
                    spanList[numSpans].reg = s.data.area;
                    
                    numSpans++;
                    s = s.next;
                }
                
                int nextAllowedBase = RC_SPAN_MAX_HEIGHT;
                for (int idx = numSpans - 1; idx >= 0; idx--) {
                    if (spanList[idx].h < walkableHeight) {
                        if (spanList[idx].y < nextAllowedBase) {
                            nextAllowedBase = rcMax(0, spanList[idx].y - walkableHeight);
                        } else {
                            spanList[idx].reg = RC_NULL_AREA;
                        }
                    } else if (spanList[idx].reg != RC_NULL_AREA) {
                        nextAllowedBase = spanList[idx].y;
                    }
                }
                
                // 将结果写回span
                int spanIdx = 0;
                s = solid.getSpan(x, y);
                while (s != null && spanIdx < MAX_SPANS) {
                    s.data.area = spanList[spanIdx].reg;
                    spanIdx++;
                    s = s.next;
                }
            }
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_FILTER_WALKABLE);
        }
    }
    
    /**
     * 紧凑span数据（用于序列过滤）
     */
    private static class CompactSpanData {
        public int y;
        public int h;
        public int reg;
        
        public CompactSpanData() {
            this.y = 0;
            this.h = 0;
            this.reg = RC_NULL_AREA;
        }
    }
} 