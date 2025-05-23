package odin.recast.algorithms;

import odin.recast.core.RecastHeightfield.CompactHeightfield;
import odin.recast.core.RecastHeightfield.CompactCell;
import odin.recast.core.RecastHeightfield.CompactSpan;
import odin.recast.config.RecastConfig.BorderSize;
import odin.recast.config.RecastEnums.TimerLabel;
import odin.recast.config.RecastEnums.RegionPartitioning;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

import static odin.recast.utils.RecastMath.*;
import static odin.recast.config.RecastConstants.*;
import static odin.recast.algorithms.RecastCompactHeightfield.*;

/**
 * Recast区域分割算法
 * 翻译自UE5 RecastRegion.cpp
 * 
 * 负责将紧凑高度字段分割为连通的区域，为轮廓生成做准备
 * 
 * @author UE5NavMesh4J
 */
public final class RecastRegion {
    
    /** 私有构造函数防止实例化 */
    private RecastRegion() {
        throw new UnsupportedOperationException("工具类不能被实例化");
    }
    
    /**
     * 区域数据结构
     */
    public static class Region {
        public int chunkId;              // 包含此区域的块ID
        public int spanCount;            // 属于此区域的span数量
        public int id;                   // 区域ID
        public int areaType;             // 区域类型
        public boolean remap;            // 是否重新映射
        public boolean visited;          // 是否已访问
        public List<Integer> connections; // 连接列表
        public List<Integer> floors;     // 楼层列表
        
        public Region(int id) {
            this.id = id;
            this.chunkId = 0;
            this.spanCount = 0;
            this.areaType = 0;
            this.remap = false;
            this.visited = false;
            this.connections = new ArrayList<>();
            this.floors = new ArrayList<>();
        }
    }
    
    /**
     * 计算距离字段（改进版）
     * @param chf 紧凑高度字段
     * @param src 源距离数组
     * @return 最大距离
     */
    private static int calculateDistanceField(CompactHeightfield chf, int[] src) {
        int w = chf.width;
        int h = chf.height;
        
        // 初始化距离
        Arrays.fill(src, 0xffff);
        
        // 标记边界单元格
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                    CompactSpan s = chf.spans[i];
                    int area = chf.areas[i] & 0xFF;
                    
                    int nc = 0;
                    for (int dir = 0; dir < 4; dir++) {
                        if (getConnection(s, dir) != RC_NOT_CONNECTED) {
                            int ax = x + getDirOffsetX(dir);
                            int ay = y + getDirOffsetY(dir);
                            int ai = chf.cells[ax + ay * w].index + getConnection(s, dir);
                            if (area == (chf.areas[ai] & 0xFF)) {
                                nc++;
                            }
                        }
                    }
                    if (nc != 4) {
                        src[i] = 0;
                    }
                }
            }
        }
        
        // 第一遍扫描（从左上到右下）
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                    CompactSpan s = chf.spans[i];
                    
                    // 检查左邻居 (-1,0)
                    if (getConnection(s, 0) != RC_NOT_CONNECTED) {
                        int ax = x + getDirOffsetX(0);
                        int ay = y + getDirOffsetY(0);
                        int ai = chf.cells[ax + ay * w].index + getConnection(s, 0);
                        if (src[ai] + 2 < src[i]) {
                            src[i] = src[ai] + 2;
                        }
                    }
                    
                    // 检查上邻居 (0,-1)
                    if (getConnection(s, 3) != RC_NOT_CONNECTED) {
                        int ax = x + getDirOffsetX(3);
                        int ay = y + getDirOffsetY(3);
                        int ai = chf.cells[ax + ay * w].index + getConnection(s, 3);
                        if (src[ai] + 2 < src[i]) {
                            src[i] = src[ai] + 2;
                        }
                    }
                }
            }
        }
        
        // 第二遍扫描（从右下到左上）
        for (int y = h - 1; y >= 0; y--) {
            for (int x = w - 1; x >= 0; x--) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                    CompactSpan s = chf.spans[i];
                    
                    // 检查右邻居 (1,0)
                    if (getConnection(s, 2) != RC_NOT_CONNECTED) {
                        int ax = x + getDirOffsetX(2);
                        int ay = y + getDirOffsetY(2);
                        int ai = chf.cells[ax + ay * w].index + getConnection(s, 2);
                        if (src[ai] + 2 < src[i]) {
                            src[i] = src[ai] + 2;
                        }
                    }
                    
                    // 检查下邻居 (0,1)
                    if (getConnection(s, 1) != RC_NOT_CONNECTED) {
                        int ax = x + getDirOffsetX(1);
                        int ay = y + getDirOffsetY(1);
                        int ai = chf.cells[ax + ay * w].index + getConnection(s, 1);
                        if (src[ai] + 2 < src[i]) {
                            src[i] = src[ai] + 2;
                        }
                    }
                }
            }
        }
        
        // 计算最大距离
        int maxDist = 0;
        for (int i = 0; i < chf.spanCount; i++) {
            maxDist = rcMax(src[i], maxDist);
        }
        
        return maxDist;
    }
    
    /**
     * 盒式模糊
     * @param chf 紧凑高度字段
     * @param thr 阈值
     * @param src 源数组
     * @param dst 目标数组
     * @return 目标数组
     */
    private static int[] boxBlur(CompactHeightfield chf, int thr, int[] src, int[] dst) {
        int w = chf.width;
        int h = chf.height;
        
        thr *= 2;
        
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                    CompactSpan s = chf.spans[i];
                    int cd = src[i];
                    
                    if (cd <= thr) {
                        dst[i] = cd;
                        continue;
                    }
                    
                    int d = cd;
                    for (int dir = 0; dir < 4; dir++) {
                        if (getConnection(s, dir) != RC_NOT_CONNECTED) {
                            int ax = x + getDirOffsetX(dir);
                            int ay = y + getDirOffsetY(dir);
                            int ai = chf.cells[ax + ay * w].index + getConnection(s, dir);
                            d += src[ai];
                        } else {
                            d += cd;
                        }
                    }
                    dst[i] = (d + 2) / 5;
                }
            }
        }
        
        return dst;
    }
    
    /**
     * 洪填区域
     * @param x X坐标
     * @param y Y坐标
     * @param i span索引
     * @param level 水位
     * @param r 区域ID
     * @param chf 紧凑高度字段
     * @param srcReg 源区域数组
     * @param srcDist 源距离数组
     * @param stack 栈
     * @return 是否成功
     */
    private static boolean floodRegion(int x, int y, int i, int level, int r,
                                     CompactHeightfield chf, int[] srcReg, int[] srcDist,
                                     List<Integer> stack) {
        int w = chf.width;
        int area = chf.areas[i] & 0xFF;
        
        // 洪填标记区域
        stack.clear();
        stack.add(x);
        stack.add(y);
        stack.add(i);
        srcReg[i] = r;
        srcDist[i] = 0;
        
        int lev = level >= 2 ? level - 2 : 0;
        int count = 0;
        
        while (!stack.isEmpty()) {
            int ci = stack.remove(stack.size() - 1);
            int cy = stack.remove(stack.size() - 1);
            int cx = stack.remove(stack.size() - 1);
            
            CompactSpan cs = chf.spans[ci];
            
            // 检查任何邻居是否已经有有效的区域设置
            int ar = 0;
            for (int dir = 0; dir < 4; dir++) {
                if (getConnection(cs, dir) != RC_NOT_CONNECTED) {
                    int ax = cx + getDirOffsetX(dir);
                    int ay = cy + getDirOffsetY(dir);
                    int ai = chf.cells[ax + ay * w].index + getConnection(cs, dir);
                    
                    if ((chf.areas[ai] & 0xFF) != area) continue;
                    
                    int nr = srcReg[ai];
                    if ((nr & RC_BORDER_REG) != 0) continue; // 不考虑边界
                    if (nr != 0 && nr != r) {
                        ar = nr;
                    }
                }
            }
            
            if (ar != 0) {
                srcReg[ci] = 0;
                continue;
            }
            
            count++;
            
            // 扩展邻居
            for (int dir = 0; dir < 4; dir++) {
                if (getConnection(cs, dir) != RC_NOT_CONNECTED) {
                    int ax = cx + getDirOffsetX(dir);
                    int ay = cy + getDirOffsetY(dir);
                    int ai = chf.cells[ax + ay * w].index + getConnection(cs, dir);
                    
                    if ((chf.areas[ai] & 0xFF) != area) continue;
                    
                    if (chf.dist[ai] >= lev && srcReg[ai] == 0) {
                        srcReg[ai] = r;
                        srcDist[ai] = 0;
                        stack.add(ax);
                        stack.add(ay);
                        stack.add(ai);
                    }
                }
            }
        }
        
        return count > 0;
    }
    
    /**
     * 构建区域（分水岭算法）
     * @param ctx 构建上下文
     * @param chf 紧凑高度字段
     * @param borderSize 边界大小
     * @param minRegionArea 最小区域面积
     * @param mergeRegionArea 合并区域面积
     * @return true如果成功构建
     */
    public static boolean buildRegions(RecastContext ctx, CompactHeightfield chf,
                                     BorderSize borderSize, int minRegionArea, int mergeRegionArea) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_BUILD_REGIONS);
        }
        
        int w = chf.width;
        int h = chf.height;
        
        try {
            // 分配工作数组
            int[] buf = new int[chf.spanCount * 4];
            
            int[] src = Arrays.copyOfRange(buf, 0, chf.spanCount);
            int[] srcReg = Arrays.copyOfRange(buf, chf.spanCount, chf.spanCount * 2);
            int[] srcDist = Arrays.copyOfRange(buf, chf.spanCount * 2, chf.spanCount * 3);
            int[] dstReg = Arrays.copyOfRange(buf, chf.spanCount * 3, chf.spanCount * 4);
            
            if (ctx != null) {
                ctx.startTimer(TimerLabel.RC_TIMER_BUILD_REGIONS_WATERSHED);
            }
            
            // 计算距离字段
            int maxDist = calculateDistanceField(chf, src);
            
            // 模糊距离字段
            int[] tmp = new int[chf.spanCount];
            boxBlur(chf, 1, src, tmp);
            System.arraycopy(tmp, 0, src, 0, chf.spanCount);
            
            // 分水岭分割
            int regionId = 1;
            int level = (maxDist + 1) & ~1;
            List<Integer> stack = new ArrayList<>();
            
            // 初始化区域数组
            Arrays.fill(srcReg, 0);
            Arrays.fill(srcDist, 0);
            
            while (level > 0) {
                level = level >= 2 ? level - 2 : 0;
                
                if (ctx != null) {
                    ctx.startTimer(TimerLabel.RC_TIMER_BUILD_REGIONS_EXPAND);
                }
                
                // 寻找新的区域种子
                for (int y = 0; y < h; y++) {
                    for (int x = 0; x < w; x++) {
                        CompactCell c = chf.cells[x + y * w];
                        for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                            if (chf.dist[i] >= level && srcReg[i] == 0 && 
                                (chf.areas[i] & 0xFF) != RC_NULL_AREA) {
                                
                                if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, stack)) {
                                    regionId++;
                                }
                            }
                        }
                    }
                }
                
                if (ctx != null) {
                    ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_REGIONS_EXPAND);
                }
            }
            
            // 扩展区域
            expandRegions(3, chf, srcReg, srcDist, dstReg, stack);
            
            // 复制结果到chf.regs
            chf.regs = new int[chf.spanCount];
            System.arraycopy(srcReg, 0, chf.regs, 0, chf.spanCount);
            chf.maxRegions = regionId;
            
            if (ctx != null) {
                ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_REGIONS_WATERSHED);
            }
            
            // 过滤小区域
            if (ctx != null) {
                ctx.startTimer(TimerLabel.RC_TIMER_BUILD_REGIONS_FILTER);
            }
            
            filterSmallRegions(ctx, minRegionArea, mergeRegionArea, chf);
            
            if (ctx != null) {
                ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_REGIONS_FILTER);
            }
            
        } catch (OutOfMemoryError e) {
            if (ctx != null) {
                ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_ERROR, 
                       "buildRegions: 内存不足");
            }
            return false;
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_REGIONS);
        }
        
        return true;
    }
    
    /**
     * 扩展区域
     * @param maxIter 最大迭代次数
     * @param chf 紧凑高度字段
     * @param srcReg 源区域数组
     * @param srcDist 源距离数组
     * @param dstReg 目标区域数组
     * @param stack 栈
     */
    private static void expandRegions(int maxIter, CompactHeightfield chf,
                                    int[] srcReg, int[] srcDist, int[] dstReg,
                                    List<Integer> stack) {
        int w = chf.width;
        int h = chf.height;
        
        // 寻找候选单元格
        stack.clear();
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                    if (srcReg[i] == 0 && (chf.areas[i] & 0xFF) != RC_NULL_AREA) {
                        stack.add(x);
                        stack.add(y);
                        stack.add(i);
                    }
                }
            }
        }
        
        int iter = 0;
        while (!stack.isEmpty() && iter < maxIter) {
            int failed = 0;
            
            System.arraycopy(srcReg, 0, dstReg, 0, chf.spanCount);
            
            for (int j = 0; j < stack.size(); j += 3) {
                int x = stack.get(j);
                int y = stack.get(j + 1);
                int i = stack.get(j + 2);
                
                if (i < 0) {
                    failed++;
                    continue;
                }
                
                int r = srcReg[i];
                int d2 = 0xffff;
                int area = chf.areas[i] & 0xFF;
                CompactSpan s = chf.spans[i];
                
                for (int dir = 0; dir < 4; dir++) {
                    if (getConnection(s, dir) == RC_NOT_CONNECTED) continue;
                    
                    int ax = x + getDirOffsetX(dir);
                    int ay = y + getDirOffsetY(dir);
                    int ai = chf.cells[ax + ay * w].index + getConnection(s, dir);
                    
                    if ((chf.areas[ai] & 0xFF) != area) continue;
                    
                    if (srcReg[ai] > 0 && (srcReg[ai] & RC_BORDER_REG) == 0) {
                        if (srcDist[ai] + 2 < d2) {
                            r = srcReg[ai];
                            d2 = srcDist[ai] + 2;
                        }
                    }
                }
                
                if (r != 0) {
                    stack.set(j + 2, -1); // 标记为已使用
                    dstReg[i] = r;
                    if (srcDist.length > i) {
                        srcDist[i] = d2;
                    }
                } else {
                    failed++;
                }
            }
            
            // 交换源和目标
            int[] tmp = srcReg;
            srcReg = dstReg;
            dstReg = tmp;
            
            if (failed * 3 == stack.size()) break;
            
            iter++;
        }
    }
    
    /**
     * 过滤小区域
     * @param ctx 构建上下文
     * @param minRegionArea 最小区域面积
     * @param mergeRegionArea 合并区域面积
     * @param chf 紧凑高度字段
     */
    private static void filterSmallRegions(RecastContext ctx, int minRegionArea, 
                                         int mergeRegionArea, CompactHeightfield chf) {
        // 计算区域大小
        int[] regionAreas = new int[chf.maxRegions];
        
        // 计算每个区域的大小
        for (int i = 0; i < chf.spanCount; i++) {
            int r = chf.regs[i];
            if ((r & RC_BORDER_REG) == 0 && r != 0) {
                regionAreas[r]++;
            }
        }
        
        // 标记小区域
        for (int i = 0; i < chf.spanCount; i++) {
            int r = chf.regs[i];
            if ((r & RC_BORDER_REG) == 0 && r != 0 && regionAreas[r] < minRegionArea) {
                chf.regs[i] = 0;
            }
        }
    }
    
    /**
     * 获取方向偏移X
     * @param dir 方向
     * @return X偏移
     */
    private static int getDirOffsetX(int dir) {
        int[] offset = {-1, 0, 1, 0};
        return offset[dir & 0x3];
    }
    
    /**
     * 获取方向偏移Y  
     * @param dir 方向
     * @return Y偏移
     */
    private static int getDirOffsetY(int dir) {
        int[] offset = {0, 1, 0, -1};
        return offset[dir & 0x3];
    }
} 