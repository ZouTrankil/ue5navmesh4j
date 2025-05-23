package odin.detour.core;

import static odin.detour.config.DetourConstants.*;
import static odin.detour.utils.DetourCommon.*;

/**
 * Detour查询过滤器
 * 翻译自UE5 DetourNavMeshQuery.h中的dtQueryFilter
 * 
 * 定义导航网格查询操作的多边形过滤和遍历成本
 * 
 * @author UE5NavMesh4J
 */
public class DetourQueryFilter {
    
    /** 每个区域类型的成本 */
    private float[] areaCost = new float[DT_MAX_AREAS];
    
    /** 进入区域类型的固定成本 */
    private float[] areaFixedCost = new float[DT_MAX_AREAS];
    
    /** 搜索启发式比例 */
    private float heuristicScale = 1.0f;
    
    /** 最低区域成本 */
    private float lowestAreaCost = 1.0f;
    
    /** 可以访问的多边形标志 */
    private int includeFlags = 0xffff;
    
    /** 不应访问的多边形标志 */
    private int excludeFlags = 0;
    
    /** 是否为回溯模式 */
    private boolean isBacktracking = false;
    
    /** 是否应忽略已关闭的节点 */
    private boolean shouldIgnoreClosedNodes = false;
    
    /** 是否使用虚拟方法调用 */
    private boolean isVirtual = true;
    
    /**
     * 默认构造函数
     */
    public DetourQueryFilter() {
        this(true);
    }
    
    /**
     * 构造函数
     * @param isVirtual 是否使用虚拟方法调用
     */
    public DetourQueryFilter(boolean isVirtual) {
        this.isVirtual = isVirtual;
        
        // 初始化默认区域成本
        for (int i = 0; i < DT_MAX_AREAS; i++) {
            areaCost[i] = 1.0f;
            areaFixedCost[i] = 0.0f;
        }
    }
    
    /**
     * 检查多边形是否可以访问
     * @param ref 多边形引用ID
     * @param tile 包含多边形的瓦片
     * @param poly 要测试的多边形
     * @return true如果可以访问
     */
    public boolean passFilter(long ref, DetourNavMesh.MeshTile tile, DetourNavMesh.Poly poly) {
        if (!isVirtual) {
            return passInlineFilter(ref, tile, poly);
        } else {
            return passVirtualFilter(ref, tile, poly);
        }
    }
    
    /**
     * 内联过滤器实现
     */
    protected boolean passInlineFilter(long ref, DetourNavMesh.MeshTile tile, DetourNavMesh.Poly poly) {
        return (poly.flags & includeFlags) != 0 && 
               (poly.flags & excludeFlags) == 0 &&
               areaCost[poly.area] < DT_UNWALKABLE_POLY_COST &&
               areaFixedCost[poly.area] < DT_UNWALKABLE_POLY_COST;
    }
    
    /**
     * 虚拟过滤器实现（默认调用内联过滤器）
     */
    protected boolean passVirtualFilter(long ref, DetourNavMesh.MeshTile tile, DetourNavMesh.Poly poly) {
        return passInlineFilter(ref, tile, poly);
    }
    
    /**
     * 计算从线段开始到结束的移动成本
     * @param pa 前一个和当前多边形边缘上的起始位置
     * @param pb 当前和下一个多边形边缘上的结束位置
     * @param prevRef 前一个多边形的引用ID
     * @param prevTile 包含前一个多边形的瓦片
     * @param prevPoly 前一个多边形
     * @param curRef 当前多边形的引用ID
     * @param curTile 包含当前多边形的瓦片
     * @param curPoly 当前多边形
     * @param nextRef 下一个多边形的引用ID
     * @param nextTile 包含下一个多边形的瓦片
     * @param nextPoly 下一个多边形
     * @return 移动成本
     */
    public float getCost(float[] pa, float[] pb,
                        long prevRef, DetourNavMesh.MeshTile prevTile, DetourNavMesh.Poly prevPoly,
                        long curRef, DetourNavMesh.MeshTile curTile, DetourNavMesh.Poly curPoly,
                        long nextRef, DetourNavMesh.MeshTile nextTile, DetourNavMesh.Poly nextPoly) {
        if (!isVirtual) {
            return getInlineCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);
        } else {
            return getVirtualCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);
        }
    }
    
    /**
     * 内联成本计算实现
     */
    protected float getInlineCost(float[] pa, float[] pb,
                                 long prevRef, DetourNavMesh.MeshTile prevTile, DetourNavMesh.Poly prevPoly,
                                 long curRef, DetourNavMesh.MeshTile curTile, DetourNavMesh.Poly curPoly,
                                 long nextRef, DetourNavMesh.MeshTile nextTile, DetourNavMesh.Poly nextPoly) {
        
        float areaChangeCost = 0.0f;
        if (nextPoly != null && nextPoly.area != curPoly.area) {
            areaChangeCost = areaFixedCost[nextPoly.area];
        }
        
        return dtVdist(pa, pb) * areaCost[curPoly.area] + areaChangeCost;
    }
    
    /**
     * 虚拟成本计算实现（默认调用内联成本计算）
     */
    protected float getVirtualCost(float[] pa, float[] pb,
                                  long prevRef, DetourNavMesh.MeshTile prevTile, DetourNavMesh.Poly prevPoly,
                                  long curRef, DetourNavMesh.MeshTile curTile, DetourNavMesh.Poly curPoly,
                                  long nextRef, DetourNavMesh.MeshTile nextTile, DetourNavMesh.Poly nextPoly) {
        return getInlineCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);
    }
    
    // ========== Getter和Setter方法 ==========
    
    /**
     * 获取区域的遍历成本
     * @param i 区域ID
     * @return 区域的遍历成本
     */
    public float getAreaCost(int i) {
        return areaCost[i];
    }
    
    /**
     * 设置区域的遍历成本
     * @param i 区域ID
     * @param cost 新的遍历成本
     */
    public void setAreaCost(int i, float cost) {
        areaCost[i] = cost;
        updateLowestAreaCost();
    }
    
    /**
     * 获取进入区域的固定成本
     * @param i 区域ID
     * @return 进入区域的固定成本
     */
    public float getAreaFixedCost(int i) {
        return areaFixedCost[i];
    }
    
    /**
     * 设置进入区域的固定成本
     * @param i 区域ID
     * @param cost 新的固定成本
     */
    public void setAreaFixedCost(int i, float cost) {
        areaFixedCost[i] = cost;
    }
    
    /**
     * 获取所有区域成本
     * @return 区域成本数组
     */
    public float[] getAllAreaCosts() {
        return areaCost.clone();
    }
    
    /**
     * 获取所有固定区域成本
     * @return 固定区域成本数组
     */
    public float[] getAllFixedAreaCosts() {
        return areaFixedCost.clone();
    }
    
    /**
     * 获取修改后的启发式比例
     * @return 修改后的启发式比例
     */
    public float getModifiedHeuristicScale() {
        return heuristicScale * (lowestAreaCost > 0 ? lowestAreaCost : 1.0f);
    }
    
    /**
     * 获取欧几里得距离启发式比例
     * @return 启发式比例
     */
    public float getHeuristicScale() {
        return heuristicScale;
    }
    
    /**
     * 设置欧几里得距离启发式比例
     * @param scale 新的比例
     */
    public void setHeuristicScale(float scale) {
        this.heuristicScale = scale;
    }
    
    /**
     * 验证链接边是否有效
     * @param side 边标志
     * @return 是否应接受此边的链接
     */
    public boolean isValidLinkSide(int side) {
        return (side & DT_LINK_FLAG_OFFMESH_CON) == 0 || 
               (side & DT_LINK_FLAG_OFFMESH_CON_BIDIR) != 0 ||
               (isBacktracking ? (side & DT_LINK_FLAG_OFFMESH_CON_BACKTRACKER) != 0
                               : (side & DT_LINK_FLAG_OFFMESH_CON_BACKTRACKER) == 0);
    }
    
    /**
     * 设置回溯模式
     * @param backtracking 是否为回溯模式
     */
    public void setIsBacktracking(boolean backtracking) {
        this.isBacktracking = backtracking;
    }
    
    /**
     * 获取是否为回溯模式
     * @return 是否为回溯模式
     */
    public boolean getIsBacktracking() {
        return isBacktracking;
    }
    
    /**
     * 设置是否应忽略已关闭的节点
     * @param shouldIgnore 是否应忽略
     */
    public void setShouldIgnoreClosedNodes(boolean shouldIgnore) {
        this.shouldIgnoreClosedNodes = shouldIgnore;
    }
    
    /**
     * 获取是否应忽略已关闭的节点
     * @return 是否应忽略已关闭的节点
     */
    public boolean getShouldIgnoreClosedNodes() {
        return shouldIgnoreClosedNodes;
    }
    
    /**
     * 获取包含标志
     * @return 包含标志
     */
    public int getIncludeFlags() {
        return includeFlags;
    }
    
    /**
     * 设置包含标志
     * @param flags 新的标志
     */
    public void setIncludeFlags(int flags) {
        this.includeFlags = flags;
    }
    
    /**
     * 获取排除标志
     * @return 排除标志
     */
    public int getExcludeFlags() {
        return excludeFlags;
    }
    
    /**
     * 设置排除标志
     * @param flags 新的标志
     */
    public void setExcludeFlags(int flags) {
        this.excludeFlags = flags;
    }
    
    /**
     * 检查两个过滤器是否有相同的数据值
     * @param other 另一个过滤器
     * @return 是否相等
     */
    public boolean equals(DetourQueryFilter other) {
        if (other == null) return false;
        
        return heuristicScale == other.heuristicScale &&
               lowestAreaCost == other.lowestAreaCost &&
               includeFlags == other.includeFlags &&
               excludeFlags == other.excludeFlags &&
               isBacktracking == other.isBacktracking &&
               shouldIgnoreClosedNodes == other.shouldIgnoreClosedNodes &&
               java.util.Arrays.equals(areaCost, other.areaCost) &&
               java.util.Arrays.equals(areaFixedCost, other.areaFixedCost);
    }
    
    /**
     * 从源过滤器复制数据值
     * @param other 源过滤器
     */
    public void copyFrom(DetourQueryFilter other) {
        if (other == null) return;
        
        this.heuristicScale = other.heuristicScale;
        this.lowestAreaCost = other.lowestAreaCost;
        this.includeFlags = other.includeFlags;
        this.excludeFlags = other.excludeFlags;
        this.isBacktracking = other.isBacktracking;
        this.shouldIgnoreClosedNodes = other.shouldIgnoreClosedNodes;
        
        System.arraycopy(other.areaCost, 0, this.areaCost, 0, DT_MAX_AREAS);
        System.arraycopy(other.areaFixedCost, 0, this.areaFixedCost, 0, DT_MAX_AREAS);
    }
    
    /**
     * 获取是否为虚拟过滤器
     * @return 是否为虚拟过滤器
     */
    public boolean getIsVirtual() {
        return isVirtual;
    }
    
    /**
     * 更新最低区域成本
     */
    private void updateLowestAreaCost() {
        lowestAreaCost = Float.MAX_VALUE;
        for (float cost : areaCost) {
            if (cost < lowestAreaCost) {
                lowestAreaCost = cost;
            }
        }
    }
} 