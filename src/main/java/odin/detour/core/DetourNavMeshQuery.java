package odin.detour.core;

import odin.detour.core.DetourNavMesh.*;
import static odin.detour.config.DetourConstants.*;
import static odin.detour.utils.DetourCommon.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

/**
 * Detour导航网格查询引擎
 * 翻译自UE5 DetourNavMeshQuery.h和DetourNavMeshQuery.cpp
 * 
 * 提供对导航网格执行路径查找相关查询的能力
 * 
 * @author UE5NavMesh4J
 */
public class DetourNavMeshQuery {
    
    /** 查询状态枚举 */
    public enum QueryStatus {
        SUCCESS(1),
        PARTIAL_RESULT(2),
        FAILURE(0),
        IN_PROGRESS(0x80000000);
        
        private final int value;
        
        QueryStatus(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
        
        public boolean isSuccess() {
            return (value & SUCCESS.value) != 0;
        }
        
        public boolean isInProgress() {
            return (value & IN_PROGRESS.value) != 0;
        }
        
        public boolean isPartial() {
            return (value & PARTIAL_RESULT.value) != 0;
        }
    }
    
    /** 查询结果数据包 */
    public static class QueryResult {
        public long ref;
        public float cost;
        public float[] pos = new float[3];
        public int flag;
        
        public QueryResult() {
            this.ref = 0;
            this.cost = 0.0f;
            this.flag = 0;
        }
        
        public QueryResult(long ref, float cost, float[] pos, int flag) {
            this.ref = ref;
            this.cost = cost;
            if (pos != null && pos.length >= 3) {
                System.arraycopy(pos, 0, this.pos, 0, 3);
            }
            this.flag = flag;
        }
    }
    
    /** 导航网格节点 */
    private static class Node {
        public float[] pos = new float[3];  // 节点位置
        public float cost;                  // 从起点到该节点的成本
        public float total;                 // cost + heuristic
        public long parentIndex;            // 父节点索引
        public int state;                   // 节点状态
        public int flags;                   // 节点标志
        public long id;                     // 多边形引用id
        
        public Node() {
            this.cost = 0.0f;
            this.total = 0.0f;
            this.parentIndex = 0;
            this.state = 0;
            this.flags = 0;
            this.id = 0;
        }
    }
    
    /** 节点状态常量 */
    private static final int DT_NODE_OPEN = 0x01;
    private static final int DT_NODE_CLOSED = 0x02;
    private static final int DT_NODE_PARENT_DETACHED = 0x04;
    
    /** 最大节点数量 */
    private static final int DT_NODE_POOL_SIZE = 65536;
    
    /** 最大开放节点数量 */
    private static final int DT_MAX_OPEN_NODES = 4096;
    
    /** 直线路径标志 */
    private static final int DT_STRAIGHTPATH_START = 0x01;
    private static final int DT_STRAIGHTPATH_END = 0x02;
    private static final int DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04;
    
    // 查询状态
    private DetourNavMesh navMesh;
    private Node[] nodePool;
    private Node[] openList;
    private int openListSize;
    private int nextFreeNode;
    private QueryStatus queryStatus;
    
    // 分片查询状态
    private long startRef, endRef;
    private float[] startPos = new float[3];
    private float[] endPos = new float[3];
    private DetourQueryFilter filter;
    private boolean requireNavigableEndLocation;
    private float costLimit;
    
    /**
     * 初始化查询引擎
     * @param nav 导航网格
     * @param maxNodes 最大节点数量
     * @return true如果初始化成功
     */
    public boolean init(DetourNavMesh nav, int maxNodes) {
        this.navMesh = nav;
        
        // 初始化节点池
        int nodePoolSize = Math.min(maxNodes, DT_NODE_POOL_SIZE);
        this.nodePool = new Node[nodePoolSize];
        for (int i = 0; i < nodePoolSize; i++) {
            this.nodePool[i] = new Node();
        }
        
        // 初始化开放列表
        this.openList = new Node[DT_MAX_OPEN_NODES];
        this.openListSize = 0;
        this.nextFreeNode = 0;
        
        this.queryStatus = QueryStatus.SUCCESS;
        
        return true;
    }
    
    /**
     * 查找最近的多边形
     * @param center 中心点
     * @param extents 搜索范围
     * @param filter 查询过滤器
     * @param nearestRef 输出：最近的多边形引用
     * @param nearestPt 输出：最近点
     * @return 查询状态
     */
    public QueryStatus findNearestPoly(float[] center, float[] extents, DetourQueryFilter filter,
                                      long[] nearestRef, float[] nearestPt) {
        nearestRef[0] = 0;
        dtVcopy(nearestPt, center);
        
        if (navMesh == null) {
            return QueryStatus.FAILURE;
        }
        
        // 计算搜索边界框
        float[] bmin = new float[3];
        float[] bmax = new float[3];
        dtVsub(bmin, center, extents);
        dtVadd(bmax, center, extents);
        
        // 查找附近的瓦片
        List<MeshTile> tiles = new ArrayList<>();
        int ntiles = queryTiles(bmin, bmax, tiles, 32);
        
        float nearestDistanceSqr = Float.MAX_VALUE;
        
        for (int i = 0; i < ntiles; i++) {
            QueryStatus status = findNearestPolyInTile(tiles.get(i), center, extents, filter,
                                                      nearestRef, nearestPt);
            if (status.isSuccess()) {
                // 检查是否更近
                float distSqr = dtVdistSqr(center, nearestPt);
                if (distSqr < nearestDistanceSqr) {
                    nearestDistanceSqr = distSqr;
                }
            }
        }
        
        return nearestRef[0] != 0 ? QueryStatus.SUCCESS : QueryStatus.FAILURE;
    }
    
    /**
     * 在指定瓦片中查找最近多边形
     * @param tile 瓦片
     * @param center 中心点
     * @param extents 搜索范围
     * @param filter 过滤器
     * @param nearestRef 输出：最近多边形引用
     * @param nearestPt 输出：最近点
     * @return 查询状态
     */
    private QueryStatus findNearestPolyInTile(MeshTile tile, float[] center, float[] extents,
                                             DetourQueryFilter filter, long[] nearestRef, float[] nearestPt) {
        float[] bmin = new float[3];
        float[] bmax = new float[3];
        dtVsub(bmin, center, extents);
        dtVadd(bmax, center, extents);
        
        // 遍历瓦片中的多边形
        for (int i = 0; i < tile.header.polyCount; i++) {
            long ref = getPolyRefBase(tile) | (long) i;
            
            if (!filter.passFilter(ref, tile, tile.polys[i])) {
                continue;
            }
            
            // 获取多边形边界框
            float[] polyBmin = new float[3];
            float[] polyBmax = new float[3];
            
            // 简化实现：使用多边形的第一个顶点作为中心
            if (tile.polys[i].vertCount > 0) {
                int iv = tile.polys[i].verts[0] * 3;
                polyBmin[0] = polyBmax[0] = (float) tile.verts[iv];
                polyBmin[1] = polyBmax[1] = (float) tile.verts[iv + 1];
                polyBmin[2] = polyBmax[2] = (float) tile.verts[iv + 2];
                
                // 计算所有顶点的边界框
                for (int j = 1; j < tile.polys[i].vertCount; j++) {
                    int jv = tile.polys[i].verts[j] * 3;
                    polyBmin[0] = Math.min(polyBmin[0], (float) tile.verts[jv]);
                    polyBmin[1] = Math.min(polyBmin[1], (float) tile.verts[jv + 1]);
                    polyBmin[2] = Math.min(polyBmin[2], (float) tile.verts[jv + 2]);
                    polyBmax[0] = Math.max(polyBmax[0], (float) tile.verts[jv]);
                    polyBmax[1] = Math.max(polyBmax[1], (float) tile.verts[jv + 1]);
                    polyBmax[2] = Math.max(polyBmax[2], (float) tile.verts[jv + 2]);
                }
                
                // 检查边界框重叠
                if (dtOverlapBounds(bmin, bmax, polyBmin, polyBmax)) {
                    nearestRef[0] = ref;
                    dtVcopy(nearestPt, center); // 简化：返回中心点
                    return QueryStatus.SUCCESS;
                }
            }
        }
        
        return QueryStatus.FAILURE;
    }
    
    /**
     * 查询指定范围内的瓦片
     * @param bmin 最小边界
     * @param bmax 最大边界
     * @param tiles 输出瓦片列表
     * @param maxTiles 最大瓦片数量
     * @return 找到的瓦片数量
     */
    private int queryTiles(float[] bmin, float[] bmax, List<MeshTile> tiles, int maxTiles) {
        // 简化实现：返回所有瓦片（实际实现需要空间划分）
        // TODO: 实现基于空间索引的瓦片查询
        return 0;
    }
    
    /**
     * 获取多边形引用基址
     * @param tile 瓦片
     * @return 引用基址
     */
    private long getPolyRefBase(MeshTile tile) {
        // 简化实现：使用瓦片的盐值作为基址
        return tile.salt << 20;
    }
    
    /**
     * 查找路径
     * @param startRef 起始多边形引用
     * @param endRef 目标多边形引用
     * @param startPos 起始位置
     * @param endPos 目标位置
     * @param costLimit 成本限制
     * @param filter 查询过滤器
     * @param path 输出路径
     * @param totalCost 输出总成本
     * @return 查询状态
     */
    public QueryStatus findPath(long startRef, long endRef, float[] startPos, float[] endPos, 
                               float costLimit, DetourQueryFilter filter, 
                               List<Long> path, float[] totalCost) {
        
        if (path != null) {
            path.clear();
        }
        if (totalCost != null && totalCost.length > 0) {
            totalCost[0] = 0.0f;
        }
        
        // 验证输入
        if (navMesh == null || startRef == 0 || endRef == 0 || 
            startPos == null || endPos == null || filter == null) {
            return QueryStatus.FAILURE;
        }
        
        // 如果起点和终点是同一个多边形
        if (startRef == endRef) {
            if (path != null) {
                path.add(startRef);
            }
            return QueryStatus.SUCCESS;
        }
        
        // 初始化分片查询
        QueryStatus status = initSlicedFindPath(startRef, endRef, startPos, endPos, costLimit, true, filter);
        if (!status.isSuccess()) {
            return status;
        }
        
        // 执行搜索
        status = updateSlicedFindPath(1000, null);
        if (status.isInProgress()) {
            return QueryStatus.PARTIAL_RESULT;
        }
        
        if (status.isSuccess()) {
            // 完成路径查找
            long[] pathArray = new long[256];
            int[] pathCount = new int[1];
            status = finalizeSlicedFindPath(pathArray, pathCount, 256);
            
            if (status.isSuccess() && path != null) {
                for (int i = 0; i < pathCount[0]; i++) {
                    path.add(pathArray[i]);
                }
            }
        }
        
        return status;
    }
    
    /**
     * 初始化分片路径查找
     * @param startRef 起始多边形引用
     * @param endRef 目标多边形引用
     * @param startPos 起始位置
     * @param endPos 目标位置
     * @param costLimit 成本限制
     * @param requireNavigableEndLocation 是否要求可导航的终点位置
     * @param filter 查询过滤器
     * @return 查询状态
     */
    public QueryStatus initSlicedFindPath(long startRef, long endRef, float[] startPos, float[] endPos,
                                         float costLimit, boolean requireNavigableEndLocation,
                                         DetourQueryFilter filter) {
        
        // 重置查询状态
        resetQuery();
        
        // 保存查询参数
        this.startRef = startRef;
        this.endRef = endRef;
        dtVcopy(this.startPos, startPos);
        dtVcopy(this.endPos, endPos);
        this.filter = filter;
        this.requireNavigableEndLocation = requireNavigableEndLocation;
        this.costLimit = costLimit;
        
        // 验证输入
        if (navMesh == null || startRef == 0 || endRef == 0 || filter == null) {
            return QueryStatus.FAILURE;
        }
        
        // 初始化起始节点
        Node startNode = getNode(startRef);
        if (startNode == null) {
            return QueryStatus.FAILURE;
        }
        
        dtVcopy(startNode.pos, startPos);
        startNode.parentIndex = 0;
        startNode.cost = 0.0f;
        startNode.total = dtVdist(startPos, endPos) * filter.getHeuristicScale();
        startNode.id = startRef;
        startNode.flags = DT_NODE_OPEN;
        
        // 添加到开放列表
        pushOpen(startNode);
        
        this.queryStatus = QueryStatus.IN_PROGRESS;
        
        return QueryStatus.IN_PROGRESS;
    }
    
    /**
     * 更新分片路径查找
     * @param maxIter 最大迭代次数
     * @param doneIters 输出：完成的迭代次数
     * @return 查询状态
     */
    public QueryStatus updateSlicedFindPath(int maxIter, int[] doneIters) {
        if (doneIters != null && doneIters.length > 0) {
            doneIters[0] = 0;
        }
        
        if (!queryStatus.isInProgress()) {
            return queryStatus;
        }
        
        int iter = 0;
        while (iter < maxIter && !isEmpty()) {
            iter++;
            
            // 获取代价最小的节点
            Node bestNode = popOpen();
            bestNode.flags &= ~DT_NODE_OPEN;
            bestNode.flags |= DT_NODE_CLOSED;
            
            // 检查是否到达目标
            if (bestNode.id == endRef) {
                queryStatus = QueryStatus.SUCCESS;
                break;
            }
            
            // 展开邻居节点
            expandNeighbors(bestNode);
            
            // 检查成本限制
            if (bestNode.total >= costLimit) {
                queryStatus = QueryStatus.PARTIAL_RESULT;
                break;
            }
        }
        
        if (doneIters != null && doneIters.length > 0) {
            doneIters[0] = iter;
        }
        
        // 如果开放列表为空且没有找到路径
        if (isEmpty() && !queryStatus.isSuccess()) {
            queryStatus = QueryStatus.FAILURE;
        }
        
        return queryStatus;
    }
    
    /**
     * 展开节点的邻居
     * @param bestNode 当前最优节点
     */
    private void expandNeighbors(Node bestNode) {
        MeshTile tile = null;
        Poly poly = null;
        
        // 获取多边形和瓦片
        // 简化实现：假设我们有方法获取瓦片和多边形
        // TODO: 实现从多边形引用获取瓦片和多边形的方法
        
        if (tile == null || poly == null) {
            return;
        }
        
        // 遍历多边形的所有边
        for (int i = 0; i < poly.vertCount; i++) {
            // 获取邻居多边形引用
            long neighbourRef = 0;
            if (poly.neis[i] != 0) {
                // 内部邻居
                neighbourRef = getPolyRefBase(tile) | (long) poly.neis[i];
            } else {
                // 外部邻居（通过链接）
                neighbourRef = getNeighbourThroughLink(tile, poly, i);
            }
            
            if (neighbourRef == 0) {
                continue;
            }
            
            // 检查过滤器
            MeshTile neighbourTile = null; // TODO: 获取邻居瓦片
            Poly neighbourPoly = null;     // TODO: 获取邻居多边形
            
            if (neighbourTile == null || neighbourPoly == null || 
                !filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                continue;
            }
            
            // 计算邻居节点位置和成本
            float[] neighbourPos = new float[3];
            float cost = calculateMoveCost(bestNode.pos, neighbourPos, bestNode.id, neighbourRef,
                                          tile, poly, neighbourTile, neighbourPoly);
            cost += bestNode.cost;
            
            // 检查是否需要更新或创建节点
            Node neighbourNode = getNode(neighbourRef);
            if (neighbourNode == null) {
                // 创建新节点
                neighbourNode = getNode(neighbourRef);
                if (neighbourNode != null) {
                    dtVcopy(neighbourNode.pos, neighbourPos);
                    neighbourNode.cost = cost;
                    neighbourNode.total = cost + calculateHeuristic(neighbourPos, endPos);
                    neighbourNode.id = neighbourRef;
                    neighbourNode.flags = DT_NODE_OPEN;
                    neighbourNode.parentIndex = getNodeIndex(bestNode);
                    
                    pushOpen(neighbourNode);
                }
            } else if (cost < neighbourNode.cost) {
                // 更新现有节点
                neighbourNode.cost = cost;
                neighbourNode.total = cost + calculateHeuristic(neighbourPos, endPos);
                neighbourNode.parentIndex = getNodeIndex(bestNode);
                
                if ((neighbourNode.flags & DT_NODE_OPEN) != 0) {
                    // 节点在开放列表中，需要重新排序
                    pushOpen(neighbourNode);
                }
            }
        }
    }
    
    /**
     * 通过链接获取邻居
     * @param tile 瓦片
     * @param poly 多边形
     * @param edge 边索引
     * @return 邻居多边形引用
     */
    private long getNeighbourThroughLink(MeshTile tile, Poly poly, int edge) {
        // 遍历多边形的链接
        for (long linkIndex = poly.firstLink; linkIndex != DT_NULL_LINK; ) {
            if (linkIndex >= tile.links.length) {
                break;
            }
            
            Link link = tile.links[(int) linkIndex];
            if (link.edge == edge) {
                return link.ref.value;
            }
            
            linkIndex = link.next;
        }
        
        return 0;
    }
    
    /**
     * 计算移动成本
     * @param from 起始位置
     * @param to 目标位置
     * @param fromRef 起始多边形引用
     * @param toRef 目标多边形引用
     * @param fromTile 起始瓦片
     * @param fromPoly 起始多边形
     * @param toTile 目标瓦片
     * @param toPoly 目标多边形
     * @return 移动成本
     */
    private float calculateMoveCost(float[] from, float[] to, 
                                   long fromRef, long toRef,
                                   MeshTile fromTile, Poly fromPoly,
                                   MeshTile toTile, Poly toPoly) {
        float cost = dtVdist(from, to);
        
        // 应用过滤器的成本修正
        if (filter != null) {
            cost *= filter.getCost(from, to, 
                                  0, null, null,           // prevRef, prevTile, prevPoly (无前一个)
                                  fromRef, fromTile, fromPoly,  // 当前
                                  toRef, toTile, toPoly);       // 下一个
        }
        
        return cost;
    }
    
    /**
     * 计算启发式成本
     * @param from 起始位置
     * @param to 目标位置
     * @return 启发式成本
     */
    private float calculateHeuristic(float[] from, float[] to) {
        return dtVdist(from, to) * (filter != null ? filter.getHeuristicScale() : 1.0f);
    }
    
    /**
     * 获取节点在池中的索引
     * @param node 节点
     * @return 节点索引
     */
    private long getNodeIndex(Node node) {
        for (int i = 0; i < nextFreeNode; i++) {
            if (nodePool[i] == node) {
                return i + 1; // 索引从1开始，0表示无效
            }
        }
        return 0;
    }
    
    /**
     * 完成分片路径查找
     * @param path 输出路径数组
     * @param pathCount 输出路径长度
     * @param maxPath 最大路径长度
     * @return 查询状态
     */
    public QueryStatus finalizeSlicedFindPath(long[] path, int[] pathCount, int maxPath) {
        pathCount[0] = 0;
        
        if (!queryStatus.isSuccess()) {
            return queryStatus;
        }
        
        // 查找终点节点
        Node endNode = getNode(endRef);
        if (endNode == null) {
            return QueryStatus.FAILURE;
        }
        
        // 回溯路径
        List<Long> pathList = new ArrayList<>();
        Node node = endNode;
        
        do {
            pathList.add(node.id);
            node = getNodeAtIndex(node.parentIndex);
        } while (node != null);
        
        // 反转路径（从起点到终点）
        int pathLen = Math.min(pathList.size(), maxPath);
        for (int i = 0; i < pathLen; i++) {
            path[i] = pathList.get(pathLen - 1 - i);
        }
        
        pathCount[0] = pathLen;
        
        return pathLen > 0 ? QueryStatus.SUCCESS : QueryStatus.FAILURE;
    }
    
    /**
     * 查找直线路径
     * @param startPos 起始位置
     * @param endPos 终点位置
     * @param path 多边形路径
     * @param pathSize 路径长度
     * @param straightPath 输出直线路径点
     * @param straightPathFlags 输出路径标志
     * @param straightPathRefs 输出多边形引用
     * @param straightPathCount 输出直线路径点数量
     * @param maxStraightPath 最大直线路径点数量
     * @param options 选项标志
     * @return 查询状态
     */
    public QueryStatus findStraightPath(float[] startPos, float[] endPos, long[] path, int pathSize,
                                       float[] straightPath, int[] straightPathFlags, long[] straightPathRefs,
                                       int[] straightPathCount, int maxStraightPath, int options) {
        
        straightPathCount[0] = 0;
        
        if (navMesh == null || straightPath == null || path == null || pathSize <= 0) {
            return QueryStatus.FAILURE;
        }
        
        // 添加起始点
        int straightPathSize = 0;
        if (straightPathSize < maxStraightPath) {
            dtVcopy(Arrays.copyOfRange(straightPath, straightPathSize * 3, (straightPathSize + 1) * 3), startPos);
            if (straightPathFlags != null) {
                straightPathFlags[straightPathSize] = DT_STRAIGHTPATH_START;
            }
            if (straightPathRefs != null) {
                straightPathRefs[straightPathSize] = path[0];
            }
            straightPathSize++;
        }
        
        // 处理路径中的每一段
        float[] portalApex = new float[3];
        float[] portalLeft = new float[3];
        float[] portalRight = new float[3];
        
        dtVcopy(portalApex, startPos);
        dtVcopy(portalLeft, startPos);
        dtVcopy(portalRight, startPos);
        
        int apexIndex = 0;
        int leftIndex = 0;
        int rightIndex = 0;
        
        for (int i = 0; i < pathSize && straightPathSize < maxStraightPath; i++) {
            // 获取门户边
            float[] left = new float[3];
            float[] right = new float[3];
            
            if (i + 1 < pathSize) {
                // 获取两个相邻多边形之间的门户
                if (!getPortalPoints(path[i], path[i + 1], left, right)) {
                    continue;
                }
            } else {
                // 最后一段，直接到终点
                dtVcopy(left, endPos);
                dtVcopy(right, endPos);
            }
            
            // 更新漏斗
            if (updateFunnel(portalApex, portalLeft, portalRight, left, right,
                            apexIndex, leftIndex, rightIndex, i, 
                            straightPath, straightPathFlags, straightPathRefs,
                            straightPathSize, maxStraightPath, path)) {
                straightPathSize++;
            }
        }
        
        // 添加终点
        if (straightPathSize < maxStraightPath) {
            dtVcopy(Arrays.copyOfRange(straightPath, straightPathSize * 3, (straightPathSize + 1) * 3), endPos);
            if (straightPathFlags != null) {
                straightPathFlags[straightPathSize] = DT_STRAIGHTPATH_END;
            }
            if (straightPathRefs != null) {
                straightPathRefs[straightPathSize] = path[pathSize - 1];
            }
            straightPathSize++;
        }
        
        straightPathCount[0] = straightPathSize;
        
        return QueryStatus.SUCCESS;
    }
    
    /**
     * 获取两个多边形之间的门户点
     * @param from 起始多边形引用
     * @param to 目标多边形引用
     * @param left 输出左门户点
     * @param right 输出右门户点
     * @return true如果找到门户
     */
    private boolean getPortalPoints(long from, long to, float[] left, float[] right) {
        // 获取多边形和瓦片
        MeshTile fromTile = null;
        Poly fromPoly = null;
        MeshTile toTile = null;
        Poly toPoly = null;
        
        // 从引用中获取瓦片和多边形
        if (!getPolyAndTile(from, fromTile, fromPoly) || 
            !getPolyAndTile(to, toTile, toPoly)) {
            return false;
        }
        
        // 查找共享边
        int fromEdge = -1;
        int toEdge = -1;
        
        // 遍历from多边形的所有边
        for (int i = 0; i < fromPoly.vertCount; i++) {
            int j = (i + 1) % fromPoly.vertCount;
            
            // 检查是否是共享边
            if (fromPoly.neis[i] != 0) {
                // 内部邻居
                long neighbourRef = getPolyRefBase(fromTile) | (long) fromPoly.neis[i];
                if (neighbourRef == to) {
                    fromEdge = i;
                    toEdge = -1;
                    break;
                }
            } else {
                // 外部邻居（通过链接）
                for (long linkIndex = fromPoly.firstLink; linkIndex != DT_NULL_LINK; ) {
                    if (linkIndex >= fromTile.links.length) {
                        break;
                    }
                    
                    Link link = fromTile.links[(int) linkIndex];
                    if (link.edge == i && link.ref.value == to) {
                        fromEdge = i;
                        toEdge = link.edge;
                        break;
                    }
                    
                    linkIndex = link.next;
                }
                
                if (fromEdge != -1) {
                    break;
                }
            }
        }
        
        if (fromEdge == -1) {
            return false;
        }
        
        // 获取门户顶点
        int fromVert = fromPoly.verts[fromEdge] * 3;
        int fromNextVert = fromPoly.verts[(fromEdge + 1) % fromPoly.vertCount] * 3;
        
        // 创建临时数组用于存储顶点数据
        float[] fromVertPos = new float[3];
        float[] fromNextVertPos = new float[3];
        
        // 转换顶点数据为float类型
        for (int i = 0; i < 3; i++) {
            fromVertPos[i] = (float) fromTile.verts[fromVert + i];
            fromNextVertPos[i] = (float) fromTile.verts[fromNextVert + i];
        }
        
        if (toEdge == -1) {
            // 内部邻居
            dtVcopy(left, fromVertPos);
            dtVcopy(right, fromNextVertPos);
        } else {
            // 外部邻居
            int toVert = toPoly.verts[toEdge] * 3;
            int toNextVert = toPoly.verts[(toEdge + 1) % toPoly.vertCount] * 3;
            
            // 创建临时数组用于存储目标顶点数据
            float[] toVertPos = new float[3];
            float[] toNextVertPos = new float[3];
            
            // 转换目标顶点数据为float类型
            for (int i = 0; i < 3; i++) {
                toVertPos[i] = (float) toTile.verts[toVert + i];
                toNextVertPos[i] = (float) toTile.verts[toNextVert + i];
            }
            
            // 确保顶点顺序一致
            if (dtVdistSqr(fromVertPos, toVertPos) < dtVdistSqr(fromVertPos, toNextVertPos)) {
                dtVcopy(left, fromVertPos);
                dtVcopy(right, fromNextVertPos);
            } else {
                dtVcopy(left, fromNextVertPos);
                dtVcopy(right, fromVertPos);
            }
        }
        
        return true;
    }
    
    /**
     * 从引用中获取多边形和瓦片
     * @param ref 多边形引用
     * @param tile 输出瓦片
     * @param poly 输出多边形
     * @return true如果成功获取
     */
    private boolean getPolyAndTile(long ref, MeshTile tile, Poly poly) {
        if (navMesh == null || ref == 0) {
            return false;
        }
        
        // 从引用中提取盐值、瓦片索引和多边形索引
        int salt = (int) ((ref >> 20) & 0x3FF);
        int tileIndex = (int) ((ref >> 12) & 0xFF);
        int polyIndex = (int) (ref & 0x3FF);
        
        // 获取瓦片
        tile = navMesh.getTile(tileIndex);
        if (tile == null || tile.salt != salt) {
            return false;
        }
        
        // 获取多边形
        if (polyIndex >= tile.header.polyCount) {
            return false;
        }
        poly = tile.polys[polyIndex];
        
        return true;
    }
    
    /**
     * 更新漏斗算法
     * @param apex 漏斗顶点
     * @param left 左边界
     * @param right 右边界
     * @param newLeft 新左点
     * @param newRight 新右点
     * @param apexIndex 顶点索引
     * @param leftIndex 左索引
     * @param rightIndex 右索引
     * @param currentIndex 当前索引
     * @param straightPath 直线路径
     * @param straightPathFlags 路径标志
     * @param straightPathRefs 路径引用
     * @param straightPathSize 当前路径大小
     * @param maxStraightPath 最大路径大小
     * @param path 原始路径
     * @return true如果添加了新点
     */
    private boolean updateFunnel(float[] apex, float[] left, float[] right,
                                float[] newLeft, float[] newRight,
                                int apexIndex, int leftIndex, int rightIndex, int currentIndex,
                                float[] straightPath, int[] straightPathFlags, long[] straightPathRefs,
                                int straightPathSize, int maxStraightPath, long[] path) {
        // 检查新左点是否在左边界内
        if (dtTriArea2D(apex, left, newLeft) <= 0.0f) {
            if (dtVequal(apex, left) || dtTriArea2D(apex, right, newLeft) > 0.0f) {
                // 更新左边界
                dtVcopy(left, newLeft);
                leftIndex = currentIndex;
            } else {
                // 添加顶点
                dtVcopy(apex, right);
                apexIndex = rightIndex;
                
                // 添加路径点
                if (straightPathSize < maxStraightPath) {
                    float[] pos = Arrays.copyOfRange(straightPath, straightPathSize * 3, (straightPathSize + 1) * 3);
                    dtVcopy(pos, apex);
                    if (straightPathFlags != null) {
                        straightPathFlags[straightPathSize] = 0;
                    }
                    if (straightPathRefs != null) {
                        straightPathRefs[straightPathSize] = path[apexIndex];
                    }
                    straightPathSize++;
                    return true;
                }
            }
        }
        
        // 检查新右点是否在右边界内
        if (dtTriArea2D(apex, right, newRight) >= 0.0f) {
            if (dtVequal(apex, right) || dtTriArea2D(apex, left, newRight) < 0.0f) {
                // 更新右边界
                dtVcopy(right, newRight);
                rightIndex = currentIndex;
            } else {
                // 添加顶点
                dtVcopy(apex, left);
                apexIndex = leftIndex;
                
                // 添加路径点
                if (straightPathSize < maxStraightPath) {
                    float[] pos = Arrays.copyOfRange(straightPath, straightPathSize * 3, (straightPathSize + 1) * 3);
                    dtVcopy(pos, apex);
                    if (straightPathFlags != null) {
                        straightPathFlags[straightPathSize] = 0;
                    }
                    if (straightPathRefs != null) {
                        straightPathRefs[straightPathSize] = path[apexIndex];
                    }
                    straightPathSize++;
                    return true;
                }
            }
        }
        
        return false;
    }
    
    /**
     * 计算2D三角形面积
     * @param a 点A
     * @param b 点B
     * @param c 点C
     * @return 三角形面积
     */
    private static float dtTriArea2D(float[] a, float[] b, float[] c) {
        float ax = b[0] - a[0];
        float ay = b[2] - a[2];
        float bx = c[0] - a[0];
        float by = c[2] - a[2];
        return bx * ay - ax * by;
    }
    
    /**
     * 检查两个向量是否相等
     * @param a 向量A
     * @param b 向量B
     * @return true如果相等
     */
    private static boolean dtVequal(float[] a, float[] b) {
        return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
    }
    
    // ========== 私有辅助方法 ==========
    
    /**
     * 重置查询状态
     */
    private void resetQuery() {
        openListSize = 0;
        nextFreeNode = 0;
        
        // 清空节点池
        for (int i = 0; i < nodePool.length; i++) {
            nodePool[i].parentIndex = 0;
            nodePool[i].cost = 0.0f;
            nodePool[i].total = 0.0f;
            nodePool[i].state = 0;
            nodePool[i].flags = 0;
            nodePool[i].id = 0;
        }
        
        queryStatus = QueryStatus.SUCCESS;
    }
    
    /**
     * 获取节点
     * @param id 多边形引用ID
     * @return 节点实例
     */
    private Node getNode(long id) {
        // 首先检查是否已存在
        for (int i = 0; i < nextFreeNode; i++) {
            if (nodePool[i].id == id) {
                return nodePool[i];
            }
        }
        
        // 创建新节点
        if (nextFreeNode >= nodePool.length) {
            return null;
        }
        
        Node node = nodePool[nextFreeNode++];
        node.id = id;
        return node;
    }
    
    /**
     * 根据索引获取节点
     * @param index 节点索引
     * @return 节点实例
     */
    private Node getNodeAtIndex(long index) {
        if (index == 0 || index > nextFreeNode) {
            return null;
        }
        return nodePool[(int)(index - 1)];
    }
    
    /**
     * 将节点添加到开放列表
     * @param node 节点
     */
    private void pushOpen(Node node) {
        if (openListSize >= openList.length) {
            return;
        }
        
        openList[openListSize++] = node;
        
        // 向上调整堆
        int hole = openListSize - 1;
        while (hole > 0) {
            int parent = (hole - 1) / 2;
            if (openList[parent].total <= openList[hole].total) {
                break;
            }
            
            Node temp = openList[hole];
            openList[hole] = openList[parent];
            openList[parent] = temp;
            hole = parent;
        }
    }
    
    /**
     * 从开放列表弹出最优节点
     * @return 最优节点
     */
    private Node popOpen() {
        if (openListSize == 0) {
            return null;
        }
        
        Node result = openList[0];
        openListSize--;
        
        if (openListSize > 0) {
            openList[0] = openList[openListSize];
            
            // 向下调整堆
            int hole = 0;
            while (true) {
                int left = hole * 2 + 1;
                int right = hole * 2 + 2;
                int smallest = hole;
                
                if (left < openListSize && openList[left].total < openList[smallest].total) {
                    smallest = left;
                }
                if (right < openListSize && openList[right].total < openList[smallest].total) {
                    smallest = right;
                }
                
                if (smallest == hole) {
                    break;
                }
                
                Node temp = openList[hole];
                openList[hole] = openList[smallest];
                openList[smallest] = temp;
                hole = smallest;
            }
        }
        
        return result;
    }
    
    /**
     * 检查开放列表是否为空
     * @return true如果为空
     */
    private boolean isEmpty() {
        return openListSize == 0;
    }
    
    // ========== 工具方法 ==========
    
    /**
     * 复制向量
     * @param dest 目标向量
     * @param src 源向量
     */
    private static void dtVcopy(float[] dest, float[] src) {
        if (dest != null && src != null && dest.length >= 3 && src.length >= 3) {
            dest[0] = src[0];
            dest[1] = src[1];
            dest[2] = src[2];
        }
    }
    
    /**
     * 向量减法 (float版本)
     * @param dest 结果向量
     * @param v1 向量1
     * @param v2 向量2
     */
    private static void dtVsub(float[] dest, float[] v1, float[] v2) {
        dest[0] = v1[0] - v2[0];
        dest[1] = v1[1] - v2[1];
        dest[2] = v1[2] - v2[2];
    }
    
    /**
     * 向量加法 (float版本)
     * @param dest 结果向量
     * @param v1 向量1
     * @param v2 向量2
     */
    private static void dtVadd(float[] dest, float[] v1, float[] v2) {
        dest[0] = v1[0] + v2[0];
        dest[1] = v1[1] + v2[1];
        dest[2] = v1[2] + v2[2];
    }
    
    /**
     * 设置向量值 (float版本)
     * @param dest 目标向量
     * @param x X值
     * @param y Y值
     * @param z Z值
     */
    private static void dtVset(float[] dest, float x, float y, float z) {
        dest[0] = x;
        dest[1] = y;
        dest[2] = z;
    }
    
    /**
     * 计算向量距离 (float版本)
     * @param v1 向量1
     * @param v2 向量2
     * @return 距离
     */
    private static float dtVdist(float[] v1, float[] v2) {
        float dx = v2[0] - v1[0];
        float dy = v2[1] - v1[1];
        float dz = v2[2] - v1[2];
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    /**
     * 计算向量距离平方 (float版本)
     * @param v1 向量1
     * @param v2 向量2
     * @return 距离平方
     */
    private static float dtVdistSqr(float[] v1, float[] v2) {
        float dx = v2[0] - v1[0];
        float dy = v2[1] - v1[1];
        float dz = v2[2] - v1[2];
        return dx * dx + dy * dy + dz * dz;
    }
    
    /**
     * 边界框重叠检测 (float版本)
     * @param amin A盒子最小值
     * @param amax A盒子最大值
     * @param bmin B盒子最小值
     * @param bmax B盒子最大值
     * @return true如果重叠
     */
    private static boolean dtOverlapBounds(float[] amin, float[] amax, float[] bmin, float[] bmax) {
        boolean overlap = true;
        overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
        overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
        overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
        return overlap;
    }
} 