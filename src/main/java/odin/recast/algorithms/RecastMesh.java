package odin.recast.algorithms;

import odin.recast.core.RecastContour.Contour;
import odin.recast.core.RecastContour.ContourSet;
import odin.recast.core.RecastPolyMesh.PolyMesh;
import odin.recast.config.RecastEnums.TimerLabel;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

import static odin.recast.utils.RecastMath.*;
import static odin.recast.config.RecastConstants.*;

/**
 * Recast多边形网格生成算法
 * 翻译自UE5 RecastMesh.cpp
 * 
 * 负责从轮廓生成最终的多边形导航网格
 * 
 * @author UE5NavMesh4J
 */
public final class RecastMesh {
    
    /** 私有构造函数防止实例化 */
    private RecastMesh() {
        throw new UnsupportedOperationException("工具类不能被实例化");
    }
    
    /** 顶点哈希桶数量 */
    private static final int VERTEX_BUCKET_COUNT = (1 << 12);
    
    /**
     * 边结构
     */
    private static class Edge {
        int[] vert = new int[2];
        int[] polyEdge = new int[2];
        int[] poly = new int[2];
    }
    
    /**
     * 计算顶点哈希值
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @return 哈希值
     */
    private static int computeVertexHash(int x, int y, int z) {
        final long h1 = 0x8da6b343L; // 大的乘法常数
        final long h2 = 0xd8163841L; // 这里任意选择的质数
        final long h3 = 0xcb1ab31fL;
        long n = h1 * x + h2 * y + h3 * z;
        return (int)(n & (VERTEX_BUCKET_COUNT - 1));
    }
    
    /**
     * 添加顶点（去重）
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param verts 顶点数组
     * @param firstVert 第一个顶点索引数组
     * @param nextVert 下一个顶点索引数组
     * @param nv 当前顶点数量
     * @return 顶点索引
     */
    private static int addVertex(int x, int y, int z, int[] verts, int[] firstVert, int[] nextVert, IntRef nv) {
        int bucket = computeVertexHash(x, 0, z);
        int i = firstVert[bucket];
        
        while (i != -1) {
            int[] v = {verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2]};
            if (v[0] == x && Math.abs(v[1] - y) <= 2 && v[2] == z) {
                return i;
            }
            i = nextVert[i]; // 下一个
        }
        
        // 未找到，创建新的
        i = nv.value;
        nv.value++;
        verts[i * 3] = x;
        verts[i * 3 + 1] = y;
        verts[i * 3 + 2] = z;
        nextVert[i] = firstVert[bucket];
        firstVert[bucket] = i;
        
        return i;
    }
    
    /**
     * 获取前一个索引
     * @param i 当前索引
     * @param n 数组长度
     * @return 前一个索引
     */
    private static int prev(int i, int n) {
        return i - 1 >= 0 ? i - 1 : n - 1;
    }
    
    /**
     * 获取下一个索引
     * @param i 当前索引
     * @param n 数组长度
     * @return 下一个索引
     */
    private static int next(int i, int n) {
        return i + 1 < n ? i + 1 : 0;
    }
    
    /**
     * 计算三角形有符号面积的两倍
     * @param a 顶点A
     * @param b 顶点B
     * @param c 顶点C
     * @return 有符号面积的两倍
     */
    private static int area2(int[] a, int[] b, int[] c) {
        return (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
    }
    
    /**
     * 异或运算
     * @param x 布尔值X
     * @param y 布尔值Y
     * @return x异或y
     */
    private static boolean xorb(boolean x, boolean y) {
        return !x ^ !y;
    }
    
    /**
     * 检查点c是否严格在有向线ab的左侧
     * @param a 点A
     * @param b 点B
     * @param c 点C
     * @return true如果c在ab左侧
     */
    private static boolean left(int[] a, int[] b, int[] c) {
        return area2(a, b, c) < 0;
    }
    
    /**
     * 检查点c是否在有向线ab的左侧或上方
     * @param a 点A
     * @param b 点B
     * @param c 点C
     * @return true如果c在ab左侧或上方
     */
    private static boolean leftOn(int[] a, int[] b, int[] c) {
        return area2(a, b, c) <= 0;
    }
    
    /**
     * 检查三点是否共线
     * @param a 点A
     * @param b 点B
     * @param c 点C
     * @return true如果三点共线
     */
    private static boolean collinear(int[] a, int[] b, int[] c) {
        return area2(a, b, c) == 0;
    }
    
    /**
     * 检查线段ab是否与cd正确相交
     * @param a 线段1的点A
     * @param b 线段1的点B
     * @param c 线段2的点C
     * @param d 线段2的点D
     * @return true如果正确相交
     */
    private static boolean intersectProp(int[] a, int[] b, int[] c, int[] d) {
        // 排除不正确的情况
        if (collinear(a, b, c) || collinear(a, b, d) ||
            collinear(c, d, a) || collinear(c, d, b)) {
            return false;
        }
        
        return xorb(left(a, b, c), left(a, b, d)) && xorb(left(c, d, a), left(c, d, b));
    }
    
    /**
     * 检查点c是否在线段ab上
     * @param a 线段的点A
     * @param b 线段的点B
     * @param c 要检查的点C
     * @return true如果c在ab上
     */
    private static boolean between(int[] a, int[] b, int[] c) {
        if (!collinear(a, b, c)) {
            return false;
        }
        // 如果ab不是垂直的，检查x上的位置；否则检查y
        if (a[0] != b[0]) {
            return ((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
        } else {
            return ((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
        }
    }
    
    /**
     * 检查线段ab和cd是否相交
     * @param a 线段1的点A
     * @param b 线段1的点B
     * @param c 线段2的点C
     * @param d 线段2的点D
     * @return true如果相交
     */
    private static boolean intersect(int[] a, int[] b, int[] c, int[] d) {
        if (intersectProp(a, b, c, d)) {
            return true;
        } else if (between(a, b, c) || between(a, b, d) ||
                   between(c, d, a) || between(c, d, b)) {
            return true;
        } else {
            return false;
        }
    }
    
    /**
     * 检查两个向量是否相等
     * @param a 向量A
     * @param b 向量B
     * @return true如果相等
     */
    private static boolean vequal(int[] a, int[] b) {
        return a[0] == b[0] && a[2] == b[2];
    }
    
    /**
     * 检查对角线(i,j)是否为多边形P的正确内部或外部对角线
     * @param i 顶点索引i
     * @param j 顶点索引j
     * @param n 顶点数量
     * @param verts 顶点数组
     * @param indices 索引数组
     * @return true如果是有效对角线
     */
    private static boolean diagonalie(int i, int j, int n, int[] verts, int[] indices) {
        int[] d0 = {verts[(indices[i] & 0x0fffffff) * 4], 
                   verts[(indices[i] & 0x0fffffff) * 4 + 1], 
                   verts[(indices[i] & 0x0fffffff) * 4 + 2]};
        int[] d1 = {verts[(indices[j] & 0x0fffffff) * 4], 
                   verts[(indices[j] & 0x0fffffff) * 4 + 1], 
                   verts[(indices[j] & 0x0fffffff) * 4 + 2]};
        
        // 对于P的每条边(k,k+1)
        for (int k = 0; k < n; k++) {
            int k1 = next(k, n);
            // 跳过入射到i或j的边
            if (!((k == i) || (k1 == i) || (k == j) || (k1 == j))) {
                int[] p0 = {verts[(indices[k] & 0x0fffffff) * 4], 
                           verts[(indices[k] & 0x0fffffff) * 4 + 1], 
                           verts[(indices[k] & 0x0fffffff) * 4 + 2]};
                int[] p1 = {verts[(indices[k1] & 0x0fffffff) * 4], 
                           verts[(indices[k1] & 0x0fffffff) * 4 + 1], 
                           verts[(indices[k1] & 0x0fffffff) * 4 + 2]};
                
                if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1)) {
                    continue;
                }
                
                if (intersect(d0, d1, p0, p1)) {
                    return false;
                }
            }
        }
        return true;
    }
    
    /**
     * 检查对角线(i,j)是否严格在多边形P内部
     * @param i 顶点索引i
     * @param j 顶点索引j
     * @param n 顶点数量
     * @param verts 顶点数组
     * @param indices 索引数组
     * @return true如果在内部
     */
    private static boolean inCone(int i, int j, int n, int[] verts, int[] indices) {
        int[] vi = {verts[(indices[i] & 0x0fffffff) * 4], 
                   verts[(indices[i] & 0x0fffffff) * 4 + 1], 
                   verts[(indices[i] & 0x0fffffff) * 4 + 2]};
        int[] vj = {verts[(indices[j] & 0x0fffffff) * 4], 
                   verts[(indices[j] & 0x0fffffff) * 4 + 1], 
                   verts[(indices[j] & 0x0fffffff) * 4 + 2]};
        int[] vi1 = {verts[(indices[next(i, n)] & 0x0fffffff) * 4], 
                    verts[(indices[next(i, n)] & 0x0fffffff) * 4 + 1], 
                    verts[(indices[next(i, n)] & 0x0fffffff) * 4 + 2]};
        int[] vin1 = {verts[(indices[prev(i, n)] & 0x0fffffff) * 4], 
                     verts[(indices[prev(i, n)] & 0x0fffffff) * 4 + 1], 
                     verts[(indices[prev(i, n)] & 0x0fffffff) * 4 + 2]};
        
        // 如果P[i]是凸顶点[i+1在(i-1,i)的左侧或上方]
        if (leftOn(vin1, vi, vi1)) {
            return left(vi, vj, vin1) && left(vj, vi, vi1);
        }
        // 假设(i-1,i,i+1)不共线，否则P[i]是凹顶点
        return !(leftOn(vi, vj, vi1) && leftOn(vj, vi, vin1));
    }
    
    /**
     * 检查(v_i, v_j)是否为P的正确内部对角线
     * @param i 顶点索引i
     * @param j 顶点索引j
     * @param n 顶点数量
     * @param verts 顶点数组
     * @param indices 索引数组
     * @return true如果是有效对角线
     */
    private static boolean diagonal(int i, int j, int n, int[] verts, int[] indices) {
        return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
    }
    
    /**
     * 三角化多边形
     * @param n 顶点数量
     * @param verts 顶点数组
     * @param indices 索引数组
     * @param tris 输出三角形数组
     * @return 三角形数量
     */
    private static int triangulate(int n, int[] verts, int[] indices, int[] tris) {
        int ntris = 0;
        int dst = 0;
        
        // 索引的最后一位用于指示顶点是否可以被移除
        for (int i = 0; i < n; i++) {
            int i1 = next(i, n);
            int i2 = next(i1, n);
            if (diagonal(i, i2, n, verts, indices)) {
                indices[i1] |= 0x80000000;
            }
        }
        
        while (n > 3) {
            int minLen = -1;
            int mini = -1;
            for (int i = 0; i < n; i++) {
                int i1 = next(i, n);
                if ((indices[i1] & 0x80000000) != 0) {
                    int[] p0 = {verts[(indices[i] & 0x0fffffff) * 4], 
                               verts[(indices[i] & 0x0fffffff) * 4 + 1], 
                               verts[(indices[i] & 0x0fffffff) * 4 + 2]};
                    int[] p2 = {verts[(indices[next(i1, n)] & 0x0fffffff) * 4], 
                               verts[(indices[next(i1, n)] & 0x0fffffff) * 4 + 1], 
                               verts[(indices[next(i1, n)] & 0x0fffffff) * 4 + 2]};
                    
                    int dx = p2[0] - p0[0];
                    int dy = p2[2] - p0[2];
                    int len = dx * dx + dy * dy;
                    
                    if (minLen < 0 || len < minLen) {
                        minLen = len;
                        mini = i;
                    }
                }
            }
            
            if (mini == -1) {
                // 不应该发生
                return -ntris;
            }
            
            int i = mini;
            int i1 = next(i, n);
            int i2 = next(i1, n);
            
            tris[dst++] = indices[i] & 0x0fffffff;
            tris[dst++] = indices[i1] & 0x0fffffff;
            tris[dst++] = indices[i2] & 0x0fffffff;
            ntris++;
            
            // 通过复制P[i+1]...P[n-1]向左移动一个索引来移除P[i1]
            n--;
            for (int k = i1; k < n; k++) {
                indices[k] = indices[k + 1];
            }
            
            if (i1 >= n) i1 = 0;
            i = prev(i1, n);
            // 更新对角线标志
            if (diagonal(prev(i, n), i1, n, verts, indices)) {
                indices[i] |= 0x80000000;
            } else {
                indices[i] &= 0x0fffffff;
            }
            
            if (diagonal(i, next(i1, n), n, verts, indices)) {
                indices[i1] |= 0x80000000;
            } else {
                indices[i1] &= 0x0fffffff;
            }
        }
        
        // 添加剩余的三角形
        tris[dst++] = indices[0] & 0x0fffffff;
        tris[dst++] = indices[1] & 0x0fffffff;
        tris[dst++] = indices[2] & 0x0fffffff;
        ntris++;
        
        return ntris;
    }
    
    /**
     * 计算多边形顶点数量
     * @param p 多边形数组
     * @param nvp 每个多边形的最大顶点数
     * @return 实际顶点数量
     */
    private static int countPolyVerts(int[] p, int nvp) {
        for (int i = 0; i < nvp; i++) {
            if (p[i] == RC_MESH_NULL_IDX) {
                return i;
            }
        }
        return nvp;
    }
    
    /**
     * 构建多边形网格
     * @param ctx 构建上下文
     * @param cset 轮廓集
     * @param nvp 每个多边形的最大顶点数
     * @param mesh 输出多边形网格
     * @return true如果构建成功
     */
    public static boolean buildPolyMesh(RecastContext ctx, ContourSet cset, int nvp, PolyMesh mesh) {
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_BUILD_POLYMESH);
        }
        
        try {
            int maxVertices = 0;
            int maxTris = 0;
            int maxVertsPerCont = 0;
            
            for (int i = 0; i < cset.nconts; i++) {
                // 跳过空轮廓
                if (cset.conts[i].nverts < 3) continue;
                
                maxVertices += cset.conts[i].nverts;
                maxVertsPerCont = rcMax(maxVertsPerCont, cset.conts[i].nverts);
                maxTris += cset.conts[i].nverts - 2;
            }
            
            if (maxVertices >= 0xfffe) {
                if (ctx != null) {
                    ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_ERROR, 
                           "buildPolyMesh: 顶点过多 %d (max: %d)", maxVertices, 0xfffe);
                }
                return false;
            }
            
            int[] verts = new int[maxVertices * 3];
            int[] polys = new int[maxTris * nvp];
            int[] regs = new int[maxTris];
            int[] areas = new int[maxTris];
            
            Arrays.fill(polys, RC_MESH_NULL_IDX);
            
            int[] firstVert = new int[VERTEX_BUCKET_COUNT];
            Arrays.fill(firstVert, -1);
            int[] nextVert = new int[maxVertices];
            
            IntRef nverts = new IntRef(0);
            int npolys = 0;
            
            for (int i = 0; i < cset.nconts; i++) {
                Contour cont = cset.conts[i];
                
                // 跳过空轮廓
                if (cont.nverts < 3) continue;
                
                // 三角化轮廓
                for (int j = 0; j < cont.nverts; j++) {
                    int[] v = {cont.verts[j * 4], cont.verts[j * 4 + 1], cont.verts[j * 4 + 2]};
                    verts[nverts.value * 3] = v[0];
                    verts[nverts.value * 3 + 1] = v[1];
                    verts[nverts.value * 3 + 2] = v[2];
                    nverts.value++;
                }
                
                int[] indices = new int[maxVertsPerCont];
                int[] tris = new int[maxVertsPerCont * 3];
                
                for (int j = 0; j < cont.nverts; j++) {
                    indices[j] = nverts.value - cont.nverts + j;
                }
                
                int ntris = triangulate(cont.nverts, verts, indices, tris);
                if (ntris <= 0) {
                    // Bad triangulation, skip.
                    if (ctx != null) {
                        ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_WARNING, 
                               "buildPolyMesh: 坏的三角化，contour %d.", i);
                    }
                    continue;
                }
                
                // 添加并合并顶点
                for (int j = 0; j < ntris; j++) {
                    int[] t = new int[nvp];
                    for (int k = 0; k < 3; k++) {
                        int v = tris[j * 3 + k];
                        int[] vert = {verts[v * 3], verts[v * 3 + 1], verts[v * 3 + 2]};
                        t[k] = addVertex(vert[0], vert[1], vert[2], verts, firstVert, nextVert, nverts);
                    }
                    
                    System.arraycopy(t, 0, polys, npolys * nvp, nvp);
                    regs[npolys] = cont.reg;
                    areas[npolys] = cont.area;
                    npolys++;
                }
            }
            
            // 移除不需要的三角形
            // TODO: 实现三角形过滤逻辑
            
            // 构建网格邻接关系
            if (!buildMeshAdjacency(polys, npolys, nverts.value, nvp)) {
                if (ctx != null) {
                    ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_ERROR, 
                           "buildPolyMesh: 无法构建网格邻接关系");
                }
                return false;
            }
            
            // 设置输出
            mesh.nverts = nverts.value;
            mesh.npolys = npolys;
            mesh.maxpolys = maxTris;
            mesh.nvp = nvp;
            
            mesh.verts = new int[nverts.value * 3];
            System.arraycopy(verts, 0, mesh.verts, 0, nverts.value * 3);
            
            mesh.polys = new int[npolys * nvp * 2];
            System.arraycopy(polys, 0, mesh.polys, 0, npolys * nvp);
            
            mesh.regs = new int[npolys];
            System.arraycopy(regs, 0, mesh.regs, 0, npolys);
            
            mesh.areas = new byte[npolys];
            for (int i = 0; i < npolys; i++) {
                mesh.areas[i] = (byte) areas[i];
            }

            // 设置边界框
            if (nverts.value > 0) {
                mesh.bmin[0] = mesh.bmax[0] = verts[0];
                mesh.bmin[1] = mesh.bmax[1] = verts[1];
                mesh.bmin[2] = mesh.bmax[2] = verts[2];
                
                for (int i = 1; i < nverts.value; i++) {
                    mesh.bmin[0] = rcMin(mesh.bmin[0], verts[i * 3]);
                    mesh.bmax[0] = rcMax(mesh.bmax[0], verts[i * 3]);
                    mesh.bmin[1] = rcMin(mesh.bmin[1], verts[i * 3 + 1]);
                    mesh.bmax[1] = rcMax(mesh.bmax[1], verts[i * 3 + 1]);
                    mesh.bmin[2] = rcMin(mesh.bmin[2], verts[i * 3 + 2]);
                    mesh.bmax[2] = rcMax(mesh.bmax[2], verts[i * 3 + 2]);
                }
            }
            
        } catch (OutOfMemoryError e) {
            if (ctx != null) {
                ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_ERROR, 
                       "buildPolyMesh: 内存不足");
            }
            return false;
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_POLYMESH);
        }
        
        return true;
    }
    
    /**
     * 构建网格邻接关系
     * @param polys 多边形数组
     * @param npolys 多边形数量
     * @param nverts 顶点数量
     * @param vertsPerPoly 每个多边形的顶点数
     * @return true如果成功
     */
    private static boolean buildMeshAdjacency(int[] polys, int npolys, int nverts, int vertsPerPoly) {
        // 基于Eric Lengyel的代码
        // http://www.terathon.com/code/edges.php
        
        int maxEdgeCount = npolys * vertsPerPoly;
        int[] firstEdge = new int[nverts + maxEdgeCount];
        int[] nextEdge = new int[maxEdgeCount];
        Edge[] edges = new Edge[maxEdgeCount];
        
        Arrays.fill(firstEdge, 0, nverts, RC_MESH_NULL_IDX);
        
        for (int i = 0; i < maxEdgeCount; i++) {
            edges[i] = new Edge();
        }
        
        int edgeCount = 0;
        
        for (int i = 0; i < npolys; i++) {
            int t = i * vertsPerPoly * 2;
            for (int j = 0; j < vertsPerPoly; j++) {
                if (polys[t + j] == RC_MESH_NULL_IDX) break;
                int v0 = polys[t + j];
                int v1 = (j + 1 >= vertsPerPoly || polys[t + j + 1] == RC_MESH_NULL_IDX) ? polys[t] : polys[t + j + 1];
                if (v0 < v1) {
                    Edge edge = edges[edgeCount];
                    edge.vert[0] = v0;
                    edge.vert[1] = v1;
                    edge.poly[0] = i;
                    edge.polyEdge[0] = j;
                    edge.poly[1] = i;
                    edge.polyEdge[1] = 0;
                    // 插入边
                    nextEdge[edgeCount] = firstEdge[v0];
                    firstEdge[v0] = edgeCount;
                    edgeCount++;
                }
            }
        }
        
        for (int i = 0; i < npolys; i++) {
            int t = i * vertsPerPoly * 2;
            for (int j = 0; j < vertsPerPoly; j++) {
                if (polys[t + j] == RC_MESH_NULL_IDX) break;
                int v0 = polys[t + j];
                int v1 = (j + 1 >= vertsPerPoly || polys[t + j + 1] == RC_MESH_NULL_IDX) ? polys[t] : polys[t + j + 1];
                if (v0 > v1) {
                    for (int e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = nextEdge[e]) {
                        Edge edge = edges[e];
                        if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1]) {
                            edge.poly[1] = i;
                            edge.polyEdge[1] = j;
                            break;
                        }
                    }
                }
            }
        }
        
        // 存储邻接关系
        for (int i = 0; i < edgeCount; i++) {
            Edge e = edges[i];
            if (e.poly[0] != e.poly[1]) {
                int p0 = e.poly[0] * vertsPerPoly * 2;
                int p1 = e.poly[1] * vertsPerPoly * 2;
                polys[p0 + vertsPerPoly + e.polyEdge[0]] = e.poly[1];
                polys[p1 + vertsPerPoly + e.polyEdge[1]] = e.poly[0];
            }
        }
        
        return true;
    }
    
    /**
     * 整数引用类型（用于模拟C++的引用传递）
     */
    private static class IntRef {
        public int value;
        
        public IntRef(int value) {
            this.value = value;
        }
    }
} 