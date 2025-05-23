package odin.recast.algorithms;

import odin.recast.core.RecastHeightfield.CompactHeightfield;
import odin.recast.core.RecastHeightfield.CompactCell;
import odin.recast.core.RecastHeightfield.CompactSpan;
import odin.recast.core.RecastContour.Contour;
import odin.recast.core.RecastContour.ContourSet;
import odin.recast.config.RecastEnums.TimerLabel;
import odin.recast.config.RecastEnums.ContourBuildFlags;

import java.util.ArrayList;
import java.util.List;

import static odin.recast.utils.RecastMath.*;
import static odin.recast.config.RecastConstants.*;
import static odin.recast.algorithms.RecastCompactHeightfield.*;

/**
 * Recast轮廓生成算法
 * 翻译自UE5 RecastContour.cpp
 * 
 * 负责从紧凑高度字段中提取轮廓，为多边形网格生成做准备
 * 
 * @author UE5NavMesh4J
 */
public final class RecastContour {
    
    /** 私有构造函数防止实例化 */
    private RecastContour() {
        throw new UnsupportedOperationException("工具类不能被实例化");
    }
    
    /**
     * 获取角高度
     * @param x X坐标
     * @param y Y坐标
     * @param i span索引
     * @param dir 方向
     * @param chf 紧凑高度字段
     * @return 角高度和边界顶点标志
     */
    private static CornerHeightResult getCornerHeight(int x, int y, int i, int dir, CompactHeightfield chf) {
        CompactSpan s = chf.spans[i];
        int ch = s.y;
        int dirp = (dir + 1) & 0x3;
        boolean isBorderVertex = false;
        
        long[] regs = new long[4];
        
        // 组合区域和区域代码以防止在两个区域之间的边界顶点被移除
        regs[0] = chf.spans[i].reg | ((long)(chf.areas[i] & 0xFF) << 16);
        
        if (getConnection(s, dir) != RC_NOT_CONNECTED) {
            int ax = x + getDirOffsetX(dir);
            int ay = y + getDirOffsetY(dir);
            int ai = chf.cells[ax + ay * chf.width].index + getConnection(s, dir);
            CompactSpan as = chf.spans[ai];
            ch = rcMax(ch, as.y);
            regs[1] = chf.spans[ai].reg | ((long)(chf.areas[ai] & 0xFF) << 16);
            
            if (getConnection(as, dirp) != RC_NOT_CONNECTED) {
                int ax2 = ax + getDirOffsetX(dirp);
                int ay2 = ay + getDirOffsetY(dirp);
                int ai2 = chf.cells[ax2 + ay2 * chf.width].index + getConnection(as, dirp);
                CompactSpan as2 = chf.spans[ai2];
                ch = rcMax(ch, as2.y);
                regs[2] = chf.spans[ai2].reg | ((long)(chf.areas[ai2] & 0xFF) << 16);
            }
        }
        
        if (getConnection(s, dirp) != RC_NOT_CONNECTED) {
            int ax = x + getDirOffsetX(dirp);
            int ay = y + getDirOffsetY(dirp);
            int ai = chf.cells[ax + ay * chf.width].index + getConnection(s, dirp);
            CompactSpan as = chf.spans[ai];
            ch = rcMax(ch, as.y);
            regs[3] = chf.spans[ai].reg | ((long)(chf.areas[ai] & 0xFF) << 16);
            
            if (getConnection(as, dir) != RC_NOT_CONNECTED) {
                int ax2 = ax + getDirOffsetX(dir);
                int ay2 = ay + getDirOffsetY(dir);
                int ai2 = chf.cells[ax2 + ay2 * chf.width].index + getConnection(as, dir);
                CompactSpan as2 = chf.spans[ai2];
                ch = rcMax(ch, as2.y);
                regs[2] = chf.spans[ai2].reg | ((long)(chf.areas[ai2] & 0xFF) << 16);
            }
        }
        
        // 检查顶点是否为特殊边缘顶点，这些顶点稍后会被移除
        for (int j = 0; j < 4; j++) {
            int a = j;
            int b = (j + 1) & 0x3;
            int c = (j + 2) & 0x3;
            int d = (j + 3) & 0x3;
            
            // 顶点是边界顶点，有两个相同的外部单元格连续出现，
            // 后面跟着两个内部单元格，并且没有区域超出边界
            boolean twoSameExts = (regs[a] & regs[b] & RC_BORDER_REG) != 0 && regs[a] == regs[b];
            boolean twoInts = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0;
            boolean intsSameArea = (regs[c] >> 16) == (regs[d] >> 16);
            boolean noZeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0;
            
            if (twoSameExts && twoInts && intsSameArea && noZeros) {
                isBorderVertex = true;
                break;
            }
        }
        
        return new CornerHeightResult(ch, isBorderVertex);
    }
    
    /**
     * 遍历轮廓
     * @param x 起始X坐标
     * @param y 起始Y坐标
     * @param i 起始span索引
     * @param chf 紧凑高度字段
     * @param flags 访问标志
     * @param points 输出点列表
     */
    private static void walkContour(int x, int y, int i, CompactHeightfield chf, 
                                   byte[] flags, List<Integer> points) {
        // 选择第一个未连接的边
        int dir = 0;
        while ((flags[i] & (1 << dir)) == 0) {
            dir++;
        }
        
        int startDir = dir;
        int starti = i;
        
        int area = chf.areas[i] & 0xFF;
        
        int iter = 0;
        while (++iter < 40000) {
            if ((flags[i] & (1 << dir)) != 0) {
                // 选择边角
                CornerHeightResult cornerResult = getCornerHeight(x, y, i, dir, chf);
                boolean isBorderVertex = cornerResult.isBorderVertex;
                boolean isAreaBorder = false;
                
                int px = x;
                int py = cornerResult.height;
                int pz = y;
                
                switch (dir) {
                    case 0: pz++; break;
                    case 1: px++; pz++; break;
                    case 2: px++; break;
                }
                
                int r = 0;
                CompactSpan s = chf.spans[i];
                if (getConnection(s, dir) != RC_NOT_CONNECTED) {
                    int ax = x + getDirOffsetX(dir);
                    int ay = y + getDirOffsetY(dir);
                    int ai = chf.cells[ax + ay * chf.width].index + getConnection(s, dir);
                    r = chf.spans[ai].reg;
                    if (area != (chf.areas[ai] & 0xFF)) {
                        isAreaBorder = true;
                    }
                }
                
                if (isBorderVertex) {
                    r |= RC_BORDER_VERTEX;
                }
                if (isAreaBorder) {
                    r |= RC_AREA_BORDER;
                }
                
                points.add(px);
                points.add(py);
                points.add(pz);
                points.add(r);
                
                flags[i] &= ~(1 << dir); // 移除已访问的边
                dir = (dir + 1) & 0x3;  // 顺时针旋转
            } else {
                int ni = -1;
                int nx = x + getDirOffsetX(dir);
                int ny = y + getDirOffsetY(dir);
                CompactSpan s = chf.spans[i];
                
                if (getConnection(s, dir) != RC_NOT_CONNECTED) {
                    CompactCell nc = chf.cells[nx + ny * chf.width];
                    ni = nc.index + getConnection(s, dir);
                }
                
                if (ni == -1) {
                    // 不应该发生
                    return;
                }
                
                x = nx;
                y = ny;
                i = ni;
                dir = (dir + 3) & 0x3; // 逆时针旋转
            }
            
            if (starti == i && startDir == dir) {
                break;
            }
        }
    }
    
    /**
     * 计算点到线段的距离平方
     * @param x 点X坐标
     * @param z 点Z坐标
     * @param px 线段起点X
     * @param pz 线段起点Z
     * @param qx 线段终点X
     * @param qz 线段终点Z
     * @return 距离平方
     */
    private static float distancePtSeg(int x, int z, int px, int pz, int qx, int qz) {
        float pqx = qx - px;
        float pqz = qz - pz;
        float dx = x - px;
        float dz = z - pz;
        float d = pqx * pqx + pqz * pqz;
        float t = pqx * dx + pqz * dz;
        
        if (d > 0) {
            t /= d;
        }
        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }
        
        dx = px + t * pqx - x;
        dz = pz + t * pqz - z;
        
        return dx * dx + dz * dz;
    }
    
    /**
     * 简化轮廓
     * @param points 原始点列表
     * @param simplified 简化后点列表
     * @param maxError 最大误差
     * @param maxEdgeLen 最大边长
     * @param buildFlags 构建标志
     */
    private static void simplifyContour(List<Integer> points, List<Integer> simplified,
                                       float maxError, int maxEdgeLen, int buildFlags) {
        // 添加初始点
        boolean hasConnections = false;
        for (int i = 0; i < points.size(); i += 4) {
            if ((points.get(i + 3) & RC_CONTOUR_REG_MASK) != 0) {
                hasConnections = true;
                break;
            }
        }
        
        if (hasConnections) {
            // 轮廓有一些到其他区域的门户
            // 在区域变化的每个位置添加新点
            for (int i = 0, ni = points.size() / 4; i < ni; i++) {
                int ii = (i + 1) % ni;
                boolean differentRegs = (points.get(i * 4 + 3) & RC_CONTOUR_REG_MASK) != 
                                       (points.get(ii * 4 + 3) & RC_CONTOUR_REG_MASK);
                boolean areaBorders = (points.get(i * 4 + 3) & RC_AREA_BORDER) != 
                                     (points.get(ii * 4 + 3) & RC_AREA_BORDER);
                if (differentRegs || areaBorders) {
                    simplified.add(points.get(i * 4 + 0));
                    simplified.add(points.get(i * 4 + 1));
                    simplified.add(points.get(i * 4 + 2));
                    simplified.add(i);
                }
            }
        }
        
        if (simplified.isEmpty()) {
            // 如果完全没有连接，为简化过程创建一些初始点
            // 找到轮廓的左下和右上顶点
            int llx = points.get(0);
            int lly = points.get(1);
            int llz = points.get(2);
            int lli = 0;
            int urx = points.get(0);
            int ury = points.get(1);
            int urz = points.get(2);
            int uri = 0;
            
            for (int i = 0; i < points.size(); i += 4) {
                int x = points.get(i + 0);
                int y = points.get(i + 1);
                int z = points.get(i + 2);
                if (x < llx || (x == llx && z < llz)) {
                    llx = x;
                    lly = y;
                    llz = z;
                    lli = i / 4;
                }
                if (x > urx || (x == urx && z > urz)) {
                    urx = x;
                    ury = y;
                    urz = z;
                    uri = i / 4;
                }
            }
            
            simplified.add(llx);
            simplified.add(lly);
            simplified.add(llz);
            simplified.add(lli);
            
            simplified.add(urx);
            simplified.add(ury);
            simplified.add(urz);
            simplified.add(uri);
        }
        
        // 添加点直到所有原始点都在简化形状的误差容忍范围内
        int pn = points.size() / 4;
        for (int i = 0; i < simplified.size() / 4; ) {
            int ii = (i + 1) % (simplified.size() / 4);
            
            int ax = simplified.get(i * 4 + 0);
            int az = simplified.get(i * 4 + 2);
            int ai = simplified.get(i * 4 + 3);
            
            int bx = simplified.get(ii * 4 + 0);
            int bz = simplified.get(ii * 4 + 2);
            int bi = simplified.get(ii * 4 + 3);
            
            // 找到从线段的最大偏差
            float maxd = 0;
            int maxi = -1;
            int ci, cinc, endi;
            
            // 按词典顺序遍历线段，以便在遍历相反线段时以类似方式计算最大偏差
            if (bx > ax || (bx == ax && bz > az)) {
                cinc = 1;
                ci = (ai + cinc) % pn;
                endi = bi;
            } else {
                cinc = pn - 1;
                ci = (bi + cinc) % pn;
                endi = ai;
            }
            
            // 只细分外边缘或区域之间的边缘
            if ((points.get(ci * 4 + 3) & RC_CONTOUR_REG_MASK) == 0 ||
                (points.get(ci * 4 + 3) & RC_AREA_BORDER) != 0) {
                while (ci != endi) {
                    float d = distancePtSeg(points.get(ci * 4 + 0), points.get(ci * 4 + 2), ax, az, bx, bz);
                    if (d > maxd) {
                        maxd = d;
                        maxi = ci;
                    }
                    ci = (ci + cinc) % pn;
                }
            }
            
            // 如果最大偏差大于可接受误差，添加新点，否则继续下一个线段
            if (maxi != -1 && maxd > (maxError * maxError)) {
                // 为新点添加空间
                for (int j = 0; j < 4; j++) {
                    simplified.add(0);
                }
                int n = simplified.size() / 4;
                for (int j = n - 1; j > i; j--) {
                    simplified.set(j * 4 + 0, simplified.get((j - 1) * 4 + 0));
                    simplified.set(j * 4 + 1, simplified.get((j - 1) * 4 + 1));
                    simplified.set(j * 4 + 2, simplified.get((j - 1) * 4 + 2));
                    simplified.set(j * 4 + 3, simplified.get((j - 1) * 4 + 3));
                }
                // 添加点
                simplified.set((i + 1) * 4 + 0, points.get(maxi * 4 + 0));
                simplified.set((i + 1) * 4 + 1, points.get(maxi * 4 + 1));
                simplified.set((i + 1) * 4 + 2, points.get(maxi * 4 + 2));
                simplified.set((i + 1) * 4 + 3, maxi);
            } else {
                i++;
            }
        }
        
        // 分割过长的边
        if (maxEdgeLen > 0 && (buildFlags & (ContourBuildFlags.RC_CONTOUR_TESS_WALL_EDGES.getValue() | 
                                            ContourBuildFlags.RC_CONTOUR_TESS_AREA_EDGES.getValue())) != 0) {
            for (int i = 0; i < simplified.size() / 4; ) {
                int ii = (i + 1) % (simplified.size() / 4);
                
                int ax = simplified.get(i * 4 + 0);
                int az = simplified.get(i * 4 + 2);
                int ai = simplified.get(i * 4 + 3);
                
                int bx = simplified.get(ii * 4 + 0);
                int bz = simplified.get(ii * 4 + 2);
                int bi = simplified.get(ii * 4 + 3);
                
                // 找到从线段的最大偏差
                int maxi = -1;
                int ci = (ai + 1) % pn;
                
                // 只细分外边缘或区域之间的边缘
                boolean tess = false;
                // 墙边缘
                if ((buildFlags & ContourBuildFlags.RC_CONTOUR_TESS_WALL_EDGES.getValue()) != 0 && 
                    (points.get(ci * 4 + 3) & RC_CONTOUR_REG_MASK) == 0) {
                    tess = true;
                }
                // 区域之间的边缘
                if ((buildFlags & ContourBuildFlags.RC_CONTOUR_TESS_AREA_EDGES.getValue()) != 0 && 
                    (points.get(ci * 4 + 3) & RC_AREA_BORDER) != 0) {
                    tess = true;
                }
                
                if (tess) {
                    int dx = bx - ax;
                    int dz = bz - az;
                    if (dx * dx + dz * dz > maxEdgeLen * maxEdgeLen) {
                        // 基于词典顺序的线段进行四舍五入，以便无论以哪个方向遍历线段，最大细分都是一致的
                        int n = bi < ai ? (bi + pn - ai) : (bi - ai);
                        if (n > 1) {
                            if (bx > ax || (bx == ax && bz > az)) {
                                maxi = (ai + n / 2) % pn;
                            } else {
                                maxi = (ai + (n + 1) / 2) % pn;
                            }
                        }
                    }
                }
                
                // 如果最大偏差大于可接受误差，添加新点，否则继续下一个线段
                if (maxi != -1) {
                    // 为新点添加空间
                    for (int j = 0; j < 4; j++) {
                        simplified.add(0);
                    }
                    int n = simplified.size() / 4;
                    for (int j = n - 1; j > i; j--) {
                        simplified.set(j * 4 + 0, simplified.get((j - 1) * 4 + 0));
                        simplified.set(j * 4 + 1, simplified.get((j - 1) * 4 + 1));
                        simplified.set(j * 4 + 2, simplified.get((j - 1) * 4 + 2));
                        simplified.set(j * 4 + 3, simplified.get((j - 1) * 4 + 3));
                    }
                    // 添加点
                    simplified.set((i + 1) * 4 + 0, points.get(maxi * 4 + 0));
                    simplified.set((i + 1) * 4 + 1, points.get(maxi * 4 + 1));
                    simplified.set((i + 1) * 4 + 2, points.get(maxi * 4 + 2));
                    simplified.set((i + 1) * 4 + 3, maxi);
                } else {
                    i++;
                }
            }
        }
        
        for (int i = 0; i < simplified.size() / 4; i++) {
            // 边缘顶点标志取自当前原始点，邻居区域取自下一个原始点
            int ai = (simplified.get(i * 4 + 3) + 1) % pn;
            int bi = simplified.get(i * 4 + 3);
            simplified.set(i * 4 + 3, (points.get(ai * 4 + 3) & (RC_CONTOUR_REG_MASK | RC_AREA_BORDER)) | 
                          (points.get(bi * 4 + 3) & RC_BORDER_VERTEX));
        }
    }
    
    /**
     * 移除退化线段
     * @param simplified 简化后的点列表
     */
    private static void removeDegenerateSegments(List<Integer> simplified) {
        // 移除在xz平面上相等的相邻顶点，否则三角化器会混乱
        for (int i = 0; i < simplified.size() / 4; i++) {
            int ni = i + 1;
            if (ni >= (simplified.size() / 4)) {
                ni = 0;
            }
            
            if (simplified.get(i * 4 + 0).equals(simplified.get(ni * 4 + 0)) &&
                simplified.get(i * 4 + 2).equals(simplified.get(ni * 4 + 2))) {
                // 退化线段，移除
                for (int j = i; j < simplified.size() / 4 - 1; j++) {
                    simplified.set(j * 4 + 0, simplified.get((j + 1) * 4 + 0));
                    simplified.set(j * 4 + 1, simplified.get((j + 1) * 4 + 1));
                    simplified.set(j * 4 + 2, simplified.get((j + 1) * 4 + 2));
                    simplified.set(j * 4 + 3, simplified.get((j + 1) * 4 + 3));
                }
                // 移除最后4个元素
                for (int j = 0; j < 4; j++) {
                    simplified.remove(simplified.size() - 1);
                }
            }
        }
    }
    
    /**
     * 构建轮廓
     * @param ctx 构建上下文
     * @param chf 紧凑高度字段
     * @param maxError 最大误差
     * @param maxEdgeLen 最大边长
     * @param cset 输出轮廓集
     * @param buildFlags 构建标志
     * @return true如果成功构建
     */
    public static boolean buildContours(RecastContext ctx, CompactHeightfield chf,
                                       float maxError, int maxEdgeLen, 
                                       ContourSet cset, int buildFlags) {
        
        if (ctx != null) {
            ctx.startTimer(TimerLabel.RC_TIMER_BUILD_CONTOURS);
        }
        
        int w = chf.width;
        int h = chf.height;
        int borderSize = chf.borderSize != null ? chf.borderSize.low : 0;
        
        try {
            // 分配标志数组
            byte[] flags = new byte[chf.spanCount];
            
            if (ctx != null) {
                ctx.startTimer(TimerLabel.RC_TIMER_BUILD_CONTOURS_TRACE);
            }
            
            // 标记边界
            for (int y = 0; y < h; y++) {
                for (int x = 0; x < w; x++) {
                    CompactCell c = chf.cells[x + y * w];
                    for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                        int res = 0;
                        CompactSpan s = chf.spans[i];
                        if (chf.spans[i].reg == 0 || (chf.spans[i].reg & RC_BORDER_REG) != 0) {
                            flags[i] = 0;
                            continue;
                        }
                        
                        for (int dir = 0; dir < 4; dir++) {
                            int r = 0;
                            if (getConnection(s, dir) != RC_NOT_CONNECTED) {
                                int ax = x + getDirOffsetX(dir);
                                int ay = y + getDirOffsetY(dir);
                                int ai = chf.cells[ax + ay * w].index + getConnection(s, dir);
                                r = chf.spans[ai].reg;
                            }
                            if (r == chf.spans[i].reg) {
                                res |= (1 << dir);
                            }
                        }
                        flags[i] = (byte)(res ^ 0xf); // 反转边缘
                    }
                }
            }
            
            if (ctx != null) {
                ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_CONTOURS_TRACE);
                ctx.startTimer(TimerLabel.RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
            }
            
            List<Contour> contours = new ArrayList<>();
            List<Integer> points = new ArrayList<>();
            List<Integer> simplified = new ArrayList<>();
            
            for (int y = 0; y < h; y++) {
                for (int x = 0; x < w; x++) {
                    CompactCell c = chf.cells[x + y * w];
                    for (int i = c.index, ni = c.index + c.count; i < ni; i++) {
                        if (flags[i] == 0 || chf.spans[i].reg == 0 || (chf.spans[i].reg & RC_BORDER_REG) != 0) {
                            continue;
                        }
                        
                        int reg = chf.spans[i].reg;
                        int area = chf.areas[i] & 0xFF;
                        
                        points.clear();
                        simplified.clear();
                        
                        walkContour(x, y, i, chf, flags, points);
                        
                        simplifyContour(points, simplified, maxError, maxEdgeLen, buildFlags);
                        removeDegenerateSegments(simplified);
                        
                        // 存储轮廓
                        if (simplified.size() / 4 >= 3) {
                            Contour cont = new Contour();
                            cont.verts = new int[simplified.size()];
                            for (int j = 0; j < simplified.size(); j++) {
                                cont.verts[j] = simplified.get(j);
                            }
                            cont.nverts = simplified.size() / 4;
                            cont.reg = reg;
                            cont.area = area;
                            contours.add(cont);
                        }
                    }
                }
            }
            
            // 将轮廓转移到输出
            cset.conts = contours.toArray(new Contour[0]);
            cset.nconts = contours.size();
            
            if (borderSize > 0) {
                // 如果网格在瓦片边界上，有些轮廓将沿着边界
                // 移除这些边界顶点
                for (int i = 0; i < cset.nconts; i++) {
                    Contour cont = cset.conts[i];
                    removeBorderVertices(cont, borderSize, w, h);
                }
            }
            
            if (ctx != null) {
                ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
            }
            
        } catch (OutOfMemoryError e) {
            if (ctx != null) {
                ctx.log(odin.recast.config.RecastEnums.LogCategory.RC_LOG_ERROR, 
                       "buildContours: 内存不足");
            }
            return false;
        }
        
        if (ctx != null) {
            ctx.stopTimer(TimerLabel.RC_TIMER_BUILD_CONTOURS);
        }
        
        return true;
    }
    
    /**
     * 移除边界顶点
     * @param cont 轮廓
     * @param borderSize 边界大小
     * @param w 宽度
     * @param h 高度
     */
    private static void removeBorderVertices(Contour cont, int borderSize, int w, int h) {
        // TODO: 实现边界顶点移除逻辑
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
    
    /**
     * 角高度结果
     */
    private static class CornerHeightResult {
        public final int height;
        public final boolean isBorderVertex;
        
        public CornerHeightResult(int height, boolean isBorderVertex) {
            this.height = height;
            this.isBorderVertex = isBorderVertex;
        }
    }
} 