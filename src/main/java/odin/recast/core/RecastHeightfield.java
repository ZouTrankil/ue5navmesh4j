package odin.recast.core;

import odin.recast.config.RecastConfig;
import static odin.recast.utils.RecastMath.*;

/**
 * Recast高度字段相关数据结构
 * 翻译自UE5 Recast.h中的高度字段相关结构体
 * 
 * @author UE5NavMesh4J
 */
public class RecastHeightfield {
    
    /**
     * 表示受阻空间的动态高度字段
     * 翻译自rcHeightfield结构体
     */
    public static class Heightfield {
        /** 高度字段的宽度（沿x轴的单元格单位） */
        public int width;
        
        /** 高度字段的高度（沿z轴的单元格单位） */
        public int height;
        
        /** 世界空间中的最小边界 [(x, y, z)] */
        public float[] bmin = new float[3];
        
        /** 世界空间中的最大边界 [(x, y, z)] */
        public float[] bmax = new float[3];
        
        /** 每个单元格的大小（在xz平面上） */
        public float cs;
        
        /** 每个单元格的高度（y轴的最小增量） */
        public float ch;
        
        /** span的高度字段 (width*height) */
        public RecastSpan.Span[][] spans;
        
        /** span池的链表 */
        public RecastSpan.SpanPool pools;
        
        /** 下一个空闲span */
        public RecastSpan.Span freelist;
        
        // UE5新光栅化器的扩展
        /** 边缘碰撞标志 (h + 1) */
        public RecastSpan.EdgeHit[] edgeHits;
        
        /** 行扩展信息 (h) */
        public RecastSpan.RowExt[] rowExt;
        
        /** 临时span的高度字段 (width*height) */
        public RecastSpan.TempSpan[][] tempspans;
        
        /**
         * 默认构造函数
         */
        public Heightfield() {
            this.width = 0;
            this.height = 0;
            this.cs = 0.0f;
            this.ch = 0.0f;
            this.spans = null;
            this.pools = null;
            this.freelist = null;
            this.edgeHits = null;
            this.rowExt = null;
            this.tempspans = null;
        }
        
        /**
         * 初始化高度字段
         * @param width 宽度
         * @param height 高度
         * @param bmin 最小边界
         * @param bmax 最大边界
         * @param cs 单元格大小
         * @param ch 单元格高度
         */
        public void init(int width, int height, float[] bmin, float[] bmax, float cs, float ch) {
            this.width = width;
            this.height = height;
            rcVcopy(this.bmin, bmin);
            rcVcopy(this.bmax, bmax);
            this.cs = cs;
            this.ch = ch;
            
            // 分配span数组
            this.spans = new RecastSpan.Span[width * height][];
            
            // 初始化UE5扩展结构
            this.edgeHits = new RecastSpan.EdgeHit[height + 1];
            for (int i = 0; i <= height; i++) {
                this.edgeHits[i] = new RecastSpan.EdgeHit();
            }
            
            this.rowExt = new RecastSpan.RowExt[height];
            for (int i = 0; i < height; i++) {
                this.rowExt[i] = new RecastSpan.RowExt();
            }
            
            this.tempspans = new RecastSpan.TempSpan[width * height][];
        }
        
        /**
         * 获取指定位置的span
         * @param x X坐标
         * @param z Z坐标
         * @return span链表的头节点
         */
        public RecastSpan.Span getSpan(int x, int z) {
            if (x < 0 || x >= width || z < 0 || z >= height) {
                return null;
            }
            
            int index = z * width + x;
            return spans[index] != null && spans[index].length > 0 ? spans[index][0] : null;
        }
        
        /**
         * 添加span到指定位置
         * @param x X坐标
         * @param z Z坐标
         * @param span 要添加的span
         */
        public void addSpan(int x, int z, RecastSpan.Span span) {
            if (x < 0 || x >= width || z < 0 || z >= height || span == null) {
                return;
            }
            
            int index = z * width + x;
            if (spans[index] == null) {
                spans[index] = new RecastSpan.Span[1];
                spans[index][0] = span;
            } else {
                // 插入到链表头部
                span.next = spans[index][0];
                spans[index][0] = span;
            }
        }
        
        /**
         * 释放资源
         */
        public void dispose() {
            spans = null;
            
            // 释放span池链表
            RecastSpan.SpanPool pool = pools;
            while (pool != null) {
                RecastSpan.SpanPool next = pool.next;
                pool = next;
            }
            pools = null;
            freelist = null;
            
            edgeHits = null;
            rowExt = null;
            tempspans = null;
        }
    }
    
    /**
     * 提供紧凑高度字段中单元格列内容的信息
     * 翻译自rcCompactCell结构体
     */
    public static class CompactCell {
        /** 列中第一个span的索引 */
        public int index;
        
        /** 列中span的数量 */
        public int count;
        
        /**
         * 默认构造函数
         */
        public CompactCell() {
            this.index = 0;
            this.count = 0;
        }
        
        /**
         * 带参数的构造函数
         * @param index 第一个span的索引
         * @param count span数量
         */
        public CompactCell(int index, int count) {
            this.index = index;
            this.count = count;
        }
        
        /**
         * 检查单元格是否为空
         * @return true如果单元格为空
         */
        public boolean isEmpty() {
            return count == 0;
        }
    }
    
    /**
     * 表示紧凑高度字段中一段未阻塞的空间
     * 翻译自rcCompactSpan结构体
     */
    public static class CompactSpan {
        /** span的下限（从高度字段的基础测量） */
        public int y;
        
        /** 打包的邻居连接数据 */
        public int con;
        
        /** span所属区域的id（如果不在区域中则为零） */
        public int reg;
        
        /** span的高度（从y测量） */
        public int h;
        
        /**
         * 默认构造函数
         */
        public CompactSpan() {
            this.y = 0;
            this.con = 0;
            this.reg = 0;
            this.h = 0;
        }
        
        /**
         * 带参数的构造函数
         * @param y span下限
         * @param con 连接数据
         * @param reg 区域id
         * @param h span高度
         */
        public CompactSpan(int y, int con, int reg, int h) {
            this.y = y;
            this.con = con;
            this.reg = reg;
            this.h = h;
        }
        
        /**
         * 获取指定方向的连接
         * @param dir 方向（0-3）
         * @return 连接值
         */
        public int getConnection(int dir) {
            if (dir < 0 || dir > 3) {
                return 0;
            }
            return (con >> (dir * 6)) & 0x3f;
        }
        
        /**
         * 设置指定方向的连接
         * @param dir 方向（0-3）
         * @param value 连接值
         */
        public void setConnection(int dir, int value) {
            if (dir < 0 || dir > 3) {
                return;
            }
            
            int shift = dir * 6;
            con = (con & ~(0x3f << shift)) | ((value & 0x3f) << shift);
        }
        
        /**
         * 获取span的顶部Y坐标
         * @return 顶部Y坐标
         */
        public int getTop() {
            return y + h;
        }
        
        /**
         * 检查是否与另一个span连接
         * @param dir 方向
         * @return true如果连接
         */
        public boolean isConnected(int dir) {
            return getConnection(dir) != 0x3f;
        }
    }
    
    /**
     * 表示未阻塞空间的紧凑静态高度字段
     * 翻译自rcCompactHeightfield结构体
     */
    public static class CompactHeightfield {
        /** 高度字段的宽度（沿x轴的单元格单位） */
        public int width;
        
        /** 高度字段的高度（沿z轴的单元格单位） */
        public int height;
        
        /** 高度字段中span的数量 */
        public int spanCount;
        
        /** 字段构建期间使用的可行走高度 */
        public int walkableHeight;
        
        /** 字段构建期间使用的可行走攀爬 */
        public int walkableClimb;
        
        /** 字段构建期间使用的AABB边界大小 */
        public RecastConfig.BorderSize borderSize;
        
        /** 字段内任何span的最大距离值 */
        public int maxDistance;
        
        /** 字段内任何span的最大区域id */
        public int maxRegions;
        
        /** 世界空间中的最小边界 [(x, y, z)] */
        public float[] bmin = new float[3];
        
        /** 世界空间中的最大边界 [(x, y, z)] */
        public float[] bmax = new float[3];
        
        /** 每个单元格的大小（在xz平面上） */
        public float cs;
        
        /** 每个单元格的高度（y轴的最小增量） */
        public float ch;
        
        /** 单元格数组 [大小: width*height] */
        public CompactCell[] cells;
        
        /** span数组 [大小: spanCount] */
        public CompactSpan[] spans;
        
        /** 包含边界距离数据的数组 [大小: spanCount] */
        public int[] dist;
        
        /** 包含区域id数据的数组 [大小: spanCount] */
        public byte[] areas;
        
        /** 包含区域标识数据的数组 [大小: spanCount] */
        public int[] regs;
        
        /**
         * 默认构造函数
         */
        public CompactHeightfield() {
            this.width = 0;
            this.height = 0;
            this.spanCount = 0;
            this.walkableHeight = 0;
            this.walkableClimb = 0;
            this.borderSize = new RecastConfig.BorderSize();
            this.maxDistance = 0;
            this.maxRegions = 0;
            this.cs = 0.0f;
            this.ch = 0.0f;
            this.cells = null;
            this.spans = null;
            this.dist = null;
            this.areas = null;
            this.regs = null;
        }
        
        /**
         * 初始化紧凑高度字段
         * @param width 宽度
         * @param height 高度
         * @param spanCount span数量
         * @param walkableHeight 可行走高度
         * @param walkableClimb 可行走攀爬
         */
        public void init(int width, int height, int spanCount, int walkableHeight, int walkableClimb) {
            this.width = width;
            this.height = height;
            this.spanCount = spanCount;
            this.walkableHeight = walkableHeight;
            this.walkableClimb = walkableClimb;
            
            // 分配数组
            this.cells = new CompactCell[width * height];
            for (int i = 0; i < width * height; i++) {
                this.cells[i] = new CompactCell();
            }
            
            this.spans = new CompactSpan[spanCount];
            for (int i = 0; i < spanCount; i++) {
                this.spans[i] = new CompactSpan();
            }
            
            this.dist = new int[spanCount];
            this.areas = new byte[spanCount];
            this.regs = new int[spanCount];
        }
        
        /**
         * 获取指定位置的单元格
         * @param x X坐标
         * @param z Z坐标
         * @return 单元格，如果坐标无效则返回null
         */
        public CompactCell getCell(int x, int z) {
            if (x < 0 || x >= width || z < 0 || z >= height) {
                return null;
            }
            return cells[z * width + x];
        }
        
        /**
         * 获取指定索引的span
         * @param index span索引
         * @return span，如果索引无效则返回null
         */
        public CompactSpan getSpan(int index) {
            if (index < 0 || index >= spanCount) {
                return null;
            }
            return spans[index];
        }
        
        /**
         * 设置边界
         * @param bmin 最小边界
         * @param bmax 最大边界
         * @param cs 单元格大小
         * @param ch 单元格高度
         */
        public void setBounds(float[] bmin, float[] bmax, float cs, float ch) {
            rcVcopy(this.bmin, bmin);
            rcVcopy(this.bmax, bmax);
            this.cs = cs;
            this.ch = ch;
        }
        
        /**
         * 获取指定位置的世界坐标
         * @param x 网格X坐标
         * @param z 网格Z坐标
         * @param worldPos 输出的世界坐标 [3]
         */
        public void getWorldPos(int x, int z, float[] worldPos) {
            worldPos[0] = bmin[0] + x * cs;
            worldPos[1] = bmin[1];
            worldPos[2] = bmin[2] + z * cs;
        }
        
        /**
         * 将世界坐标转换为网格坐标
         * @param worldPos 世界坐标 [3]
         * @param gridPos 输出的网格坐标 [2] (x, z)
         */
        public void worldToGrid(float[] worldPos, int[] gridPos) {
            gridPos[0] = (int) rcFloor((worldPos[0] - bmin[0]) / cs);
            gridPos[1] = (int) rcFloor((worldPos[2] - bmin[2]) / cs);
        }
        
        /**
         * 释放资源
         */
        public void dispose() {
            cells = null;
            spans = null;
            dist = null;
            areas = null;
            regs = null;
        }
    }
} 