package odin.recast.core;

import static odin.recast.config.RecastConstants.*;

/**
 * Recast Span相关数据结构
 * 翻译自UE5 Recast.h中的span相关结构体
 * 
 * @author UE5NavMesh4J
 */
public class RecastSpan {
    
    /**
     * 表示高度字段中span的数据
     * 翻译自rcSpanData结构体
     */
    public static class SpanData {
        /** span的下限 [限制: < smax] */
        public int smin;
        
        /** span的上限 [限制: <= RC_SPAN_MAX_HEIGHT] */
        public int smax;
        
        /** 分配给span的区域id */
        public int area;
        
        /**
         * 默认构造函数
         */
        public SpanData() {
            this.smin = 0;
            this.smax = 0;
            this.area = 0;
        }
        
        /**
         * 带参数的构造函数
         * @param smin span下限
         * @param smax span上限
         * @param area 区域id
         */
        public SpanData(int smin, int smax, int area) {
            this.smin = smin;
            this.smax = smax;
            this.area = area;
        }
        
        /**
         * 复制构造函数
         * @param other 要复制的SpanData
         */
        public SpanData(SpanData other) {
            this.smin = other.smin;
            this.smax = other.smax;
            this.area = other.area;
        }
        
        /**
         * 获取span的高度
         * @return span高度
         */
        public int getHeight() {
            return smax - smin;
        }
        
        /**
         * 检查是否有效
         * @return true如果span有效
         */
        public boolean isValid() {
            return smin <= smax && smax <= RC_SPAN_MAX_HEIGHT;
        }
    }
    
    /**
     * Span缓存结构
     * 翻译自rcSpanCache结构体
     */
    public static class SpanCache {
        /** X坐标 */
        public int x;
        
        /** Y坐标 */
        public int y;
        
        /** Span数据 */
        public SpanData data;
        
        /**
         * 默认构造函数
         */
        public SpanCache() {
            this.x = 0;
            this.y = 0;
            this.data = new SpanData();
        }
        
        /**
         * 带参数的构造函数
         * @param x X坐标
         * @param y Y坐标
         * @param data Span数据
         */
        public SpanCache(int x, int y, SpanData data) {
            this.x = x;
            this.y = y;
            this.data = new SpanData(data);
        }
    }
    
    /**
     * 表示高度字段中的一个span
     * 翻译自rcSpan结构体
     */
    public static class Span {
        /** Span数据 */
        public SpanData data;
        
        /** 列中更高的下一个span */
        public Span next;
        
        /**
         * 默认构造函数
         */
        public Span() {
            this.data = new SpanData();
            this.next = null;
        }
        
        /**
         * 带数据的构造函数
         * @param data Span数据
         */
        public Span(SpanData data) {
            this.data = new SpanData(data);
            this.next = null;
        }
        
        /**
         * 带参数的构造函数
         * @param smin span下限
         * @param smax span上限
         * @param area 区域id
         */
        public Span(int smin, int smax, int area) {
            this.data = new SpanData(smin, smax, area);
            this.next = null;
        }
        
        /**
         * 获取span高度
         * @return span高度
         */
        public int getHeight() {
            return data.getHeight();
        }
        
        /**
         * 检查是否与另一个span重叠
         * @param other 另一个span
         * @return true如果重叠
         */
        public boolean overlaps(Span other) {
            return !(data.smax <= other.data.smin || data.smin >= other.data.smax);
        }
        
        /**
         * 检查是否在指定高度范围内
         * @param minHeight 最小高度
         * @param maxHeight 最大高度
         * @return true如果在范围内
         */
        public boolean isInHeightRange(int minHeight, int maxHeight) {
            return data.smin >= minHeight && data.smax <= maxHeight;
        }
    }
    
    /**
     * 用于在高度字段中快速分配span的内存池
     * 翻译自rcSpanPool结构体
     */
    public static class SpanPool {
        /** 下一个span池 */
        public SpanPool next;
        
        /** 池中的span数组 */
        public Span[] items;
        
        /** 当前分配索引 */
        private int allocIndex;
        
        /**
         * 默认构造函数
         */
        public SpanPool() {
            this.next = null;
            this.items = new Span[RC_SPANS_PER_POOL];
            this.allocIndex = 0;
            
            // 初始化所有span
            for (int i = 0; i < RC_SPANS_PER_POOL; i++) {
                this.items[i] = new Span();
            }
        }
        
        /**
         * 从池中分配一个span
         * @return 分配的span，如果池已满则返回null
         */
        public Span allocSpan() {
            if (allocIndex < RC_SPANS_PER_POOL) {
                return items[allocIndex++];
            }
            return null;
        }
        
        /**
         * 检查池是否已满
         * @return true如果池已满
         */
        public boolean isFull() {
            return allocIndex >= RC_SPANS_PER_POOL;
        }
        
        /**
         * 获取剩余容量
         * @return 剩余span数量
         */
        public int getRemainingCapacity() {
            return RC_SPANS_PER_POOL - allocIndex;
        }
        
        /**
         * 重置池（清空所有分配）
         */
        public void reset() {
            allocIndex = 0;
            // 重新初始化所有span
            for (int i = 0; i < RC_SPANS_PER_POOL; i++) {
                items[i].data = new SpanData();
                items[i].next = null;
            }
        }
    }
    
    /**
     * UE5新光栅化器的扩展结构
     * 翻译自rcRowExt结构体
     */
    public static class RowExt {
        /** 最小列 */
        public int minCol;
        
        /** 最大列 */
        public int maxCol;
        
        /**
         * 默认构造函数
         */
        public RowExt() {
            this.minCol = Integer.MAX_VALUE;
            this.maxCol = Integer.MIN_VALUE;
        }
        
        /**
         * 带参数的构造函数
         * @param minCol 最小列
         * @param maxCol 最大列
         */
        public RowExt(int minCol, int maxCol) {
            this.minCol = minCol;
            this.maxCol = maxCol;
        }
        
        /**
         * 更新列范围
         * @param col 新的列索引
         */
        public void updateRange(int col) {
            minCol = Math.min(minCol, col);
            maxCol = Math.max(maxCol, col);
        }
        
        /**
         * 检查是否有效范围
         * @return true如果有有效范围
         */
        public boolean hasValidRange() {
            return minCol <= maxCol;
        }
    }
    
    /**
     * 边缘碰撞结构
     * 翻译自rcEdgeHit结构体
     */
    public static class EdgeHit {
        /** 碰撞数组 */
        public byte[] hits;
        
        /**
         * 默认构造函数
         */
        public EdgeHit() {
            this.hits = new byte[2];
        }
        
        /**
         * 设置碰撞值
         * @param index 索引 (0或1)
         * @param value 碰撞值
         */
        public void setHit(int index, byte value) {
            if (index >= 0 && index < hits.length) {
                hits[index] = value;
            }
        }
        
        /**
         * 获取碰撞值
         * @param index 索引 (0或1)
         * @return 碰撞值
         */
        public byte getHit(int index) {
            if (index >= 0 && index < hits.length) {
                return hits[index];
            }
            return 0;
        }
    }
    
    /**
     * 临时span结构
     * 翻译自rcTempSpan结构体
     */
    public static class TempSpan {
        /** span的下限和上限 */
        public int[] sminmax;
        
        /**
         * 默认构造函数
         */
        public TempSpan() {
            this.sminmax = new int[2];
        }
        
        /**
         * 带参数的构造函数
         * @param smin span下限
         * @param smax span上限
         */
        public TempSpan(int smin, int smax) {
            this.sminmax = new int[]{smin, smax};
        }
        
        /**
         * 获取span下限
         * @return span下限
         */
        public int getSMin() {
            return sminmax[0];
        }
        
        /**
         * 设置span下限
         * @param smin span下限
         */
        public void setSMin(int smin) {
            sminmax[0] = smin;
        }
        
        /**
         * 获取span上限
         * @return span上限
         */
        public int getSMax() {
            return sminmax[1];
        }
        
        /**
         * 设置span上限
         * @param smax span上限
         */
        public void setSMax(int smax) {
            sminmax[1] = smax;
        }
        
        /**
         * 获取span高度
         * @return span高度
         */
        public int getHeight() {
            return sminmax[1] - sminmax[0];
        }
    }
} 