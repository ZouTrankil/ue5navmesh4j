package odin.recast.config;

/**
 * Recast枚举定义
 * 翻译自UE5 Recast.h中的各种枚举
 * 
 * @author UE5NavMesh4J
 */
public final class RecastEnums {
    
    /**
     * Recast日志分类
     * @see rcContext
     */
    public enum LogCategory {
        /** 进度日志条目 */
        RC_LOG_PROGRESS(1),
        /** 警告日志条目 */
        RC_LOG_WARNING(2),
        /** 错误日志条目 */
        RC_LOG_ERROR(3);
        
        private final int value;
        
        LogCategory(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * Recast性能计时器分类
     * @see rcContext
     */
    public enum TimerLabel {
        /** 用户定义的构建总时间 */
        RC_TIMER_TOTAL,
        /** 用户定义的构建时间 */
        RC_TIMER_TEMP,
        /** 光栅化三角形的时间 */
        RC_TIMER_RASTERIZE_TRIANGLES,
        /** 构建紧凑高度字段的时间 */
        RC_TIMER_BUILD_COMPACTHEIGHTFIELD,
        /** 构建轮廓的时间 */
        RC_TIMER_BUILD_CONTOURS,
        /** 简化轮廓的时间 */
        RC_TIMER_BUILD_CONTOURS_TRACE,
        /** 简化轮廓的时间 */
        RC_TIMER_BUILD_CONTOURS_SIMPLIFY,
        /** 过滤边框的时间 */
        RC_TIMER_FILTER_BORDER,
        /** 过滤可行走区域的时间 */
        RC_TIMER_FILTER_WALKABLE,
        /** 过滤低障碍的时间 */
        RC_TIMER_FILTER_LOW_OBSTACLES,
        /** 标记可达区域的时间 */
        RC_TIMER_MEDIAN_AREA,
        /** 过滤低高度区域的时间 */
        RC_TIMER_FILTER_LOW_HEIGHT_SPANS,
        /** 构建多边形网格的时间 */
        RC_TIMER_BUILD_POLYMESH,
        /** 合并多边形网格的时间 */
        RC_TIMER_MERGE_POLYMESH,
        /** 构建详细网格的时间 */
        RC_TIMER_BUILD_POLYMESHDETAIL,
        /** 合并详细网格的时间 */
        RC_TIMER_MERGE_POLYMESHDETAIL,
        /** 构建距离字段的时间 */
        RC_TIMER_BUILD_DISTANCEFIELD,
        /** 构建距离字段距离的时间 */
        RC_TIMER_BUILD_DISTANCEFIELD_DIST,
        /** 构建距离字段模糊的时间 */
        RC_TIMER_BUILD_DISTANCEFIELD_BLUR,
        /** 构建区域的时间 */
        RC_TIMER_BUILD_REGIONS,
        /** 构建区域扩展的时间 */
        RC_TIMER_BUILD_REGIONS_WATERSHED,
        /** 构建区域扩展的时间 */
        RC_TIMER_BUILD_REGIONS_EXPAND,
        /** 构建区域洪填的时间 */
        RC_TIMER_BUILD_REGIONS_FLOOD,
        /** 构建区域过滤的时间 */
        RC_TIMER_BUILD_REGIONS_FILTER,
        /** 构建层的时间 */
        RC_TIMER_BUILD_LAYERS,
        /** 计时器的最大数量 */
        RC_MAX_TIMERS;
    }
    
    /**
     * 光栅化标志
     * 用于控制三角形光栅化行为
     */
    public enum RasterizationFlags {
        /** 无特殊标志 */
        RC_NONE(0),
        /** 将三角形投影到高度字段底部 */
        RC_PROJECT_TO_BOTTOM(1);
        
        private final int value;
        
        RasterizationFlags(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * 邻居坡度过滤模式
     * 用于控制边缘过滤的行为
     */
    public enum NeighborSlopeFilterMode {
        /** 不进行坡度过滤 */
        RC_SLOPE_FILTER_NONE,
        /** 使用原始Recast算法 */
        RC_SLOPE_FILTER_RECAST,
        /** 使用基于可行走坡度的高度过滤 */
        RC_SLOPE_FILTER_USE_HEIGHT_FROM_WALKABLE_SLOPE;
    }
    
    /**
     * 区域分割算法类型
     */
    public enum RegionPartitioning {
        /** 分水岭算法 */
        RC_REGION_PARTITION_WATERSHED,
        /** 单调分割 */
        RC_REGION_PARTITION_MONOTONE,
        /** 分层分割 */
        RC_REGION_PARTITION_LAYERS;
    }
    
    /**
     * 多边形网格类型
     */
    public enum PolygonType {
        /** 任意多边形 */
        RC_POLYGONS_ANY,
        /** 凸多边形 */
        RC_POLYGONS_CONVEX;
    }
    
    /**
     * 轮廓构建标志
     */
    public enum ContourBuildFlags {
        /** 构建时考虑瓦片边界 */
        RC_CONTOUR_TESS_WALL_EDGES(0x01),
        /** 构建时考虑区域边界 */
        RC_CONTOUR_TESS_AREA_EDGES(0x02),
        /** 构建时考虑所有边界 */
        RC_CONTOUR_TESS_TILE_EDGES(0x04);
        
        private final int value;
        
        ContourBuildFlags(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * 距离字段构建标志
     */
    public enum DistanceFieldFlags {
        /** 使用准确距离计算 */
        RC_DISTANCE_FIELD_ACCURATE(0x01),
        /** 使用快速距离计算 */
        RC_DISTANCE_FIELD_FAST(0x02);
        
        private final int value;
        
        DistanceFieldFlags(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * 区域合并标志
     */
    public enum RegionMergeFlags {
        /** 合并单调区域 */
        RC_MERGE_MONOTONE_REGIONS(0x01),
        /** 合并小区域 */
        RC_MERGE_SMALL_REGIONS(0x02);
        
        private final int value;
        
        RegionMergeFlags(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /** 私有构造函数防止实例化 */
    private RecastEnums() {
        throw new UnsupportedOperationException("枚举类不能被实例化");
    }
} 