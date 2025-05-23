package odin.recast.config;

/**
 * Recast构建配置类
 * 翻译自UE5 Recast.h中的rcConfig结构体
 * 指定执行Recast构建时使用的配置
 * 
 * @author UE5NavMesh4J
 */
public class RecastConfig {
    
    /** 沿x轴的字段宽度。[限制: >= 0] [单位: vx] */
    public int width;
    
    /** 沿z轴的字段高度。[限制: >= 0] [单位: vx] */
    public int height;
    
    /** xz平面上瓦片的宽度/高度大小。[限制: >= 0] [单位: vx] */
    public int tileSize;
    
    /** 高度字段周围不可导航边界的大小 */
    public BorderSize borderSize;
    
    /** 用于字段的xz平面单元格大小。[限制: > 0] [单位: wu] */
    public float cs;
    
    /** 用于字段的y轴单元格大小。[限制: > 0] [单位: wu] */
    public float ch;
    
    /** 字段AABB的最小边界。[(x, y, z)] [单位: wu] */
    public float[] bmin = new float[3];
    
    /** 字段AABB的最大边界。[(x, y, z)] [单位: wu] */
    public float[] bmax = new float[3];
    
    /** 被视为可行走的最大坡度。[限制: 0 <= value < 90] [单位: 度] */
    public float walkableSlopeAngle;
    
    /** 仍被视为可行走的地面到"天花板"的最小高度。[限制: >= 3] [单位: vx] */
    public int walkableHeight;
    
    /** 被视为仍可穿越的最大边缘高度。[限制: >=0] [单位: vx] */
    public int walkableClimb;
    
    /** 距离侵蚀/收缩高度字段可行走区域远离障碍物的距离。[限制: >=0] [单位: vx] */
    public int walkableRadius;
    
    /** 相对于cs和walkableSlopeAngle的最大步高 [限制: >=0] [单位: wu] */
    public float maxStepFromWalkableSlope;
    
    /** 网格边界上轮廓边缘允许的最大长度。[限制: >=0] [单位: vx] */
    public int maxEdgeLen;
    
    /** 简化轮廓的边界边缘应偏离原始原始轮廓的最大距离。[限制: >=0] [单位: wu] */
    public float maxSimplificationError;
    
    /** 简化轮廓时，与MaxSimplificationError比较时垂直误差的考虑程度。[限制: >=0] */
    public float simplificationElevationRatio;
    
    /** 允许形成孤立岛屿区域的最小单元格数。[限制: >=0] [单位: vx] */
    public int minRegionArea;
    
    /** 任何span计数小于此值的区域，如果可能，将与较大区域合并。[限制: >=0] [单位: vx] */
    public int mergeRegionArea;
    
    /** 区域块大小 [单位: vx] */
    public int regionChunkSize;
    
    /** 区域分割方法：创建多边形网格 */
    public int regionPartitioning;
    
    /** 轮廓到多边形转换过程中生成的多边形允许的最大顶点数。[限制: >= 3] */
    public int maxVertsPerPoly;
    
    /** 设置生成细节网格时使用的采样距离。（仅用于高度细节。）[限制: 0 或 >= 0.9] [单位: wu] */
    public float detailSampleDist;
    
    /** 细节网格表面应偏离高度字段数据的最大距离。（仅用于高度细节。）[限制: >=0] [单位: wu] */
    public float detailSampleMaxError;
    
    /**
     * 指定高度字段周围的边界大小
     * 翻译自UE5 Recast.h中的rcBorderSize结构体
     */
    public static class BorderSize {
        /** 轴负方向的边界大小 [限制: >= 0] [单位: vx] */
        public int low;
        
        /** 轴正方向的边界大小 [限制: >= 0] [单位: vx] */
        public int high;
        
        public BorderSize() {
            this.low = 0;
            this.high = 0;
        }
        
        public BorderSize(int low, int high) {
            this.low = low;
            this.high = high;
        }
    }
    
    /**
     * 默认构造函数
     */
    public RecastConfig() {
        // 初始化默认值
        this.width = 0;
        this.height = 0;
        this.tileSize = 0;
        this.borderSize = new BorderSize();
        this.cs = 0.3f;
        this.ch = 0.2f;
        this.walkableSlopeAngle = 45.0f;
        this.walkableHeight = 2;
        this.walkableClimb = 1;
        this.walkableRadius = 1;
        this.maxStepFromWalkableSlope = 0.0f;
        this.maxEdgeLen = 12;
        this.maxSimplificationError = 1.3f;
        this.simplificationElevationRatio = 0.0f;
        this.minRegionArea = 8;
        this.mergeRegionArea = 20;
        this.regionChunkSize = 64;
        this.regionPartitioning = RecastEnums.RegionPartitioning.RC_REGION_PARTITION_WATERSHED.ordinal();
        this.maxVertsPerPoly = 6;
        this.detailSampleDist = 6.0f;
        this.detailSampleMaxError = 1.0f;
    }
    
    /**
     * 复制构造函数
     * @param other 要复制的配置
     */
    public RecastConfig(RecastConfig other) {
        this.width = other.width;
        this.height = other.height;
        this.tileSize = other.tileSize;
        this.borderSize = new BorderSize(other.borderSize.low, other.borderSize.high);
        this.cs = other.cs;
        this.ch = other.ch;
        System.arraycopy(other.bmin, 0, this.bmin, 0, 3);
        System.arraycopy(other.bmax, 0, this.bmax, 0, 3);
        this.walkableSlopeAngle = other.walkableSlopeAngle;
        this.walkableHeight = other.walkableHeight;
        this.walkableClimb = other.walkableClimb;
        this.walkableRadius = other.walkableRadius;
        this.maxStepFromWalkableSlope = other.maxStepFromWalkableSlope;
        this.maxEdgeLen = other.maxEdgeLen;
        this.maxSimplificationError = other.maxSimplificationError;
        this.simplificationElevationRatio = other.simplificationElevationRatio;
        this.minRegionArea = other.minRegionArea;
        this.mergeRegionArea = other.mergeRegionArea;
        this.regionChunkSize = other.regionChunkSize;
        this.regionPartitioning = other.regionPartitioning;
        this.maxVertsPerPoly = other.maxVertsPerPoly;
        this.detailSampleDist = other.detailSampleDist;
        this.detailSampleMaxError = other.detailSampleMaxError;
    }
    
    /**
     * 设置边界框
     * @param minX 最小X坐标
     * @param minY 最小Y坐标
     * @param minZ 最小Z坐标
     * @param maxX 最大X坐标
     * @param maxY 最大Y坐标
     * @param maxZ 最大Z坐标
     */
    public void setBounds(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
        this.bmin[0] = minX;
        this.bmin[1] = minY;
        this.bmin[2] = minZ;
        this.bmax[0] = maxX;
        this.bmax[1] = maxY;
        this.bmax[2] = maxZ;
    }
    
    /**
     * 获取区域分割类型
     * @return 区域分割枚举值
     */
    public RecastEnums.RegionPartitioning getRegionPartitioning() {
        RecastEnums.RegionPartitioning[] values = RecastEnums.RegionPartitioning.values();
        if (regionPartitioning >= 0 && regionPartitioning < values.length) {
            return values[regionPartitioning];
        }
        return RecastEnums.RegionPartitioning.RC_REGION_PARTITION_WATERSHED;
    }
    
    /**
     * 设置区域分割类型
     * @param partitioning 区域分割枚举值
     */
    public void setRegionPartitioning(RecastEnums.RegionPartitioning partitioning) {
        this.regionPartitioning = partitioning.ordinal();
    }
} 