package odin.recast.config;

/**
 * Recast核心常量定义
 * 翻译自UE5 Recast.h
 * 
 * @author UE5NavMesh4J
 */
public final class RecastConstants {
    
    /** PI常量 */
    public static final double RC_PI = 3.14159265358979323846;
    
    /** 定义分配给rcSpanData::smin和rcSpanData::smax的位数 */
    public static final int RC_SPAN_HEIGHT_BITS = 29;
    
    /** rcSpanData::smin和rcSpanData::smax的最大值 */
    public static final int RC_SPAN_MAX_HEIGHT = (1 << RC_SPAN_HEIGHT_BITS) - 1;
    
    /** 每个span池分配的span数量 */
    public static final int RC_SPANS_PER_POOL = 2048;
    
    /** 空值标识 */
    public static final int RC_NULL_AREA = 0;
    public static final int RC_WALKABLE_AREA = 63;
    public static final int RC_NOT_CONNECTED = 0x3f;
    
    /** 方向常量 */
    public static final int RC_DIR_NONE = -1;
    public static final int RC_DIR_OFFSET_X = 0;
    public static final int RC_DIR_OFFSET_Y = 1;
    
    /** 边界值 */
    public static final int RC_BORDER_REG = 0x8000;
    public static final int RC_AREA_BORDER = 0x20;
    
    /** 网格常量 */
    public static final int RC_MULTIPLE_REGS = (RC_BORDER_REG | RC_AREA_BORDER);
    
    /** 高度字段边界标志 */
    public static final int RC_BORDER_VERTEX = 0x10000;
    
    /** 轮廓区域id掩码 */
    public static final int RC_CONTOUR_REG_MASK = 0xffff;
    
    /** 网格中的无效索引值 */
    public static final int RC_MESH_NULL_IDX = 0xffff;
    
    /** Detour相关常量 */
    public static final int RC_NOT_CONNECTED_DETOUR = 0xff;
    
    /** 私有构造函数防止实例化 */
    private RecastConstants() {
        throw new UnsupportedOperationException("常量类不能被实例化");
    }
} 