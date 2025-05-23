package odin.detour.config;

/**
 * Detour常量定义
 * 翻译自UE5 DetourNavMesh.h中的各种常量
 * 
 * @author UE5NavMesh4J
 */
public final class DetourConstants {
    
    /** 导航多边形的最大顶点数 */
    public static final int DT_VERTS_PER_POLYGON = 6;
    
    /** 导航网格数据版本号 */
    public static final int DT_NAVMESH_VERSION = 7;
    
    /** 导航网格状态魔数 */
    public static final int DT_NAVMESH_STATE_MAGIC = ('D' << 24) | ('N' << 16) | ('M' << 8) | 'S';
    
    /** 导航网格状态版本号 */
    public static final int DT_NAVMESH_STATE_VERSION = 1;
    
    /** 表示实体链接到外部实体的标志 */
    public static final int DT_EXT_LINK = 0x8000;
    
    /** 表示实体不链接到任何东西的值 */
    public static final long DT_NULL_LINK = 0xffffffffL;
    
    /** 离网连接可以双向遍历的标志 */
    public static final int DT_OFFMESH_CON_BIDIR = 0x01;
    
    /** 离网连接点类型标志（UE5扩展） */
    public static final int DT_OFFMESH_CON_POINT = 0x02;
    
    /** 离网连接段类型标志（UE5扩展） */
    public static final int DT_OFFMESH_CON_SEGMENT = 0x04;
    
    /** 离网连接廉价区域标志（UE5扩展） */
    public static final int DT_OFFMESH_CON_CHEAPAREA = 0x08;
    
    /** 离网连接自动生成标志（UE5扩展） */
    public static final int DT_OFFMESH_CON_GENERATED = 0x10;
    
    /** 用户定义区域ID的最大数量 */
    public static final int DT_MAX_AREAS = 64;
    
    /** 导航网格瓦片的最小盐值位数（UE5扩展） */
    public static final int DT_MIN_SALT_BITS = 5;
    
    /** 盐值基数（UE5扩展） */
    public static final int DT_SALT_BASE = 1;
    
    /** 分辨率数量（UE5扩展） */
    public static final int DT_RESOLUTION_COUNT = 3;
    
    /** 最大离网连接段部分数（UE5扩展） */
    public static final int DT_MAX_OFFMESH_SEGMENT_PARTS = 4;
    
    /** 无效段部分（UE5扩展） */
    public static final int DT_INVALID_SEGMENT_PART = 0xffff;
    
    /** 连接标志 - 内部连接 */
    public static final int DT_CONNECTION_INTERNAL = (1 << 7);
    
    /** 链接标志 - 离网连接 */
    public static final int DT_LINK_FLAG_OFFMESH_CON = 0x01;
    
    /** 链接标志 - 离网连接双向 */
    public static final int DT_LINK_FLAG_OFFMESH_CON_BIDIR = 0x02;
    
    /** 链接标志 - 离网连接回溯器 */
    public static final int DT_LINK_FLAG_OFFMESH_CON_BACKTRACKER = 0x04;
    
    /** 链接标志 - 离网连接启用 */
    public static final int DT_LINK_FLAG_OFFMESH_CON_ENABLED = (1 << 3);
    
    /** 链接标志 - 侧面掩码 */
    public static final int DT_LINK_FLAG_SIDE_MASK = 7;
    
    /** 集群链接标志 - 前向有效 */
    public static final int DT_CLINK_VALID_FWD = 0x01;
    
    /** 集群链接标志 - 后向有效 */
    public static final int DT_CLINK_VALID_BCK = 0x02;
    
    /** 第一个集群链接索引 */
    public static final long DT_CLINK_FIRST = 0x80000000L;
    
    /** 不可行走多边形的成本 */
    public static final float DT_UNWALKABLE_POLY_COST = Float.MAX_VALUE;
    
    /** 私有构造函数防止实例化 */
    private DetourConstants() {
        throw new UnsupportedOperationException("常量类不能被实例化");
    }
} 