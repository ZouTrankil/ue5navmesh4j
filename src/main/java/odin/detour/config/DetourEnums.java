package odin.detour.config;

/**
 * Detour枚举定义
 * 翻译自UE5 DetourNavMesh.h中的各种枚举
 * 
 * @author UE5NavMesh4J
 */
public final class DetourEnums {
    
    /**
     * 瓦片标志用于各种函数和字段
     */
    public enum TileFlags {
        /** 导航网格拥有瓦片内存并负责释放它 */
        DT_TILE_FREE_DATA(0x01);
        
        private final int value;
        
        TileFlags(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * 由dtNavMeshQuery::findStraightPath返回的顶点标志
     */
    public enum StraightPathFlags {
        /** 顶点是路径中的起始位置 */
        DT_STRAIGHTPATH_START(0x01),
        /** 顶点是路径中的结束位置 */
        DT_STRAIGHTPATH_END(0x02),
        /** 顶点是离网连接的起始点 */
        DT_STRAIGHTPATH_OFFMESH_CONNECTION(0x04);
        
        private final int value;
        
        StraightPathFlags(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * dtNavMeshQuery::findStraightPath的选项
     */
    public enum StraightPathOptions {
        /** 在区域变化的每个多边形边缘交叉处添加顶点 */
        DT_STRAIGHTPATH_AREA_CROSSINGS(0x01),
        /** 在每个多边形边缘交叉处添加顶点 */
        DT_STRAIGHTPATH_ALL_CROSSINGS(0x02);
        
        private final int value;
        
        StraightPathOptions(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * 表示导航网格多边形类型的标志
     */
    public enum PolyTypes {
        /** 多边形是作为网格表面一部分的标准凸多边形 */
        DT_POLYTYPE_GROUND(0),
        /** 多边形是由两个顶点组成的离网连接 */
        DT_POLYTYPE_OFFMESH_POINT(1),
        /** 多边形是由四个顶点组成的离网连接（UE5扩展） */
        DT_POLYTYPE_OFFMESH_SEGMENT(2);
        
        private final int value;
        
        PolyTypes(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * 旋转枚举（UE5扩展）
     */
    public enum Rotation {
        /** 不旋转 */
        DT_ROTATE_0,
        /** 旋转90度 */
        DT_ROTATE_90,
        /** 旋转180度 */
        DT_ROTATE_180,
        /** 旋转270度 */
        DT_ROTATE_270;
    }
    
    /**
     * 状态结果枚举
     */
    public enum Status {
        /** 操作失败 */
        DT_FAILURE(1 << 31),
        /** 操作成功 */
        DT_SUCCESS(1 << 30),
        /** 操作正在进行中 */
        DT_IN_PROGRESS(1 << 29),
        /** 输入数据不符合函数的要求 */
        DT_STATUS_DETAIL_MASK(0x0ffffff),
        /** 内存不足 */
        DT_WRONG_MAGIC(1 << 0),
        /** 数据版本错误 */
        DT_WRONG_VERSION(1 << 1),
        /** 内存不足 */
        DT_OUT_OF_MEMORY(1 << 2),
        /** 输入参数无效 */
        DT_INVALID_PARAM(1 << 3),
        /** 结果缓冲区太小 */
        DT_BUFFER_TOO_SMALL(1 << 4),
        /** 查询超出边界 */
        DT_OUT_OF_NODES(1 << 5),
        /** 查询结果部分 */
        DT_PARTIAL_RESULT(1 << 6);
        
        private final int value;
        
        Status(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
        
        /** 检查状态是否表示失败 */
        public static boolean dtStatusFailed(int status) {
            return (status & DT_FAILURE.value) != 0;
        }
        
        /** 检查状态是否表示成功 */
        public static boolean dtStatusSucceed(int status) {
            return (status & DT_SUCCESS.value) != 0;
        }
        
        /** 检查状态是否表示正在进行 */
        public static boolean dtStatusInProgress(int status) {
            return (status & DT_IN_PROGRESS.value) != 0;
        }
        
        /** 检查状态是否包含详细信息 */
        public static boolean dtStatusDetail(int status, int detail) {
            return (status & detail) != 0;
        }
    }
    
    /** 私有构造函数防止实例化 */
    private DetourEnums() {
        throw new UnsupportedOperationException("枚举类不能被实例化");
    }
} 