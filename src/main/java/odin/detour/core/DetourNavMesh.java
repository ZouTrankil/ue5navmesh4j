package odin.detour.core;

import java.util.List;
import java.util.ArrayList;
import static odin.detour.config.DetourConstants.*;
import static odin.detour.config.DetourEnums.*;

/**
 * Detour导航网格核心数据结构
 * 翻译自UE5 DetourNavMesh.h中的数据结构
 * 
 * @author UE5NavMesh4J
 */
public class DetourNavMesh {
    
    /**
     * 多边形引用类型
     */
    public static class PolyRef {
        public long value;
        
        public PolyRef() {
            this.value = 0;
        }
        
        public PolyRef(long value) {
            this.value = value;
        }
        
        public boolean isValid() {
            return value != 0;
        }
    }
    
    /**
     * 瓦片引用类型
     */
    public static class TileRef {
        public long value;
        
        public TileRef() {
            this.value = 0;
        }
        
        public TileRef(long value) {
            this.value = value;
        }
        
        public boolean isValid() {
            return value != 0;
        }
    }
    
    /**
     * 集群引用类型
     */
    public static class ClusterRef {
        public long value;
        
        public ClusterRef() {
            this.value = 0;
        }
        
        public ClusterRef(long value) {
            this.value = value;
        }
        
        public boolean isValid() {
            return value != 0;
        }
    }
    
    /**
     * 定义导航网格瓦片内的多边形
     */
    public static class Poly {
        /** 第一个链接在链表中的索引（如果没有链接则为DT_NULL_LINK） */
        public long firstLink;
        
        /** 多边形顶点的索引，实际顶点位于MeshTile::verts中 */
        public int[] verts = new int[DT_VERTS_PER_POLYGON];
        
        /** 打包数据，表示每个边的邻居多边形引用和标志 */
        public int[] neis = new int[DT_VERTS_PER_POLYGON];
        
        /** 多边形类型（区域） */
        public int area;
        
        /** 多边形标志 */
        public int flags;
        
        /** 多边形中的顶点数量 */
        public int vertCount;
        
        /** 位打包的区域id和多边形类型 */
        public int areaAndtype;
        
        /**
         * 默认构造函数
         */
        public Poly() {
            this.firstLink = DT_NULL_LINK;
            this.flags = 0;
            this.vertCount = 0;
            this.areaAndtype = 0;
        }
        
        /**
         * 设置用户定义的区域id
         * @param area 区域id [限制: < DT_MAX_AREAS]
         */
        public void setArea(int area) {
            areaAndtype = (areaAndtype & 0xc0) | (area & 0x3f);
        }
        
        /**
         * 设置多边形类型
         * @param type 多边形类型 (参见: PolyTypes)
         */
        public void setType(int type) {
            areaAndtype = (areaAndtype & 0x3f) | (type << 6);
        }
        
        /**
         * 获取用户定义的区域id
         * @return 区域id
         */
        public int getArea() {
            return areaAndtype & 0x3f;
        }
        
        /**
         * 获取多边形类型
         * @return 多边形类型 (参见: PolyTypes)
         */
        public int getType() {
            return areaAndtype >> 6;
        }
    }
    
    /**
     * 定义详细子网格数据在MeshTile中的位置
     */
    public static class PolyDetail {
        /** MeshTile::detailVerts数组中顶点的偏移 */
        public int vertBase;
        
        /** MeshTile::detailTris数组中三角形的偏移 */
        public int triBase;
        
        /** 子网格中的顶点数量 */
        public int vertCount;
        
        /** 子网格中的三角形数量 */
        public int triCount;
        
        /**
         * 默认构造函数
         */
        public PolyDetail() {
            this.vertBase = 0;
            this.triBase = 0;
            this.vertCount = 0;
            this.triCount = 0;
        }
    }
    
    /**
     * 定义多边形之间的链接
     */
    public static class Link {
        /** 邻居引用（链接到的邻居） */
        public PolyRef ref;
        
        /** 下一个链接的索引 */
        public long next;
        
        /** 拥有此链接的多边形边的索引 */
        public int edge;
        
        /** 如果是边界链接，定义链接在哪一边 */
        public int side;
        
        /** 如果是边界链接，定义最小子边区域 */
        public int bmin;
        
        /** 如果是边界链接，定义最大子边区域 */
        public int bmax;
        
        /**
         * 默认构造函数
         */
        public Link() {
            this.ref = new PolyRef();
            this.next = DT_NULL_LINK;
            this.edge = 0;
            this.side = 0;
            this.bmin = 0;
            this.bmax = 0;
        }
    }
    
    /**
     * 边界体积节点
     */
    public static class BVNode {
        /** 节点AABB的最小边界 [(x, y, z)] */
        public int[] bmin = new int[3];
        
        /** 节点AABB的最大边界 [(x, y, z)] */
        public int[] bmax = new int[3];
        
        /** 节点的索引（负数表示转义序列） */
        public int i;
        
        /**
         * 默认构造函数
         */
        public BVNode() {
            this.i = 0;
        }
    }
    
    /**
     * 定义导航网格离网连接
     */
    public static class OffMeshConnection {
        /** 连接的端点 [(ax, ay, az, bx, by, bz)] */
        public double[] pos = new double[6];
        
        /** 端点的半径 [限制: >= 0] */
        public double rad;
        
        /** 端点的捕捉高度（小于0 = 使用步高）（UE5扩展） */
        public double height;
        
        /** 离网连接的id（构建导航网格时用户分配） */
        public long userId;
        
        /** 瓦片内连接的多边形引用 */
        public int poly;
        
        /** 端点边 */
        public int side;
        
        /** 链接标志 */
        public int flags;
        
        /**
         * 默认构造函数
         */
        public OffMeshConnection() {
            this.rad = 0.0;
            this.height = 0.0;
            this.userId = 0;
            this.poly = 0;
            this.side = 0;
            this.flags = 0;
        }
        
        /**
         * 设置链接标志
         * @param conTypeFlags 连接类型标志
         */
        public void setFlags(int conTypeFlags) {
            flags = ((conTypeFlags & DT_OFFMESH_CON_BIDIR) != 0 ? 0x80 : 0) |
                   ((conTypeFlags & DT_OFFMESH_CON_CHEAPAREA) != 0 ? 0x40 : 0) |
                   ((conTypeFlags & DT_OFFMESH_CON_GENERATED) != 0 ? 0x20 : 0);
        }
        
        /**
         * 获取链接方向
         * @return true如果是双向链接
         */
        public boolean getBiDirectional() {
            return (flags & 0x80) != 0;
        }
        
        /**
         * 获取链接捕捉模式
         * @return true如果捕捉到最便宜的区域
         */
        public boolean getSnapToCheapestArea() {
            return (flags & 0x40) != 0;
        }
        
        /**
         * 指示链接是否自动生成
         * @return true如果链接是自动生成的
         */
        public boolean getIsGenerated() {
            return (flags & 0x20) != 0;
        }
    }
    
    /**
     * 段类型离网连接（UE5扩展）
     */
    public static class OffMeshSegmentConnection {
        /** 段A的起点 */
        public double[] startA = new double[3];
        
        /** 段A的终点 */
        public double[] endA = new double[3];
        
        /** 段B的起点 */
        public double[] startB = new double[3];
        
        /** 段B的终点 */
        public double[] endB = new double[3];
        
        /** 端点的半径 [限制: >= 0] */
        public double rad;
        
        /** 离网连接的id（构建导航网格时用户分配） */
        public long userId;
        
        /** 段池中的第一个多边形（+ header->offMeshSegPolyBase） */
        public int firstPoly;
        
        /** 创建的多边形数量 */
        public int npolys;
        
        /** 链接标志 */
        public int flags;
        
        /**
         * 默认构造函数
         */
        public OffMeshSegmentConnection() {
            this.rad = 0.0;
            this.userId = 0;
            this.firstPoly = 0;
            this.npolys = 0;
            this.flags = 0;
        }
        
        /**
         * 设置链接标志
         * @param conFlags 连接标志
         */
        public void setFlags(int conFlags) {
            flags = ((conFlags & DT_OFFMESH_CON_BIDIR) != 0 ? 0x80 : 0);
        }
        
        /**
         * 获取链接方向
         * @return true如果是双向链接
         */
        public boolean getBiDirectional() {
            return (flags & 0x80) != 0;
        }
    }
    
    /**
     * 多边形集群（UE5扩展）
     */
    public static class Cluster {
        /** 集群的中心位置 */
        public double[] center = new double[3];
        
        /** MeshTile.links数组中的链接 */
        public long firstLink;
        
        /** 集群链接数量 */
        public long numLinks;
        
        /**
         * 默认构造函数
         */
        public Cluster() {
            this.firstLink = DT_NULL_LINK;
            this.numLinks = 0;
        }
    }
    
    /**
     * 集群之间的链接（UE5扩展）
     */
    public static class ClusterLink {
        /** 目标瓦片和集群 */
        public ClusterRef ref;
        
        /** MeshTile.links数组中的下一个链接 */
        public long next;
        
        /** 链接遍历数据 */
        public int flags;
        
        /**
         * 默认构造函数
         */
        public ClusterLink() {
            this.ref = new ClusterRef();
            this.next = DT_NULL_LINK;
            this.flags = 0;
        }
    }
    
    /**
     * 提供与MeshTile对象相关的高级信息
     */
    public static class MeshHeader {
        /** 瓦片数据格式版本号 */
        public int version;
        
        /** NavMesh瓦片网格内瓦片的层 (x, y, layer) */
        public int layer;
        
        /** 瓦片中多边形的数量 */
        public int polyCount;
        
        /** 瓦片中顶点的数量 */
        public int vertCount;
        
        /** NavMesh瓦片网格内瓦片的x位置 (x, y, layer) */
        public int x;
        
        /** NavMesh瓦片网格内瓦片的y位置 (x, y, layer) */
        public int y;
        
        /** 分配的链接数量 */
        public int maxLinkCount;
        
        /** 详细网格中子网格的数量 */
        public int detailMeshCount;
        
        /** 详细网格中唯一顶点的数量（除了多边形顶点） */
        public int detailVertCount;
        
        /** 详细网格中三角形的数量 */
        public int detailTriCount;
        
        /** 边界体积节点的数量（如果禁用边界体积则为零） */
        public int bvNodeCount;
        
        /** 点类型离网连接的数量 */
        public int offMeshConCount;
        
        /** 第一个点类型离网连接多边形的索引 */
        public int offMeshBase;
        
        /** 段类型离网连接的数量（UE5扩展） */
        public int offMeshSegConCount;
        
        /** 第一个段类型离网连接多边形的索引（UE5扩展） */
        public int offMeshSegPolyBase;
        
        /** 段类型离网连接使用的第一个顶点的索引（UE5扩展） */
        public int offMeshSegVertBase;
        
        /** 集群数量（UE5扩展） */
        public int clusterCount;
        
        /** 瓦片使用的分辨率索引（UE5扩展） */
        public int resolution;
        
        /** 瓦片AABB的最小边界 [(x, y, z)] */
        public double[] bmin = new double[3];
        
        /** 瓦片AABB的最大边界 [(x, y, z)] */
        public double[] bmax = new double[3];
        
        /**
         * 默认构造函数
         */
        public MeshHeader() {
            this.version = DT_NAVMESH_VERSION;
            this.layer = 0;
            this.polyCount = 0;
            this.vertCount = 0;
            this.x = 0;
            this.y = 0;
            this.maxLinkCount = 0;
            this.detailMeshCount = 0;
            this.detailVertCount = 0;
            this.detailTriCount = 0;
            this.bvNodeCount = 0;
            this.offMeshConCount = 0;
            this.offMeshBase = 0;
            this.offMeshSegConCount = 0;
            this.offMeshSegPolyBase = 0;
            this.offMeshSegVertBase = 0;
            this.clusterCount = 0;
            this.resolution = 0;
        }
    }
    
    /**
     * 定义导航网格瓦片
     */
    public static class MeshTile {
        /** 描述瓦片修改的计数器 */
        public long salt;
        
        /** 指向下一个空闲链接的索引 */
        public long linksFreeList;
        
        /** 瓦片头 */
        public MeshHeader header;
        
        /** 瓦片多边形 [大小: MeshHeader::polyCount] */
        public Poly[] polys;
        
        /** 瓦片顶点 [大小: MeshHeader::vertCount] */
        public double[] verts;
        
        /** 瓦片链接 [大小: MeshHeader::maxLinkCount] */
        public Link[] links;
        
        /** 瓦片的详细子网格 [大小: MeshHeader::detailMeshCount] */
        public PolyDetail[] detailMeshes;
        
        /** 详细网格的唯一顶点 [(x, y, z) * MeshHeader::detailVertCount] */
        public double[] detailVerts;
        
        /** 详细网格的三角形 [(vertA, vertB, vertC) * MeshHeader::detailTriCount] */
        public byte[] detailTris;
        
        /** 瓦片边界体积节点 [大小: MeshHeader::bvNodeCount] */
        public BVNode[] bvTree;
        
        /** 瓦片离网连接 [大小: MeshHeader::offMeshConCount] */
        public OffMeshConnection[] offMeshCons;
        
        /** 瓦片段类型离网连接 [大小: MeshHeader::offMeshSegConCount] （UE5扩展） */
        public OffMeshSegmentConnection[] offMeshSeg;
        
        /** 集群数据（UE5扩展） */
        public Cluster[] clusters;
        
        /** 每个地面类型多边形的集群Id [大小: MeshHeader::polyCount] （UE5扩展） */
        public int[] polyClusters;
        
        /** 瓦片数据（正常情况下不直接访问） */
        public byte[] data;
        
        /** 瓦片数据的大小 */
        public int dataSize;
        
        /** 瓦片标志 (参见: TileFlags) */
        public int flags;
        
        /** 下一个空闲瓦片，或空间网格中的下一个瓦片 */
        public MeshTile next;
        
        /** 动态链接数组（UE5扩展） */
        public List<Link> dynamicLinksO;
        
        /** 下一个空闲动态链接的索引（UE5扩展） */
        public long dynamicFreeListO;
        
        /** 动态集群链接数组（UE5扩展） */
        public List<ClusterLink> dynamicLinksC;
        
        /** 下一个空闲动态集群链接的索引（UE5扩展） */
        public long dynamicFreeListC;
        
        /**
         * 默认构造函数
         */
        public MeshTile() {
            this.salt = 0;
            this.linksFreeList = DT_NULL_LINK;
            this.header = null;
            this.polys = null;
            this.verts = null;
            this.links = null;
            this.detailMeshes = null;
            this.detailVerts = null;
            this.detailTris = null;
            this.bvTree = null;
            this.offMeshCons = null;
            this.offMeshSeg = null;
            this.clusters = null;
            this.polyClusters = null;
            this.data = null;
            this.dataSize = 0;
            this.flags = 0;
            this.next = null;
            this.dynamicLinksO = new ArrayList<>();
            this.dynamicFreeListO = DT_NULL_LINK;
            this.dynamicLinksC = new ArrayList<>();
            this.dynamicFreeListC = DT_NULL_LINK;
        }
    }
    
    /**
     * 依赖于导航网格分辨率的配置参数（UE5扩展）
     */
    public static class NavMeshResParams {
        /** 边界体积量化因子 */
        public double bvQuantFactor;
        
        /**
         * 默认构造函数
         */
        public NavMeshResParams() {
            this.bvQuantFactor = 1.0;
        }
    }
    
    /**
     * 用于定义多瓦片导航网格的配置参数
     */
    public static class NavMeshParams {
        /** 使用瓦片的代理的高度（UE5扩展） */
        public double walkableHeight;
        
        /** 使用瓦片的代理的半径（UE5扩展） */
        public double walkableRadius;
        
        /** 使用瓦片的代理的最大攀爬高度（UE5扩展） */
        public double walkableClimb;
        
        /** 依赖于分辨率的参数（UE5扩展） */
        public NavMeshResParams[] resolutionParams = new NavMeshResParams[DT_RESOLUTION_COUNT];
        
        /** 导航网格瓦片空间的世界空间原点 [(x, y, z)] */
        public double[] orig = new double[3];
        
        /** 每个瓦片的宽度（沿x轴） */
        public double tileWidth;
        
        /** 每个瓦片的高度（沿z轴） */
        public double tileHeight;
        
        /** 导航网格可以包含的最大瓦片数 */
        public int maxTiles;
        
        /** 每个瓦片可以包含的最大多边形数 */
        public int maxPolys;
        
        /**
         * 默认构造函数
         */
        public NavMeshParams() {
            this.walkableHeight = 2.0;
            this.walkableRadius = 0.6;
            this.walkableClimb = 0.9;
            
            for (int i = 0; i < DT_RESOLUTION_COUNT; i++) {
                this.resolutionParams[i] = new NavMeshResParams();
            }
            
            this.tileWidth = 0.0;
            this.tileHeight = 0.0;
            this.maxTiles = 0;
            this.maxPolys = 0;
        }
    }
    
    /**
     * 获取指定索引的瓦片
     * @param index 瓦片索引
     * @return 瓦片实例，如果索引无效则返回null
     */
    public MeshTile getTile(int index) {
        if (index < 0 || index >= maxTiles) {
            return null;
        }
        return tiles[index];
    }
} 