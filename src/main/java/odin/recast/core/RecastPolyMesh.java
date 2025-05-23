package odin.recast.core;

import odin.recast.config.RecastConfig;
import static odin.recast.utils.RecastMath.*;

/**
 * Recast多边形网格相关数据结构
 * 翻译自UE5 Recast.h中的多边形网格相关结构体
 * 
 * @author UE5NavMesh4J
 */
public class RecastPolyMesh {
    
    /**
     * 表示适用于构建导航网格的多边形网格
     * 翻译自rcPolyMesh结构体
     */
    public static class PolyMesh {
        /** 网格顶点 [形式: (x, y, z) * nverts] */
        public int[] verts;
        
        /** 多边形和邻居数据 [长度: maxpolys * 2 * nvp] */
        public int[] polys;
        
        /** 分配给每个多边形的区域id [长度: maxpolys] */
        public int[] regs;
        
        /** 用户定义的每个多边形标志 [长度: maxpolys] */
        public int[] flags;
        
        /** 分配给每个多边形的区域id [长度: maxpolys] */
        public byte[] areas;
        
        /** 顶点数量 */
        public int nverts;
        
        /** 多边形数量 */
        public int npolys;
        
        /** 分配的多边形数量 */
        public int maxpolys;
        
        /** 每个多边形的最大顶点数 */
        public int nvp;
        
        /** 世界空间中的最小边界 [(x, y, z)] */
        public float[] bmin = new float[3];
        
        /** 世界空间中的最大边界 [(x, y, z)] */
        public float[] bmax = new float[3];
        
        /** 每个单元格的大小（在xz平面上） */
        public float cs;
        
        /** 每个单元格的高度（y轴的最小增量） */
        public float ch;
        
        /** 用于生成网格的源数据的AABB边界大小 */
        public RecastConfig.BorderSize borderSize;
        
        /**
         * 默认构造函数
         */
        public PolyMesh() {
            this.verts = null;
            this.polys = null;
            this.regs = null;
            this.flags = null;
            this.areas = null;
            this.nverts = 0;
            this.npolys = 0;
            this.maxpolys = 0;
            this.nvp = 0;
            this.cs = 0.0f;
            this.ch = 0.0f;
            this.borderSize = new RecastConfig.BorderSize();
        }
        
        /**
         * 初始化多边形网格
         * @param maxpolys 最大多边形数量
         * @param nvp 每个多边形的最大顶点数
         * @param nverts 顶点数量
         */
        public void init(int maxpolys, int nvp, int nverts) {
            this.maxpolys = maxpolys;
            this.nvp = nvp;
            this.nverts = nverts;
            this.npolys = 0;
            
            if (nverts > 0) {
                this.verts = new int[nverts * 3];
            }
            
            if (maxpolys > 0) {
                this.polys = new int[maxpolys * nvp * 2];
                this.regs = new int[maxpolys];
                this.flags = new int[maxpolys];
                this.areas = new byte[maxpolys];
                
                // 初始化多边形数组
                for (int i = 0; i < maxpolys * nvp * 2; i++) {
                    this.polys[i] = 0xffff; // RC_MESH_NULL_IDX
                }
            }
        }
        
        /**
         * 获取指定索引的顶点
         * @param index 顶点索引
         * @param vertex 输出顶点坐标 [3]
         * @return true如果成功获取
         */
        public boolean getVertex(int index, int[] vertex) {
            if (index < 0 || index >= nverts || verts == null || vertex.length < 3) {
                return false;
            }
            
            int offset = index * 3;
            vertex[0] = verts[offset];     // x
            vertex[1] = verts[offset + 1]; // y
            vertex[2] = verts[offset + 2]; // z
            
            return true;
        }
        
        /**
         * 设置指定索引的顶点
         * @param index 顶点索引
         * @param vertex 顶点坐标 [3]
         * @return true如果成功设置
         */
        public boolean setVertex(int index, int[] vertex) {
            if (index < 0 || index >= nverts || verts == null || vertex.length < 3) {
                return false;
            }
            
            int offset = index * 3;
            verts[offset] = vertex[0];     // x
            verts[offset + 1] = vertex[1]; // y
            verts[offset + 2] = vertex[2]; // z
            
            return true;
        }
        
        /**
         * 获取指定索引的多边形
         * @param index 多边形索引
         * @param poly 输出多边形数据 [nvp * 2] (顶点索引 + 邻居索引)
         * @return true如果成功获取
         */
        public boolean getPolygon(int index, int[] poly) {
            if (index < 0 || index >= npolys || polys == null || poly.length < nvp * 2) {
                return false;
            }
            
            int offset = index * nvp * 2;
            System.arraycopy(polys, offset, poly, 0, nvp * 2);
            
            return true;
        }
        
        /**
         * 设置指定索引的多边形
         * @param index 多边形索引
         * @param poly 多边形数据 [nvp * 2] (顶点索引 + 邻居索引)
         * @return true如果成功设置
         */
        public boolean setPolygon(int index, int[] poly) {
            if (index < 0 || index >= maxpolys || polys == null || poly.length < nvp * 2) {
                return false;
            }
            
            int offset = index * nvp * 2;
            System.arraycopy(poly, 0, polys, offset, nvp * 2);
            
            // 更新多边形数量
            if (index >= npolys) {
                npolys = index + 1;
            }
            
            return true;
        }
        
        /**
         * 获取多边形的顶点数量
         * @param polyIndex 多边形索引
         * @return 顶点数量，-1表示错误
         */
        public int getPolyVertexCount(int polyIndex) {
            if (polyIndex < 0 || polyIndex >= npolys || polys == null) {
                return -1;
            }
            
            int offset = polyIndex * nvp * 2;
            int count = 0;
            
            for (int i = 0; i < nvp; i++) {
                if (polys[offset + i] != 0xffff) {
                    count++;
                } else {
                    break;
                }
            }
            
            return count;
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
         * 设置边界大小
         * @param borderSize 边界大小
         */
        public void setBorderSize(RecastConfig.BorderSize borderSize) {
            this.borderSize = new RecastConfig.BorderSize(borderSize.low, borderSize.high);
        }
        
        /**
         * 检查多边形网格是否有效
         * @return true如果有效
         */
        public boolean isValid() {
            return nverts > 0 && npolys > 0 && verts != null && polys != null;
        }
        
        /**
         * 释放资源
         */
        public void dispose() {
            verts = null;
            polys = null;
            regs = null;
            flags = null;
            areas = null;
            nverts = 0;
            npolys = 0;
            maxpolys = 0;
        }
    }
    
    /**
     * 包含表示与其关联多边形网格对象中多边形相关的详细高度数据的三角形网格
     * 翻译自rcPolyMeshDetail结构体
     */
    public static class PolyMeshDetail {
        /** 子网格数据 [大小: 4*nmeshes] */
        public int[] meshes;
        
        /** 网格顶点 [大小: 3*nverts] */
        public float[] verts;
        
        /** 网格三角形 [大小: 4*ntris] */
        public byte[] tris;
        
        /** 由meshes定义的子网格数量 */
        public int nmeshes;
        
        /** verts中的顶点数量 */
        public int nverts;
        
        /** tris中的三角形数量 */
        public int ntris;
        
        /**
         * 默认构造函数
         */
        public PolyMeshDetail() {
            this.meshes = null;
            this.verts = null;
            this.tris = null;
            this.nmeshes = 0;
            this.nverts = 0;
            this.ntris = 0;
        }
        
        /**
         * 初始化详细多边形网格
         * @param nmeshes 子网格数量
         * @param nverts 顶点数量
         * @param ntris 三角形数量
         */
        public void init(int nmeshes, int nverts, int ntris) {
            this.nmeshes = nmeshes;
            this.nverts = nverts;
            this.ntris = ntris;
            
            if (nmeshes > 0) {
                this.meshes = new int[nmeshes * 4];
            }
            
            if (nverts > 0) {
                this.verts = new float[nverts * 3];
            }
            
            if (ntris > 0) {
                this.tris = new byte[ntris * 4];
            }
        }
        
        /**
         * 获取指定索引的子网格信息
         * @param index 子网格索引
         * @param mesh 输出子网格数据 [4] (vertBase, triBase, vertCount, triCount)
         * @return true如果成功获取
         */
        public boolean getMesh(int index, int[] mesh) {
            if (index < 0 || index >= nmeshes || meshes == null || mesh.length < 4) {
                return false;
            }
            
            int offset = index * 4;
            mesh[0] = meshes[offset];     // vertBase
            mesh[1] = meshes[offset + 1]; // triBase
            mesh[2] = meshes[offset + 2]; // vertCount
            mesh[3] = meshes[offset + 3]; // triCount
            
            return true;
        }
        
        /**
         * 设置指定索引的子网格信息
         * @param index 子网格索引
         * @param mesh 子网格数据 [4] (vertBase, triBase, vertCount, triCount)
         * @return true如果成功设置
         */
        public boolean setMesh(int index, int[] mesh) {
            if (index < 0 || index >= nmeshes || meshes == null || mesh.length < 4) {
                return false;
            }
            
            int offset = index * 4;
            meshes[offset] = mesh[0];     // vertBase
            meshes[offset + 1] = mesh[1]; // triBase
            meshes[offset + 2] = mesh[2]; // vertCount
            meshes[offset + 3] = mesh[3]; // triCount
            
            return true;
        }
        
        /**
         * 获取指定索引的顶点
         * @param index 顶点索引
         * @param vertex 输出顶点坐标 [3]
         * @return true如果成功获取
         */
        public boolean getVertex(int index, float[] vertex) {
            if (index < 0 || index >= nverts || verts == null || vertex.length < 3) {
                return false;
            }
            
            int offset = index * 3;
            vertex[0] = verts[offset];     // x
            vertex[1] = verts[offset + 1]; // y
            vertex[2] = verts[offset + 2]; // z
            
            return true;
        }
        
        /**
         * 设置指定索引的顶点
         * @param index 顶点索引
         * @param vertex 顶点坐标 [3]
         * @return true如果成功设置
         */
        public boolean setVertex(int index, float[] vertex) {
            if (index < 0 || index >= nverts || verts == null || vertex.length < 3) {
                return false;
            }
            
            int offset = index * 3;
            verts[offset] = vertex[0];     // x
            verts[offset + 1] = vertex[1]; // y
            verts[offset + 2] = vertex[2]; // z
            
            return true;
        }
        
        /**
         * 获取指定索引的三角形
         * @param index 三角形索引
         * @param triangle 输出三角形数据 [4] (v0, v1, v2, flags)
         * @return true如果成功获取
         */
        public boolean getTriangle(int index, byte[] triangle) {
            if (index < 0 || index >= ntris || tris == null || triangle.length < 4) {
                return false;
            }
            
            int offset = index * 4;
            triangle[0] = tris[offset];     // v0
            triangle[1] = tris[offset + 1]; // v1
            triangle[2] = tris[offset + 2]; // v2
            triangle[3] = tris[offset + 3]; // flags
            
            return true;
        }
        
        /**
         * 设置指定索引的三角形
         * @param index 三角形索引
         * @param triangle 三角形数据 [4] (v0, v1, v2, flags)
         * @return true如果成功设置
         */
        public boolean setTriangle(int index, byte[] triangle) {
            if (index < 0 || index >= ntris || tris == null || triangle.length < 4) {
                return false;
            }
            
            int offset = index * 4;
            tris[offset] = triangle[0];     // v0
            tris[offset + 1] = triangle[1]; // v1
            tris[offset + 2] = triangle[2]; // v2
            tris[offset + 3] = triangle[3]; // flags
            
            return true;
        }
        
        /**
         * 计算指定子网格的边界框
         * @param meshIndex 子网格索引
         * @param bmin 输出的最小边界 [3]
         * @param bmax 输出的最大边界 [3]
         * @return true如果成功计算
         */
        public boolean getMeshBounds(int meshIndex, float[] bmin, float[] bmax) {
            if (meshIndex < 0 || meshIndex >= nmeshes || bmin.length < 3 || bmax.length < 3) {
                return false;
            }
            
            int[] mesh = new int[4];
            if (!getMesh(meshIndex, mesh)) {
                return false;
            }
            
            int vertBase = mesh[0];
            int vertCount = mesh[2];
            
            if (vertCount == 0) {
                return false;
            }
            
            bmin[0] = bmin[1] = bmin[2] = Float.MAX_VALUE;
            bmax[0] = bmax[1] = bmax[2] = Float.MIN_VALUE;
            
            for (int i = 0; i < vertCount; i++) {
                float[] vertex = new float[3];
                if (getVertex(vertBase + i, vertex)) {
                    bmin[0] = rcMin(bmin[0], vertex[0]);
                    bmin[1] = rcMin(bmin[1], vertex[1]);
                    bmin[2] = rcMin(bmin[2], vertex[2]);
                    
                    bmax[0] = rcMax(bmax[0], vertex[0]);
                    bmax[1] = rcMax(bmax[1], vertex[1]);
                    bmax[2] = rcMax(bmax[2], vertex[2]);
                }
            }
            
            return true;
        }
        
        /**
         * 检查详细多边形网格是否有效
         * @return true如果有效
         */
        public boolean isValid() {
            return nmeshes > 0 && nverts > 0 && ntris > 0 && 
                   meshes != null && verts != null && tris != null;
        }
        
        /**
         * 释放资源
         */
        public void dispose() {
            meshes = null;
            verts = null;
            tris = null;
            nmeshes = 0;
            nverts = 0;
            ntris = 0;
        }
    }
} 