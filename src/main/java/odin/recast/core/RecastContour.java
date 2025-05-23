package odin.recast.core;

import odin.recast.config.RecastConfig;
import static odin.recast.utils.RecastMath.*;

/**
 * Recast轮廓相关数据结构
 * 翻译自UE5 Recast.h中的轮廓相关结构体
 * 
 * @author UE5NavMesh4J
 */
public class RecastContour {
    
    /**
     * 表示字段空间中简单的、不重叠的轮廓
     * 翻译自rcContour结构体
     */
    public static class Contour {
        /** 简化轮廓顶点和连接数据 [大小: 4 * nverts] */
        public int[] verts;
        
        /** 简化轮廓中的顶点数量 */
        public int nverts;
        
        /** 原始轮廓顶点和连接数据 [大小: 4 * nrverts] */
        public int[] rverts;
        
        /** 原始轮廓中的顶点数量 */
        public int nrverts;
        
        /** 轮廓的区域id */
        public int reg;
        
        /** 轮廓的区域id */
        public int area;
        
        /**
         * 默认构造函数
         */
        public Contour() {
            this.verts = null;
            this.nverts = 0;
            this.rverts = null;
            this.nrverts = 0;
            this.reg = 0;
            this.area = 0;
        }
        
        /**
         * 初始化轮廓
         * @param nverts 简化顶点数量
         * @param nrverts 原始顶点数量
         */
        public void init(int nverts, int nrverts) {
            this.nverts = nverts;
            this.nrverts = nrverts;
            
            if (nverts > 0) {
                this.verts = new int[nverts * 4];
            }
            
            if (nrverts > 0) {
                this.rverts = new int[nrverts * 4];
            }
        }
        
        /**
         * 获取指定索引的简化顶点
         * @param index 顶点索引
         * @param vertex 输出顶点数据 [4] (x, y, z, regionId)
         * @return true如果成功获取
         */
        public boolean getVertex(int index, int[] vertex) {
            if (index < 0 || index >= nverts || verts == null || vertex.length < 4) {
                return false;
            }
            
            int offset = index * 4;
            vertex[0] = verts[offset];     // x
            vertex[1] = verts[offset + 1]; // y
            vertex[2] = verts[offset + 2]; // z
            vertex[3] = verts[offset + 3]; // regionId/connection
            
            return true;
        }
        
        /**
         * 设置指定索引的简化顶点
         * @param index 顶点索引
         * @param vertex 顶点数据 [4] (x, y, z, regionId)
         * @return true如果成功设置
         */
        public boolean setVertex(int index, int[] vertex) {
            if (index < 0 || index >= nverts || verts == null || vertex.length < 4) {
                return false;
            }
            
            int offset = index * 4;
            verts[offset] = vertex[0];     // x
            verts[offset + 1] = vertex[1]; // y
            verts[offset + 2] = vertex[2]; // z
            verts[offset + 3] = vertex[3]; // regionId/connection
            
            return true;
        }
        
        /**
         * 获取指定索引的原始顶点
         * @param index 顶点索引
         * @param vertex 输出顶点数据 [4] (x, y, z, regionId)
         * @return true如果成功获取
         */
        public boolean getRawVertex(int index, int[] vertex) {
            if (index < 0 || index >= nrverts || rverts == null || vertex.length < 4) {
                return false;
            }
            
            int offset = index * 4;
            vertex[0] = rverts[offset];     // x
            vertex[1] = rverts[offset + 1]; // y
            vertex[2] = rverts[offset + 2]; // z
            vertex[3] = rverts[offset + 3]; // regionId/connection
            
            return true;
        }
        
        /**
         * 设置指定索引的原始顶点
         * @param index 顶点索引
         * @param vertex 顶点数据 [4] (x, y, z, regionId)
         * @return true如果成功设置
         */
        public boolean setRawVertex(int index, int[] vertex) {
            if (index < 0 || index >= nrverts || rverts == null || vertex.length < 4) {
                return false;
            }
            
            int offset = index * 4;
            rverts[offset] = vertex[0];     // x
            rverts[offset + 1] = vertex[1]; // y
            rverts[offset + 2] = vertex[2]; // z
            rverts[offset + 3] = vertex[3]; // regionId/connection
            
            return true;
        }
        
        /**
         * 检查轮廓是否有效
         * @return true如果轮廓有效
         */
        public boolean isValid() {
            return nverts > 0 && verts != null && verts.length >= nverts * 4;
        }
        
        /**
         * 获取轮廓的边界框
         * @param bmin 最小边界 [3]
         * @param bmax 最大边界 [3]
         * @return true如果成功计算边界框
         */
        public boolean getBounds(int[] bmin, int[] bmax) {
            if (!isValid() || bmin.length < 3 || bmax.length < 3) {
                return false;
            }
            
            bmin[0] = bmin[1] = bmin[2] = Integer.MAX_VALUE;
            bmax[0] = bmax[1] = bmax[2] = Integer.MIN_VALUE;
            
            for (int i = 0; i < nverts; i++) {
                int offset = i * 4;
                int x = verts[offset];
                int y = verts[offset + 1];
                int z = verts[offset + 2];
                
                bmin[0] = Math.min(bmin[0], x);
                bmin[1] = Math.min(bmin[1], y);
                bmin[2] = Math.min(bmin[2], z);
                
                bmax[0] = Math.max(bmax[0], x);
                bmax[1] = Math.max(bmax[1], y);
                bmax[2] = Math.max(bmax[2], z);
            }
            
            return true;
        }
        
        /**
         * 释放资源
         */
        public void dispose() {
            verts = null;
            rverts = null;
            nverts = 0;
            nrverts = 0;
        }
    }
    
    /**
     * 表示一组相关轮廓
     * 翻译自rcContourSet结构体
     */
    public static class ContourSet {
        /** 集合中的轮廓数组 [大小: nconts] */
        public Contour[] conts;
        
        /** 集合中轮廓的数量 */
        public int nconts;
        
        /** 世界空间中的最小边界 [(x, y, z)] */
        public float[] bmin = new float[3];
        
        /** 世界空间中的最大边界 [(x, y, z)] */
        public float[] bmax = new float[3];
        
        /** 每个单元格的大小（在xz平面上） */
        public float cs;
        
        /** 每个单元格的高度（y轴的最小增量） */
        public float ch;
        
        /** 集合的宽度（沿x轴的单元格单位） */
        public int width;
        
        /** 集合的高度（沿z轴的单元格单位） */
        public int height;
        
        /** 用于生成轮廓来源数据的AABB边界大小 */
        public RecastConfig.BorderSize borderSize;
        
        /**
         * 默认构造函数
         */
        public ContourSet() {
            this.conts = null;
            this.nconts = 0;
            this.cs = 0.0f;
            this.ch = 0.0f;
            this.width = 0;
            this.height = 0;
            this.borderSize = new RecastConfig.BorderSize();
        }
        
        /**
         * 初始化轮廓集合
         * @param nconts 轮廓数量
         */
        public void init(int nconts) {
            this.nconts = nconts;
            if (nconts > 0) {
                this.conts = new Contour[nconts];
                for (int i = 0; i < nconts; i++) {
                    this.conts[i] = new Contour();
                }
            }
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
         * 设置网格尺寸
         * @param width 网格宽度
         * @param height 网格高度
         * @param borderSize 边界大小
         */
        public void setGridSize(int width, int height, RecastConfig.BorderSize borderSize) {
            this.width = width;
            this.height = height;
            this.borderSize = new RecastConfig.BorderSize(borderSize.low, borderSize.high);
        }
        
        /**
         * 获取指定索引的轮廓
         * @param index 轮廓索引
         * @return 轮廓，如果索引无效则返回null
         */
        public Contour getContour(int index) {
            if (index < 0 || index >= nconts || conts == null) {
                return null;
            }
            return conts[index];
        }
        
        /**
         * 计算集合中所有轮廓的边界框
         * @param bmin 输出的最小边界 [3]
         * @param bmax 输出的最大边界 [3]
         * @return true如果成功计算
         */
        public boolean calculateBounds(float[] bmin, float[] bmax) {
            if (conts == null || nconts == 0 || bmin.length < 3 || bmax.length < 3) {
                return false;
            }
            
            bmin[0] = bmin[1] = bmin[2] = Float.MAX_VALUE;
            bmax[0] = bmax[1] = bmax[2] = Float.MIN_VALUE;
            
            for (int i = 0; i < nconts; i++) {
                Contour cont = conts[i];
                if (cont == null || !cont.isValid()) {
                    continue;
                }
                
                int[] contBmin = new int[3];
                int[] contBmax = new int[3];
                
                if (cont.getBounds(contBmin, contBmax)) {
                    bmin[0] = rcMin(bmin[0], (float)contBmin[0]);
                    bmin[1] = rcMin(bmin[1], (float)contBmin[1]);
                    bmin[2] = rcMin(bmin[2], (float)contBmin[2]);
                    
                    bmax[0] = rcMax(bmax[0], (float)contBmax[0]);
                    bmax[1] = rcMax(bmax[1], (float)contBmax[1]);
                    bmax[2] = rcMax(bmax[2], (float)contBmax[2]);
                }
            }
            
            return true;
        }
        
        /**
         * 获取指定区域的轮廓数量
         * @param regionId 区域id
         * @return 轮廓数量
         */
        public int getContourCountByRegion(int regionId) {
            int count = 0;
            if (conts != null) {
                for (int i = 0; i < nconts; i++) {
                    if (conts[i] != null && conts[i].reg == regionId) {
                        count++;
                    }
                }
            }
            return count;
        }
        
        /**
         * 检查轮廓集合是否有效
         * @return true如果有效
         */
        public boolean isValid() {
            return nconts > 0 && conts != null;
        }
        
        /**
         * 释放资源
         */
        public void dispose() {
            if (conts != null) {
                for (int i = 0; i < nconts; i++) {
                    if (conts[i] != null) {
                        conts[i].dispose();
                    }
                }
                conts = null;
            }
            nconts = 0;
        }
    }
} 