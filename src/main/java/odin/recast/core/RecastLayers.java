package odin.recast.core;

import static odin.recast.utils.RecastMath.*;

/**
 * Recast高度字段层次相关数据结构
 * 翻译自UE5 Recast.h中的层次相关结构体
 * 
 * @author UE5NavMesh4J
 */
public class RecastLayers {
    
    /**
     * 表示层集合中的一个高度字段层
     * 翻译自rcHeightfieldLayer结构体
     */
    public static class HeightfieldLayer {
        /** 世界空间中的最小边界 [(x, y, z)] */
        public float[] bmin = new float[3];
        
        /** 世界空间中的最大边界 [(x, y, z)] */
        public float[] bmax = new float[3];
        
        /** 每个单元格的大小（在xz平面上） */
        public float cs;
        
        /** 每个单元格的高度（y轴的最小增量） */
        public float ch;
        
        /** 高度字段的宽度（沿x轴的单元格单位） */
        public int width;
        
        /** 高度字段的高度（沿z轴的单元格单位） */
        public int height;
        
        /** 可用数据的最小x边界 */
        public int minx;
        
        /** 可用数据的最大x边界 */
        public int maxx;
        
        /** 可用数据的最小y边界（沿z轴） */
        public int miny;
        
        /** 可用数据的最大y边界（沿z轴） */
        public int maxy;
        
        /** 可用数据的最小高度边界（沿y轴） */
        public int hmin;
        
        /** 可用数据的最大高度边界（沿y轴） */
        public int hmax;
        
        /** 高度字段 [大小: (width - borderSize*2) * (height - borderSize*2)] */
        public int[] heights;
        
        /** 区域id [大小: 与heights相同] */
        public byte[] areas;
        
        /** 打包的邻居连接信息 [大小: 与heights相同] */
        public byte[] cons;
        
        /**
         * 默认构造函数
         */
        public HeightfieldLayer() {
            this.cs = 0.0f;
            this.ch = 0.0f;
            this.width = 0;
            this.height = 0;
            this.minx = 0;
            this.maxx = 0;
            this.miny = 0;
            this.maxy = 0;
            this.hmin = 0;
            this.hmax = 0;
            this.heights = null;
            this.areas = null;
            this.cons = null;
        }
        
        /**
         * 初始化高度字段层
         * @param width 宽度
         * @param height 高度
         */
        public void init(int width, int height) {
            this.width = width;
            this.height = height;
            
            int size = width * height;
            if (size > 0) {
                this.heights = new int[size];
                this.areas = new byte[size];
                this.cons = new byte[size];
                
                // 初始化数组
                for (int i = 0; i < size; i++) {
                    this.heights[i] = 0;
                    this.areas[i] = 0;
                    this.cons[i] = 0;
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
         * 设置可用数据边界
         * @param minx 最小x
         * @param maxx 最大x
         * @param miny 最小y
         * @param maxy 最大y
         * @param hmin 最小高度
         * @param hmax 最大高度
         */
        public void setDataBounds(int minx, int maxx, int miny, int maxy, int hmin, int hmax) {
            this.minx = minx;
            this.maxx = maxx;
            this.miny = miny;
            this.maxy = maxy;
            this.hmin = hmin;
            this.hmax = hmax;
        }
        
        /**
         * 获取指定位置的高度
         * @param x X坐标
         * @param y Y坐标
         * @return 高度值，-1表示无效坐标
         */
        public int getHeight(int x, int y) {
            if (x < 0 || x >= width || y < 0 || y >= height || heights == null) {
                return -1;
            }
            return heights[y * width + x];
        }
        
        /**
         * 设置指定位置的高度
         * @param x X坐标
         * @param y Y坐标
         * @param height 高度值
         * @return true如果成功设置
         */
        public boolean setHeight(int x, int y, int height) {
            if (x < 0 || x >= width || y < 0 || y >= this.height || heights == null) {
                return false;
            }
            heights[y * width + x] = height;
            return true;
        }
        
        /**
         * 获取指定位置的区域id
         * @param x X坐标
         * @param y Y坐标
         * @return 区域id，-1表示无效坐标
         */
        public int getArea(int x, int y) {
            if (x < 0 || x >= width || y < 0 || y >= height || areas == null) {
                return -1;
            }
            return areas[y * width + x] & 0xFF;
        }
        
        /**
         * 设置指定位置的区域id
         * @param x X坐标
         * @param y Y坐标
         * @param area 区域id
         * @return true如果成功设置
         */
        public boolean setArea(int x, int y, byte area) {
            if (x < 0 || x >= width || y < 0 || y >= height || areas == null) {
                return false;
            }
            areas[y * width + x] = area;
            return true;
        }
        
        /**
         * 获取指定位置的连接信息
         * @param x X坐标
         * @param y Y坐标
         * @return 连接信息，-1表示无效坐标
         */
        public int getConnection(int x, int y) {
            if (x < 0 || x >= width || y < 0 || y >= height || cons == null) {
                return -1;
            }
            return cons[y * width + x] & 0xFF;
        }
        
        /**
         * 设置指定位置的连接信息
         * @param x X坐标
         * @param y Y坐标
         * @param con 连接信息
         * @return true如果成功设置
         */
        public boolean setConnection(int x, int y, byte con) {
            if (x < 0 || x >= width || y < 0 || y >= height || cons == null) {
                return false;
            }
            cons[y * width + x] = con;
            return true;
        }
        
        /**
         * 检查坐标是否在有效数据范围内
         * @param x X坐标
         * @param y Y坐标
         * @return true如果在有效范围内
         */
        public boolean isInValidDataBounds(int x, int y) {
            return x >= minx && x <= maxx && y >= miny && y <= maxy;
        }
        
        /**
         * 计算有效数据区域的大小
         * @return 有效数据区域的单元格数量
         */
        public int getValidDataSize() {
            if (maxx < minx || maxy < miny) {
                return 0;
            }
            return (maxx - minx + 1) * (maxy - miny + 1);
        }
        
        /**
         * 获取层的总单元格数量
         * @return 总单元格数量
         */
        public int getTotalSize() {
            return width * height;
        }
        
        /**
         * 检查高度字段层是否有效
         * @return true如果有效
         */
        public boolean isValid() {
            return width > 0 && height > 0 && heights != null && areas != null && cons != null;
        }
        
        /**
         * 释放资源
         */
        public void dispose() {
            heights = null;
            areas = null;
            cons = null;
            width = 0;
            height = 0;
        }
    }
    
    /**
     * 表示一组高度字段层
     * 翻译自rcHeightfieldLayerSet结构体
     */
    public static class HeightfieldLayerSet {
        /** 集合中的层 [大小: nlayers] */
        public HeightfieldLayer[] layers;
        
        /** 集合中层的数量 */
        public int nlayers;
        
        /**
         * 默认构造函数
         */
        public HeightfieldLayerSet() {
            this.layers = null;
            this.nlayers = 0;
        }
        
        /**
         * 初始化高度字段层集合
         * @param nlayers 层数量
         */
        public void init(int nlayers) {
            this.nlayers = nlayers;
            if (nlayers > 0) {
                this.layers = new HeightfieldLayer[nlayers];
                for (int i = 0; i < nlayers; i++) {
                    this.layers[i] = new HeightfieldLayer();
                }
            }
        }
        
        /**
         * 获取指定索引的层
         * @param index 层索引
         * @return 层，如果索引无效则返回null
         */
        public HeightfieldLayer getLayer(int index) {
            if (index < 0 || index >= nlayers || layers == null) {
                return null;
            }
            return layers[index];
        }
        
        /**
         * 设置指定索引的层
         * @param index 层索引
         * @param layer 层对象
         * @return true如果成功设置
         */
        public boolean setLayer(int index, HeightfieldLayer layer) {
            if (index < 0 || index >= nlayers || layers == null) {
                return false;
            }
            layers[index] = layer;
            return true;
        }
        
        /**
         * 计算所有层的边界框
         * @param bmin 输出的最小边界 [3]
         * @param bmax 输出的最大边界 [3]
         * @return true如果成功计算
         */
        public boolean calculateBounds(float[] bmin, float[] bmax) {
            if (layers == null || nlayers == 0 || bmin.length < 3 || bmax.length < 3) {
                return false;
            }
            
            bmin[0] = bmin[1] = bmin[2] = Float.MAX_VALUE;
            bmax[0] = bmax[1] = bmax[2] = Float.MIN_VALUE;
            
            boolean hasValidLayer = false;
            
            for (int i = 0; i < nlayers; i++) {
                HeightfieldLayer layer = layers[i];
                if (layer != null && layer.isValid()) {
                    bmin[0] = rcMin(bmin[0], layer.bmin[0]);
                    bmin[1] = rcMin(bmin[1], layer.bmin[1]);
                    bmin[2] = rcMin(bmin[2], layer.bmin[2]);
                    
                    bmax[0] = rcMax(bmax[0], layer.bmax[0]);
                    bmax[1] = rcMax(bmax[1], layer.bmax[1]);
                    bmax[2] = rcMax(bmax[2], layer.bmax[2]);
                    
                    hasValidLayer = true;
                }
            }
            
            return hasValidLayer;
        }
        
        /**
         * 获取指定区域id的层数量
         * @param areaId 区域id
         * @return 层数量
         */
        public int getLayerCountByArea(byte areaId) {
            int count = 0;
            if (layers != null) {
                for (int i = 0; i < nlayers; i++) {
                    HeightfieldLayer layer = layers[i];
                    if (layer != null && layer.isValid()) {
                        // 检查层中是否包含指定区域
                        boolean hasArea = false;
                        for (int j = 0; j < layer.getTotalSize(); j++) {
                            if (layer.areas[j] == areaId) {
                                hasArea = true;
                                break;
                            }
                        }
                        if (hasArea) {
                            count++;
                        }
                    }
                }
            }
            return count;
        }
        
        /**
         * 获取总的有效数据单元格数量
         * @return 总有效数据单元格数量
         */
        public int getTotalValidDataSize() {
            int total = 0;
            if (layers != null) {
                for (int i = 0; i < nlayers; i++) {
                    HeightfieldLayer layer = layers[i];
                    if (layer != null && layer.isValid()) {
                        total += layer.getValidDataSize();
                    }
                }
            }
            return total;
        }
        
        /**
         * 获取总的单元格数量
         * @return 总单元格数量
         */
        public int getTotalSize() {
            int total = 0;
            if (layers != null) {
                for (int i = 0; i < nlayers; i++) {
                    HeightfieldLayer layer = layers[i];
                    if (layer != null && layer.isValid()) {
                        total += layer.getTotalSize();
                    }
                }
            }
            return total;
        }
        
        /**
         * 查找包含指定位置的层
         * @param worldPos 世界坐标 [3]
         * @return 层索引，-1表示未找到
         */
        public int findLayerAt(float[] worldPos) {
            if (layers == null || nlayers == 0 || worldPos.length < 3) {
                return -1;
            }
            
            for (int i = 0; i < nlayers; i++) {
                HeightfieldLayer layer = layers[i];
                if (layer != null && layer.isValid()) {
                    // 检查位置是否在层的边界内
                    if (worldPos[0] >= layer.bmin[0] && worldPos[0] <= layer.bmax[0] &&
                        worldPos[1] >= layer.bmin[1] && worldPos[1] <= layer.bmax[1] &&
                        worldPos[2] >= layer.bmin[2] && worldPos[2] <= layer.bmax[2]) {
                        return i;
                    }
                }
            }
            
            return -1;
        }
        
        /**
         * 检查高度字段层集合是否有效
         * @return true如果有效
         */
        public boolean isValid() {
            return nlayers > 0 && layers != null;
        }
        
        /**
         * 释放资源
         */
        public void dispose() {
            if (layers != null) {
                for (int i = 0; i < nlayers; i++) {
                    if (layers[i] != null) {
                        layers[i].dispose();
                    }
                }
                layers = null;
            }
            nlayers = 0;
        }
    }
} 