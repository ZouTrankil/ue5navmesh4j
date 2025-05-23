package odin.recast.core;

import static odin.recast.utils.RecastMath.*;

/**
 * Recast簇相关数据结构
 * 翻译自UE5 Recast.h中的簇相关结构体
 * 
 * @author UE5NavMesh4J
 */
public class RecastCluster {
    
    /**
     * 表示一组簇
     * 翻译自rcClusterSet结构体
     * 注意：此结构体在UE5中由WITH_NAVMESH_CLUSTER_LINKS宏控制
     */
    public static class ClusterSet {
        /** 簇的数量 */
        public int nclusters;
        
        /** 每个簇的中心点 [大小: 3 * nclusters] */
        public float[] center;
        
        /** 每个簇的链接数量 [大小: nclusters] */
        public int[] nlinks;
        
        /** 邻居id [大小: nlinks的总和] */
        public int[] links;
        
        /**
         * 默认构造函数
         */
        public ClusterSet() {
            this.nclusters = 0;
            this.center = null;
            this.nlinks = null;
            this.links = null;
        }
        
        /**
         * 初始化簇集合
         * @param nclusters 簇数量
         * @param totalLinks 总链接数量
         */
        public void init(int nclusters, int totalLinks) {
            this.nclusters = nclusters;
            
            if (nclusters > 0) {
                this.center = new float[nclusters * 3];
                this.nlinks = new int[nclusters];
                
                // 初始化中心点和链接数量
                for (int i = 0; i < nclusters * 3; i++) {
                    this.center[i] = 0.0f;
                }
                
                for (int i = 0; i < nclusters; i++) {
                    this.nlinks[i] = 0;
                }
            }
            
            if (totalLinks > 0) {
                this.links = new int[totalLinks];
                // 初始化链接数组
                for (int i = 0; i < totalLinks; i++) {
                    this.links[i] = -1; // 无效链接
                }
            }
        }
        
        /**
         * 获取指定簇的中心点
         * @param clusterIndex 簇索引
         * @param centerOut 输出的中心点 [3]
         * @return true如果成功获取
         */
        public boolean getClusterCenter(int clusterIndex, float[] centerOut) {
            if (clusterIndex < 0 || clusterIndex >= nclusters || center == null || centerOut.length < 3) {
                return false;
            }
            
            int offset = clusterIndex * 3;
            centerOut[0] = center[offset];     // x
            centerOut[1] = center[offset + 1]; // y
            centerOut[2] = center[offset + 2]; // z
            
            return true;
        }
        
        /**
         * 设置指定簇的中心点
         * @param clusterIndex 簇索引
         * @param centerPos 中心点坐标 [3]
         * @return true如果成功设置
         */
        public boolean setClusterCenter(int clusterIndex, float[] centerPos) {
            if (clusterIndex < 0 || clusterIndex >= nclusters || center == null || centerPos.length < 3) {
                return false;
            }
            
            int offset = clusterIndex * 3;
            center[offset] = centerPos[0];     // x
            center[offset + 1] = centerPos[1]; // y
            center[offset + 2] = centerPos[2]; // z
            
            return true;
        }
        
        /**
         * 获取指定簇的链接数量
         * @param clusterIndex 簇索引
         * @return 链接数量，-1表示错误
         */
        public int getClusterLinkCount(int clusterIndex) {
            if (clusterIndex < 0 || clusterIndex >= nclusters || nlinks == null) {
                return -1;
            }
            return nlinks[clusterIndex];
        }
        
        /**
         * 设置指定簇的链接数量
         * @param clusterIndex 簇索引
         * @param linkCount 链接数量
         * @return true如果成功设置
         */
        public boolean setClusterLinkCount(int clusterIndex, int linkCount) {
            if (clusterIndex < 0 || clusterIndex >= nclusters || nlinks == null || linkCount < 0) {
                return false;
            }
            nlinks[clusterIndex] = linkCount;
            return true;
        }
        
        /**
         * 获取指定簇的链接起始索引
         * @param clusterIndex 簇索引
         * @return 链接起始索引，-1表示错误
         */
        public int getClusterLinkStartIndex(int clusterIndex) {
            if (clusterIndex < 0 || clusterIndex >= nclusters || nlinks == null) {
                return -1;
            }
            
            int startIndex = 0;
            for (int i = 0; i < clusterIndex; i++) {
                startIndex += nlinks[i];
            }
            
            return startIndex;
        }
        
        /**
         * 获取指定簇的指定链接
         * @param clusterIndex 簇索引
         * @param linkIndex 链接索引（相对于簇）
         * @return 邻居簇id，-1表示错误
         */
        public int getClusterLink(int clusterIndex, int linkIndex) {
            if (clusterIndex < 0 || clusterIndex >= nclusters || 
                linkIndex < 0 || linkIndex >= getClusterLinkCount(clusterIndex) || links == null) {
                return -1;
            }
            
            int startIndex = getClusterLinkStartIndex(clusterIndex);
            if (startIndex == -1) {
                return -1;
            }
            
            return links[startIndex + linkIndex];
        }
        
        /**
         * 设置指定簇的指定链接
         * @param clusterIndex 簇索引
         * @param linkIndex 链接索引（相对于簇）
         * @param neighborId 邻居簇id
         * @return true如果成功设置
         */
        public boolean setClusterLink(int clusterIndex, int linkIndex, int neighborId) {
            if (clusterIndex < 0 || clusterIndex >= nclusters || 
                linkIndex < 0 || linkIndex >= getClusterLinkCount(clusterIndex) || links == null) {
                return false;
            }
            
            int startIndex = getClusterLinkStartIndex(clusterIndex);
            if (startIndex == -1) {
                return false;
            }
            
            links[startIndex + linkIndex] = neighborId;
            return true;
        }
        
        /**
         * 获取指定簇的所有链接
         * @param clusterIndex 簇索引
         * @param linksOut 输出的链接数组
         * @return 实际获取的链接数量，-1表示错误
         */
        public int getClusterLinks(int clusterIndex, int[] linksOut) {
            int linkCount = getClusterLinkCount(clusterIndex);
            if (linkCount <= 0 || linksOut.length < linkCount) {
                return -1;
            }
            
            int startIndex = getClusterLinkStartIndex(clusterIndex);
            if (startIndex == -1) {
                return -1;
            }
            
            System.arraycopy(links, startIndex, linksOut, 0, linkCount);
            return linkCount;
        }
        
        /**
         * 设置指定簇的所有链接
         * @param clusterIndex 簇索引
         * @param clusterLinks 链接数组
         * @param linkCount 链接数量
         * @return true如果成功设置
         */
        public boolean setClusterLinks(int clusterIndex, int[] clusterLinks, int linkCount) {
            if (clusterIndex < 0 || clusterIndex >= nclusters || 
                clusterLinks.length < linkCount || linkCount < 0) {
                return false;
            }
            
            // 先设置链接数量
            if (!setClusterLinkCount(clusterIndex, linkCount)) {
                return false;
            }
            
            int startIndex = getClusterLinkStartIndex(clusterIndex);
            if (startIndex == -1) {
                return false;
            }
            
            System.arraycopy(clusterLinks, 0, links, startIndex, linkCount);
            return true;
        }
        
        /**
         * 计算两个簇中心之间的距离
         * @param cluster1 簇1索引
         * @param cluster2 簇2索引
         * @return 距离，-1表示错误
         */
        public float getClusterDistance(int cluster1, int cluster2) {
            float[] center1 = new float[3];
            float[] center2 = new float[3];
            
            if (!getClusterCenter(cluster1, center1) || !getClusterCenter(cluster2, center2)) {
                return -1.0f;
            }
            
            return rcVdist(center1, center2);
        }
        
        /**
         * 查找距离指定位置最近的簇
         * @param pos 位置 [3]
         * @return 最近簇的索引，-1表示错误
         */
        public int findNearestCluster(float[] pos) {
            if (pos.length < 3 || nclusters == 0 || center == null) {
                return -1;
            }
            
            int nearestCluster = -1;
            float minDistance = Float.MAX_VALUE;
            
            for (int i = 0; i < nclusters; i++) {
                float[] clusterCenter = new float[3];
                if (getClusterCenter(i, clusterCenter)) {
                    float distance = rcVdist(pos, clusterCenter);
                    if (distance < minDistance) {
                        minDistance = distance;
                        nearestCluster = i;
                    }
                }
            }
            
            return nearestCluster;
        }
        
        /**
         * 检查两个簇是否相邻（有直接链接）
         * @param cluster1 簇1索引
         * @param cluster2 簇2索引
         * @return true如果相邻
         */
        public boolean areAdjacent(int cluster1, int cluster2) {
            int linkCount = getClusterLinkCount(cluster1);
            if (linkCount <= 0) {
                return false;
            }
            
            for (int i = 0; i < linkCount; i++) {
                if (getClusterLink(cluster1, i) == cluster2) {
                    return true;
                }
            }
            
            return false;
        }
        
        /**
         * 计算集合中所有簇的边界框
         * @param bmin 输出的最小边界 [3]
         * @param bmax 输出的最大边界 [3]
         * @return true如果成功计算
         */
        public boolean calculateBounds(float[] bmin, float[] bmax) {
            if (nclusters == 0 || center == null || bmin.length < 3 || bmax.length < 3) {
                return false;
            }
            
            bmin[0] = bmin[1] = bmin[2] = Float.MAX_VALUE;
            bmax[0] = bmax[1] = bmax[2] = Float.MIN_VALUE;
            
            for (int i = 0; i < nclusters; i++) {
                float[] clusterCenter = new float[3];
                if (getClusterCenter(i, clusterCenter)) {
                    rcVmin(bmin, clusterCenter);
                    rcVmax(bmax, clusterCenter);
                }
            }
            
            return true;
        }
        
        /**
         * 获取总的链接数量
         * @return 总链接数量
         */
        public int getTotalLinkCount() {
            if (nlinks == null) {
                return 0;
            }
            
            int total = 0;
            for (int i = 0; i < nclusters; i++) {
                total += nlinks[i];
            }
            
            return total;
        }
        
        /**
         * 检查簇集合是否有效
         * @return true如果有效
         */
        public boolean isValid() {
            return nclusters > 0 && center != null && nlinks != null;
        }
        
        /**
         * 释放资源
         */
        public void dispose() {
            center = null;
            nlinks = null;
            links = null;
            nclusters = 0;
        }
    }
} 