package odin.recast;

import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshParams;
import org.recast4j.detour.Poly;
import org.recast4j.detour.NavMeshBuilder;
import org.recast4j.detour.MeshHeader;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class UE5NavMeshReader {
    private NavMesh navMesh;
    private List<float[]> vertices;      // 存储顶点坐标
    private List<float[]> texCoords;     // 存储纹理坐标
    private List<float[]> normals;       // 存储法线
    private List<int[]> indices;         // 存储索引数据
    private Map<String, List<int[]>> groups;  // 存储组信息
    private String currentGroup;         // 当前组名

    public UE5NavMeshReader() {
        vertices = new ArrayList<>();
        texCoords = new ArrayList<>();
        normals = new ArrayList<>();
        indices = new ArrayList<>();
        groups = new HashMap<>();
        currentGroup = "default";
    }

    /**
     * 从OBJ文件加载导航网格数据
     * @param filePath OBJ文件路径
     * @throws IOException 如果文件读取失败
     * @throws IllegalArgumentException 如果文件格式不正确
     */
    public void loadFromFile(String filePath) throws IOException, IllegalArgumentException {
        File file = new File(filePath);
        if (!file.exists()) {
            throw new IOException("文件不存在: " + filePath);
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            int lineNumber = 0;
            while ((line = reader.readLine()) != null) {
                lineNumber++;
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }

                try {
                    parseLine(line);
                } catch (Exception e) {
                    throw new IllegalArgumentException("解析第 " + lineNumber + " 行时出错: " + e.getMessage());
                }
            }

            if (vertices.isEmpty()) {
                throw new IllegalArgumentException("文件中没有找到顶点数据");
            }

            if (indices.isEmpty()) {
                throw new IllegalArgumentException("文件中没有找到面数据");
            }

            // 创建NavMesh数据
            createNavMesh();
        }
    }

    private void parseLine(String line) {
        String[] parts = line.split("\\s+");
        if (parts.length == 0) return;

        switch (parts[0]) {
            case "v":  // 顶点
                if (parts.length >= 4) {
                    float[] vertex = new float[3];
                    vertex[0] = Float.parseFloat(parts[1]); // x
                    vertex[1] = Float.parseFloat(parts[2]); // y
                    vertex[2] = Float.parseFloat(parts[3]); // z
                    vertices.add(vertex);
                }
                break;

            case "vt":  // 纹理坐标
                if (parts.length >= 3) {
                    float[] texCoord = new float[2];
                    texCoord[0] = Float.parseFloat(parts[1]); // u
                    texCoord[1] = Float.parseFloat(parts[2]); // v
                    texCoords.add(texCoord);
                }
                break;

            case "vn":  // 法线
                if (parts.length >= 4) {
                    float[] normal = new float[3];
                    normal[0] = Float.parseFloat(parts[1]); // nx
                    normal[1] = Float.parseFloat(parts[2]); // ny
                    normal[2] = Float.parseFloat(parts[3]); // nz
                    normals.add(normal);
                }
                break;

            case "f":  // 面
                if (parts.length >= 4) {
                    // 处理多边形面，将其三角化
                    List<int[]> faceIndices = new ArrayList<>();
                    for (int i = 1; i < parts.length; i++) {
                        String[] vertexData = parts[i].split("/");
                        int[] index = new int[3];  // [顶点索引, 纹理索引, 法线索引]
                        
                        // 顶点索引
                        index[0] = Integer.parseInt(vertexData[0]) - 1;
                        
                        // 纹理索引（如果有）
                        if (vertexData.length > 1 && !vertexData[1].isEmpty()) {
                            index[1] = Integer.parseInt(vertexData[1]) - 1;
                        } else {
                            index[1] = -1;
                        }
                        
                        // 法线索引（如果有）
                        if (vertexData.length > 2 && !vertexData[2].isEmpty()) {
                            index[2] = Integer.parseInt(vertexData[2]) - 1;
                        } else {
                            index[2] = -1;
                        }
                        
                        faceIndices.add(index);
                    }
                    
                    // 三角化
                    for (int i = 1; i < faceIndices.size() - 1; i++) {
                        int[] triangle = new int[3];
                        triangle[0] = faceIndices.get(0)[0];
                        triangle[1] = faceIndices.get(i)[0];
                        triangle[2] = faceIndices.get(i + 1)[0];
                        indices.add(triangle);
                        
                        // 添加到当前组
                        groups.computeIfAbsent(currentGroup, k -> new ArrayList<>()).add(triangle);
                    }
                }
                break;

            case "g":  // 组
                if (parts.length >= 2) {
                    currentGroup = parts[1];
                }
                break;

            default:
                break;
        }
    }

    /**
     * 获取所有顶点坐标
     * @return 顶点坐标列表
     */
    public List<float[]> getVertices() {
        return vertices;
    }

    /**
     * 获取所有纹理坐标
     * @return 纹理坐标列表
     */
    public List<float[]> getTexCoords() {
        return texCoords;
    }

    /**
     * 获取所有法线
     * @return 法线列表
     */
    public List<float[]> getNormals() {
        return normals;
    }

    /**
     * 获取所有索引数据
     * @return 索引数据列表
     */
    public List<int[]> getIndices() {
        return indices;
    }

    /**
     * 获取所有组信息
     * @return 组信息映射
     */
    public Map<String, List<int[]>> getGroups() {
        return groups;
    }

    /**
     * 获取导航网格实例
     * @return NavMesh实例
     */
    public NavMesh getNavMesh() {
        return navMesh;
    }

    private void createNavMesh() {
        // 创建NavMesh参数
        NavMeshParams params = new NavMeshParams();
        params.orig[0] = 0;
        params.orig[1] = 0;
        params.orig[2] = 0;
        params.tileWidth = 32;
        params.tileHeight = 32;
        params.maxTiles = 256;
        params.maxPolys = 16384;

        // 创建NavMesh实例
        navMesh = new NavMesh(params, 6);
        
        // 创建MeshData
        MeshData meshData = new MeshData();
        meshData.verts = new float[vertices.size() * 3];
        meshData.polys = new Poly[indices.size()];
        
        // 填充顶点数据
        for (int i = 0; i < vertices.size(); i++) {
            float[] vertex = vertices.get(i);
            meshData.verts[i * 3] = vertex[0];
            meshData.verts[i * 3 + 1] = vertex[1];
            meshData.verts[i * 3 + 2] = vertex[2];
        }

        // 填充多边形数据
        for (int i = 0; i < indices.size(); i++) {
            int[] index = indices.get(i);
            Poly poly = new Poly(i, 6);
            poly.vertCount = 3;  // 三角形
            
            // 设置顶点索引
            poly.verts[0] = index[0];
            poly.verts[1] = index[1];
            poly.verts[2] = index[2];
            
            // 设置邻接多边形引用
            poly.neis[0] = 0;
            poly.neis[1] = 0;
            poly.neis[2] = 0;
            
            // 设置多边形类型和区域
            poly.flags = 1;  // 默认标志
            poly.setArea(0);   // 默认区域
            
            meshData.polys[i] = poly;
        }

        // 创建MeshHeader
        MeshHeader header = new MeshHeader();
        header.magic = MeshHeader.DT_NAVMESH_MAGIC;
        header.version = MeshHeader.DT_NAVMESH_VERSION;
        header.x = 0;
        header.y = 0;
        header.layer = 0;
        header.userId = 0;
        header.polyCount = indices.size();
        header.vertCount = vertices.size();
        header.maxLinkCount = 0;
        header.detailMeshCount = 0;
        header.detailVertCount = 0;
        header.detailTriCount = 0;
        header.bvNodeCount = 0;
        header.offMeshBase = 0;
        header.walkableHeight = 2.0f;
        header.walkableRadius = 0.6f;
        header.walkableClimb = 0.9f;
        header.bmin[0] = Float.MAX_VALUE;
        header.bmin[1] = Float.MAX_VALUE;
        header.bmin[2] = Float.MAX_VALUE;
        header.bmax[0] = Float.MIN_VALUE;
        header.bmax[1] = Float.MIN_VALUE;
        header.bmax[2] = Float.MIN_VALUE;

        // 计算边界框
        for (float[] vertex : vertices) {
            header.bmin[0] = Math.min(header.bmin[0], vertex[0]);
            header.bmin[1] = Math.min(header.bmin[1], vertex[1]);
            header.bmin[2] = Math.min(header.bmin[2], vertex[2]);
            header.bmax[0] = Math.max(header.bmax[0], vertex[0]);
            header.bmax[1] = Math.max(header.bmax[1], vertex[1]);
            header.bmax[2] = Math.max(header.bmax[2], vertex[2]);
        }

        // 设置header到meshData
        meshData.header = header;

        // 添加Tile到NavMesh
        navMesh.addTile(meshData, 0, 0);
    }
} 