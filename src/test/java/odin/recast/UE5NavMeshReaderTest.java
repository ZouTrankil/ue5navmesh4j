package odin.recast;

import org.junit.Test;
import static org.junit.Assert.*;

public class UE5NavMeshReaderTest {
    
    @Test
    public void testLoadFromFile() {
        UE5NavMeshReader reader = new UE5NavMeshReader();
        try {
             reader.loadFromFile("E:\\202505\\ue5navmesh4j\\ue5navmesh4j\\dungeon.obj");
             assertNotNull(reader.getNavMesh());
        } catch (Exception e) {
            fail("加载导航网格文件失败: " + e.getMessage());
        }
    }
} 