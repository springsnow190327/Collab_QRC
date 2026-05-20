/*
Copyright 2025 NVIDIA CORPORATION

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "nvblox/core/types.h"
#include "nvblox/io/mesh_io.h"
#include "nvblox/io/ply_parser.h"
#include "nvblox/io/pointcloud_io.h"
#include "nvblox/mesh/mesh_block.h"
#include "nvblox/sensors/pointcloud.h"

using namespace nvblox;

constexpr float kFloatEpsilon = 1e-3f;

class PlyIOTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::srand(0);
    // Create test directory for all PLY test files
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up only the files that were created via getTestFilePath()
    for (const auto& filepath : created_files_) {
      std::filesystem::remove(filepath);
    }
  }

  // Helper to get full path to test file
  std::string getTestFilePath(const std::string& filename) {
    std::string full_path = test_dir_ + "/" + filename;
    created_files_.push_back(full_path);
    return full_path;
  }

  const std::string test_dir_ = "test_ply_files";
  std::vector<std::string> created_files_;

  // Helper to create a simple test mesh
  ColorMeshLayer createTestMesh() {
    constexpr float kBlockSize = 0.1f;
    ColorMeshLayer mesh_layer(kBlockSize, MemoryType::kUnified);

    // Create a simple mesh block with triangle
    Index3D block_idx(0, 0, 0);
    auto mesh_block = mesh_layer.allocateBlockAtIndex(block_idx);

    // Add 3 vertices forming a triangle
    mesh_block->vertices.push_back(Vector3f(0.0f, 0.0f, 0.0f));
    mesh_block->vertices.push_back(Vector3f(1.0f, 0.0f, 0.0f));
    mesh_block->vertices.push_back(Vector3f(0.5f, 1.0f, 0.0f));

    // Add colors for vertices (using vertex_appearances for ColorMeshLayer)
    mesh_block->vertex_appearances.push_back(Color(255, 0, 0));  // Red
    mesh_block->vertex_appearances.push_back(Color(0, 255, 0));  // Green
    mesh_block->vertex_appearances.push_back(Color(0, 0, 255));  // Blue

    // Add normals
    mesh_block->vertex_normals.push_back(Vector3f(0.0f, 0.0f, 1.0f));
    mesh_block->vertex_normals.push_back(Vector3f(0.0f, 0.0f, 1.0f));
    mesh_block->vertex_normals.push_back(Vector3f(0.0f, 0.0f, 1.0f));

    // Add triangle indices
    mesh_block->triangles.push_back(0);
    mesh_block->triangles.push_back(1);
    mesh_block->triangles.push_back(2);

    return mesh_layer;
  }

  // Helper to create a test pointcloud
  Pointcloud createTestPointcloud() {
    Pointcloud pointcloud(MemoryType::kUnified);

    // Add some test points
    std::vector<Vector3f> points_host = {
        Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f),
        Vector3f(7.0f, 8.0f, 9.0f), Vector3f(10.0f, 11.0f, 12.0f)};

    pointcloud.copyPointsFromAsync(points_host, CudaStreamOwning());

    return pointcloud;
  }

  // Helper to create a test pointcloud with timestamps
  Pointcloud createTestPointcloudWithTimestamps() {
    Pointcloud pointcloud(MemoryType::kUnified);

    // Add some test points
    std::vector<Vector3f> points_host = {
        Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f),
        Vector3f(7.0f, 8.0f, 9.0f), Vector3f(10.0f, 11.0f, 12.0f)};

    // Add corresponding timestamps (in milliseconds)
    std::vector<Time> timestamps_host = {Time(100), Time(200), Time(300),
                                         Time(400)};

    CudaStreamOwning cuda_stream;
    pointcloud.copyPointsFromAsync(points_host, cuda_stream);
    pointcloud.copyTimestampsFromAsync(timestamps_host, cuda_stream);
    cuda_stream.synchronize();

    return pointcloud;
  }
};

TEST_F(PlyIOTest, MeshRoundTrip) {
  // Create test mesh layer
  ColorMeshLayer original_mesh_layer = createTestMesh();

  // Convert the mesh layer to a ColorMesh object
  CudaStreamOwning cuda_stream;
  std::shared_ptr<const ColorMesh> original_mesh_ptr =
      original_mesh_layer.getMesh(cuda_stream);
  ASSERT_NE(original_mesh_ptr, nullptr);

  // Copy original mesh data to host for comparison
  std::vector<Vector3f> original_vertices =
      original_mesh_ptr->vertices.toVectorAsync(cuda_stream);
  std::vector<Vector3f> original_normals =
      original_mesh_ptr->vertex_normals.toVectorAsync(cuda_stream);
  std::vector<Color> original_colors =
      original_mesh_ptr->vertex_appearances.toVectorAsync(cuda_stream);
  std::vector<int> original_triangles =
      original_mesh_ptr->triangles.toVectorAsync(cuda_stream);
  cuda_stream.synchronize();

  // Write mesh to PLY file
  const std::string filename = getTestFilePath("test_mesh_roundtrip.ply");
  EXPECT_TRUE(io::outputColorMeshLayerToPly(original_mesh_layer, filename));

  // Read mesh back using PLY parser
  io::PlyParser parser(filename);
  ASSERT_TRUE(parser.isValid());

  // Verify header info
  EXPECT_TRUE(parser.hasColors());
  EXPECT_TRUE(parser.hasNormals());
  EXPECT_TRUE(parser.hasTriangles());

  EXPECT_EQ(parser.numVertices(), original_vertices.size());
  EXPECT_EQ(parser.numFaces(), original_triangles.size() / 3);

  // Convert parsed data to ColorMesh using the new toMesh function
  ColorMesh loaded_mesh(MemoryType::kUnified);
  ASSERT_TRUE(parser.toMesh(&loaded_mesh, cuda_stream));
  cuda_stream.synchronize();

  // Direct mesh-to-mesh comparison - Verify vertices
  EXPECT_EQ(loaded_mesh.vertices.size(), original_vertices.size());
  for (size_t i = 0; i < loaded_mesh.vertices.size(); i++) {
    EXPECT_NEAR(loaded_mesh.vertices[i].x(), original_vertices[i].x(),
                kFloatEpsilon);
    EXPECT_NEAR(loaded_mesh.vertices[i].y(), original_vertices[i].y(),
                kFloatEpsilon);
    EXPECT_NEAR(loaded_mesh.vertices[i].z(), original_vertices[i].z(),
                kFloatEpsilon);
  }

  // Direct mesh-to-mesh comparison - Verify colors (vertex_appearances)
  EXPECT_EQ(loaded_mesh.vertex_appearances.size(), original_colors.size());
  for (size_t i = 0; i < loaded_mesh.vertex_appearances.size(); i++) {
    EXPECT_EQ(loaded_mesh.vertex_appearances[i].r(), original_colors[i].r());
    EXPECT_EQ(loaded_mesh.vertex_appearances[i].g(), original_colors[i].g());
    EXPECT_EQ(loaded_mesh.vertex_appearances[i].b(), original_colors[i].b());
  }

  // Direct mesh-to-mesh comparison - Verify normals (vertex_normals)
  EXPECT_EQ(loaded_mesh.vertex_normals.size(), original_normals.size());
  for (size_t i = 0; i < loaded_mesh.vertex_normals.size(); i++) {
    EXPECT_NEAR(loaded_mesh.vertex_normals[i].x(), original_normals[i].x(),
                kFloatEpsilon);
    EXPECT_NEAR(loaded_mesh.vertex_normals[i].y(), original_normals[i].y(),
                kFloatEpsilon);
    EXPECT_NEAR(loaded_mesh.vertex_normals[i].z(), original_normals[i].z(),
                kFloatEpsilon);
  }

  // Direct mesh-to-mesh comparison - Verify triangles
  EXPECT_EQ(loaded_mesh.triangles.size(), original_triangles.size());
  for (size_t i = 0; i < loaded_mesh.triangles.size(); i++) {
    EXPECT_EQ(loaded_mesh.triangles[i], original_triangles[i]);
  }
}

TEST_F(PlyIOTest, PointcloudRoundTrip) {
  // Create test pointcloud
  Pointcloud original_pointcloud = createTestPointcloud();

  // Write pointcloud to PLY file (without intensities or timestamps - basic
  // pointcloud)
  const std::string filename = getTestFilePath("test_pointcloud_roundtrip.ply");
  CudaStreamOwning cuda_stream;
  EXPECT_TRUE(
      io::outputPointcloudToPly(original_pointcloud, filename, cuda_stream));

  // Read pointcloud back using PLY parser
  io::PlyParser parser(filename);
  ASSERT_TRUE(parser.isValid());

  // Verify data integrity (no intensities or timestamps in basic pointcloud
  // output)
  EXPECT_FALSE(parser.hasIntensities());
  EXPECT_FALSE(parser.hasTimestamps());
  EXPECT_EQ(parser.numVertices(), original_pointcloud.size());

  // Convert parsed data to Pointcloud using the new toPointcloud function
  Pointcloud loaded_pointcloud(MemoryType::kUnified);
  ASSERT_TRUE(parser.toPointcloud(&loaded_pointcloud, cuda_stream));
  cuda_stream.synchronize();

  // Direct pointcloud-to-pointcloud comparison - Verify point count
  EXPECT_EQ(loaded_pointcloud.size(), original_pointcloud.size());
  EXPECT_FALSE(loaded_pointcloud.timestamps_ms().has_value());

  // Direct pointcloud-to-pointcloud comparison - Verify points
  // Access unified memory directly without copying
  for (int i = 0; i < loaded_pointcloud.size(); i++) {
    EXPECT_NEAR(loaded_pointcloud.point(i).x(),
                original_pointcloud.point(i).x(), kFloatEpsilon);
    EXPECT_NEAR(loaded_pointcloud.point(i).y(),
                original_pointcloud.point(i).y(), kFloatEpsilon);
    EXPECT_NEAR(loaded_pointcloud.point(i).z(),
                original_pointcloud.point(i).z(), kFloatEpsilon);
  }
}

TEST_F(PlyIOTest, PointcloudWithTimestampsRoundTrip) {
  // Create test pointcloud with timestamps
  Pointcloud original_pointcloud = createTestPointcloudWithTimestamps();
  ASSERT_TRUE(original_pointcloud.timestamps_ms().has_value());

  // Write pointcloud to PLY file (with timestamps)
  CudaStreamOwning cuda_stream;
  const std::string filename =
      getTestFilePath("test_pointcloud_timestamps_roundtrip.ply");
  EXPECT_TRUE(
      io::outputPointcloudToPly(original_pointcloud, filename, cuda_stream));

  // Read pointcloud back using PLY parser
  io::PlyParser parser(filename);
  ASSERT_TRUE(parser.isValid());

  // Verify data integrity
  EXPECT_TRUE(parser.hasTimestamps());
  EXPECT_EQ(parser.numVertices(), original_pointcloud.size());

  // Convert parsed data to Pointcloud
  Pointcloud loaded_pointcloud(MemoryType::kUnified);
  ASSERT_TRUE(parser.toPointcloud(&loaded_pointcloud, cuda_stream));
  cuda_stream.synchronize();

  // Verify point count and timestamps flag
  EXPECT_EQ(loaded_pointcloud.size(), original_pointcloud.size());
  EXPECT_TRUE(loaded_pointcloud.timestamps_ms().has_value());

  // Verify points
  for (int i = 0; i < loaded_pointcloud.size(); i++) {
    EXPECT_NEAR(loaded_pointcloud.point(i).x(),
                original_pointcloud.point(i).x(), kFloatEpsilon);
    EXPECT_NEAR(loaded_pointcloud.point(i).y(),
                original_pointcloud.point(i).y(), kFloatEpsilon);
    EXPECT_NEAR(loaded_pointcloud.point(i).z(),
                original_pointcloud.point(i).z(), kFloatEpsilon);
  }

  // Verify timestamps
  for (int i = 0; i < loaded_pointcloud.size(); i++) {
    EXPECT_EQ(loaded_pointcloud.timestamps_ms().value()[i],
              original_pointcloud.timestamps_ms().value()[i])
        << "Timestamp mismatch at index " << i;
  }
}

TEST_F(PlyIOTest, PointsWithIntensityRoundTrip) {
  // Create test data
  std::vector<Vector3f> original_points = {Vector3f(1.0f, 2.0f, 3.0f),
                                           Vector3f(4.0f, 5.0f, 6.0f)};
  std::vector<float> original_intensities = {0.25f, 0.75f};

  // Write points with intensities to PLY file
  const std::string filename =
      getTestFilePath("test_points_intensity_roundtrip.ply");
  EXPECT_TRUE(
      io::outputPointsToPly(original_points, original_intensities, filename));

  // Read points back using PLY parser
  io::PlyParser parser(filename);
  ASSERT_TRUE(parser.isValid());

  // Verify data integrity
  EXPECT_TRUE(parser.hasIntensities());
  EXPECT_EQ(parser.numVertices(), original_points.size());

  // Get parsed data
  const auto& loaded_points = parser.points();
  const auto& loaded_intensities = parser.intensities();

  // Verify points
  EXPECT_EQ(loaded_points.size(), original_points.size());
  for (size_t i = 0; i < loaded_points.size(); i++) {
    EXPECT_NEAR(loaded_points[i].x(), original_points[i].x(), kFloatEpsilon);
    EXPECT_NEAR(loaded_points[i].y(), original_points[i].y(), kFloatEpsilon);
    EXPECT_NEAR(loaded_points[i].z(), original_points[i].z(), kFloatEpsilon);
  }

  // Verify intensities
  EXPECT_EQ(loaded_intensities.size(), original_intensities.size());
  for (size_t i = 0; i < loaded_intensities.size(); i++) {
    EXPECT_NEAR(loaded_intensities[i], original_intensities[i], kFloatEpsilon);
  }
}

TEST_F(PlyIOTest, InvalidPlyFiles) {
  // Test 1: Missing 'ply' header
  {
    const std::string filename = getTestFilePath("test_invalid_no_header.ply");
    std::ofstream file(filename);
    file << "format ascii 1.0\n";
    file << "element vertex 0\n";
    file << "end_header\n";
    file.close();

    io::PlyParser parser(filename);
    EXPECT_FALSE(parser.isValid());
  }

  // Test 2: Non-ASCII format
  {
    const std::string filename = getTestFilePath("test_invalid_binary.ply");
    std::ofstream file(filename);
    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "element vertex 1\n";
    file << "end_header\n";
    file.close();

    io::PlyParser parser(filename);
    EXPECT_FALSE(parser.isValid());
  }

  // Test 3: No vertices
  {
    const std::string filename =
        getTestFilePath("test_invalid_no_vertices.ply");
    std::ofstream file(filename);
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex 0\n";
    file << "end_header\n";
    file.close();

    io::PlyParser parser(filename);
    EXPECT_FALSE(parser.isValid());
  }

  // Test 4: Mismatched vertex count
  {
    const std::string filename =
        getTestFilePath("test_invalid_vertex_count.ply");
    std::ofstream file(filename);
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex 3\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
    file << "1.0 2.0 3.0\n";  // Only 1 vertex provided, expected 3
    file.close();

    io::PlyParser parser(filename);
    EXPECT_FALSE(parser.isValid());
  }

  // Test 5: File doesn't exist
  {
    const std::string filename =
        getTestFilePath("test_file_does_not_exist.ply");
    io::PlyParser parser(filename);
    EXPECT_FALSE(parser.isValid());
  }

  // Test 6: Mesh without triangles cannot be converted to mesh
  {
    const std::string filename = getTestFilePath("test_no_triangles.ply");
    std::ofstream file(filename);
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex 3\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
    file << "1.0 2.0 3.0\n";
    file << "4.0 5.0 6.0\n";
    file << "7.0 8.0 9.0\n";
    file.close();

    io::PlyParser parser(filename);
    ASSERT_TRUE(parser.isValid());
    EXPECT_FALSE(parser.hasTriangles());

    // Should fail to convert to mesh (no triangles)
    ColorMesh mesh(MemoryType::kUnified);
    CudaStreamOwning cuda_stream;
    EXPECT_FALSE(parser.toMesh(&mesh, cuda_stream));
  }
}

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
