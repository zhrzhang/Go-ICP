#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <chrono>
#include <random>

using namespace std;

typedef pcl::PointXYZ Point;

void load_data(string data_dir, pcl::PointCloud<Point>::Ptr pc) {
    pcl::io::loadPLYFile<Point> (data_dir, *pc);
}

void scale_pc(pcl::PointCloud<Point>::Ptr pc_input, pcl::PointCloud<Point>::Ptr pc_scale, Point& min_pt, Point& max_pt) {
    pcl::copyPointCloud(*pc_input, *pc_scale);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pc_scale, centroid);

    for (int i = 0; i < pc_scale -> points.size(); i++) {
        pc_scale -> points[i].x -= centroid[0];
        pc_scale -> points[i].y -= centroid[1];
        pc_scale -> points[i].z -= centroid[2];
    }

    Eigen::Vector4f centroid_shifted;
    pcl::compute3DCentroid(*pc_scale, centroid_shifted);

    pcl::getMinMax3D(*pc_scale, min_pt, max_pt);
}

void transl_pc(pcl::PointCloud<Point>::Ptr pc_input, double (&transl)[3]) {
    for (int i = 0; i < pc_input -> points.size(); i++) {
        pc_input -> points[i].x += transl[0];
        pc_input -> points[i].y += transl[1];
        pc_input -> points[i].z += transl[2];
    }
}

void ply_manipl(string input_dir, string output_dir, bool is_shuffle = false) {
    std::vector<string> coords;
    
    ifstream plyfile(input_dir);
    ofstream output;
    output.open(output_dir);

    if (!plyfile.is_open()) {
        cout << "Error opening file" << std::endl;
        exit (1);
    }

    string coord;
    getline(plyfile, coord);

    while (coord != "end_header") {
        getline(plyfile, coord);

        stringstream ss;
        ss << coord;
        string identifier;
        string name;
        string value;

        ss >> identifier >> name >> value;
        if (name == "vertex")
            output << value << "\n";
    }

    while (!plyfile.eof()) {
        getline(plyfile, coord);
        coords.push_back(coord);
    }

    if (is_shuffle) {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        shuffle(coords.begin(), coords.end(), std::default_random_engine(seed));
    }

    for (int j = 0; j < coords.size(); ++j) {
        output << coords[j];
        output << "\n";
    }
    output.close();
}

int main() {
    std::string ref_dir = "/home/zrzhang/Documents/GoogleDrive/ResearchMAC/calibration/lidar_calibration/Lidar_Calib_dev_ws/src/pc_registration/data/bunny/data/bun000.ply";
    std::string mov_dir = "/home/zrzhang/Documents/GoogleDrive/ResearchMAC/calibration/lidar_calibration/Lidar_Calib_dev_ws/src/pc_registration/data/bunny/data/bun045.ply";

    pcl::PointCloud<Point>::Ptr pc_ref (new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr pc_mov (new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr pc_ref_scale (new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr pc_mov_scale (new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr pc_mov_scale_out (new pcl::PointCloud<Point>);

    // load the original point cloud data
    load_data(ref_dir, pc_ref);
    load_data(mov_dir, pc_mov);

    // scale the point cloud to centralize the point cloud at (0, 0, 0)
    Point pt_min_ref;
    Point pt_max_ref;
    Point pt_min_mov;
    Point pt_max_mov;

    scale_pc(pc_ref, pc_ref_scale, pt_min_ref, pt_max_ref);
    scale_pc(pc_mov, pc_mov_scale, pt_min_mov, pt_max_mov);

    // remove the NaNs in the point cloud data and save the corresponding .ply file
    std::vector<int> indices_ref;
    pcl::removeNaNFromPointCloud(*pc_ref_scale, *pc_ref_scale, indices_ref);
    string ref_scale_dir = "/home/zrzhang/Documents/GoogleDrive/ResearchMAC/calibration/lidar_calibration/Lidar_Calib_dev_ws/src/Go-ICP/bun000_scale.ply";
    pcl::io::savePLYFileASCII(ref_scale_dir, *pc_ref_scale);

    // translate the target point cloud, remove the NaNs in the translated point cloud, and save the resulting point cloud into a .ply file
    double transl_vec[3] = {12, 26, 33};
    transl_pc(pc_mov_scale, transl_vec);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_mov_scale, *pc_mov_scale_out, indices);

    std::string transl_pc_dir = "/home/zrzhang/Documents/GoogleDrive/ResearchMAC/calibration/lidar_calibration/Lidar_Calib_dev_ws/src/Go-ICP/bun045_transl.ply";
    pcl::io::savePLYFileASCII(transl_pc_dir, *pc_mov_scale_out);

    // read the scaled ref point cloud and manipulate the ply file
    string ref_shuffled_dir = "/home/zrzhang/Documents/GoogleDrive/ResearchMAC/calibration/lidar_calibration/Lidar_Calib_dev_ws/src/Go-ICP/bun000_shuffled.txt";
    ply_manipl(ref_scale_dir, ref_shuffled_dir, true);

    // read the translated pc_mov and manipulate the ply file
    string mov_output_dir = "/home/zrzhang/Documents/GoogleDrive/ResearchMAC/calibration/lidar_calibration/Lidar_Calib_dev_ws/src/Go-ICP/bun045_scaled.txt";
    ply_manipl(transl_pc_dir, mov_output_dir, false);
    return 0;
}
