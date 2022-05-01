#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>

class Camera {

public:

    std::vector<std::string> param_list;
    float image_width;
    float image_height;
    float fx;
    float fy;
    float cx;
    float cy;

    //RGBD画像群のディレクトリにある camera_params.txtの設定データから読み込む
    Camera(std::string camera_param_file) { 
        std::ifstream input_file(camera_param_file);
        if (!input_file.is_open()) {
            throw std::runtime_error("failed to open file!");
        }

        std::string line;
        while (std::getline(input_file, line)) {
            param_list.push_back(line);
        }
        input_file.close();
        image_width = std::stoi(param_list[0]);
        image_height = std::stoi(param_list[1]);
        fx = std::stof(param_list[2]);
        fy = std::stof(param_list[3]);
        cx = std::stof(param_list[4]);
        cy = std::stof(param_list[5]);
    }

};

int main(int argc, char *argv[]) {
    printf(argv[0]);
    using namespace open3d;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    int N = 100;  //フォルダに入ってる多視点RGBD画像の視点数
    int skip_N_frames = 1;
    std::string data_dir;

    //data_dir = "TUM/bunny/Synthetic_bunny_Circle/";
    // data_dir = "TUM/bunny/Synthetic_bunny_Wave/"

    data_dir = "TUM/Kenny/Synthetic_Kenny_Circle/";
    // data_dir = "TUM/Kenny/Synthetic_Kenny_Wave/"

    // data_dir = "TUM/tank/Synthetic_Tank_Circle/";
    // data_dir = "TUM/tank/Synthetic_Tank_Wave/"

    // data_dir = "TUM/Teddy/Synthetic_Teddy_Circle/";
    // data_dir = "TUM/Teddy/Synthetic_Teddy_Wave/"

    // data_dir = "TUM/Leopard/Synthetic_Leopard_Circle/"
    // data_dir = "TUM/Leopard/Synthetic_Leopard_Wave/"

    // data_dir = "TUM/Teddy/Kinect_Teddy_Handheld/"
    // data_dir = "TUM/Leopard/Kinect_Leopard_Turntable/"

    // data_dir = "rgb_inhand/cheezit/"
    // data_dir = "rgb_inhand/mustard/"
    //
    // data_dir = "stanford/figure-mvs/"
    // data_dir = "stanford/sokrates-mvs/"

    Camera *camera = new Camera(data_dir + "camera_params.txt");
    auto camera_intrinsic = open3d::camera::PinholeCameraIntrinsic(
            camera->image_width, camera->image_height, camera->fx, camera->fy,
            camera->cx, camera->cy);

    float radius = 0.05;
    int max_nn = 30;
    auto const kdt = open3d::geometry::KDTreeSearchParamHybrid(radius, max_nn);
    std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> rgbds;
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pcds;

    std::string rgb_file;
    std::string depth_file;
    std::string ply_file;
    std::string all_ply_file;
    std::string mesh_file;
    cv::Mat3b RGBim;
    cv::Mat1s Depthim;

    for (int i = 0; i < N; i++) {
        //入力のRGB画像のファイル名の設定
        std::ostringstream num;
        num << std::setfill('0') << std::setw(6) << i;
        rgb_file = data_dir + "color_" + num.str() + ".png";  //入力のRGB画像のファイル名の設定
        depth_file = data_dir + "depth_" + num.str() + ".png";  //入力のDepth画像のファイル名の設定
        ply_file = data_dir + "points_" + num.str() + ".ply";  // 3D点群（各視点）を保存するためのファイル名の設定
        all_ply_file = data_dir + "allpoints.ply";  //全視点を統合した3D点群を保存するためのファイル名の設定
        mesh_file = data_dir + "mesh.ply";  //統合後生成する3Dメッシュデータを保存するためのファイル名の設定

        RGBim = cv::imread(rgb_file, cv::IMREAD_COLOR);
        cv::cvtColor(RGBim, RGBim, cv::COLOR_BGR2RGB);
        Depthim = cv::imread(depth_file, cv::IMREAD_ANYDEPTH);

        // RGB画像をファイルから読み込む
        open3d::geometry::Image color;
        color.Prepare(RGBim.cols, RGBim.rows, RGBim.channels(), sizeof(uchar));
        memcpy(color.data_.data(), RGBim.data, RGBim.rows * RGBim.cols * RGBim.channels() * sizeof(uchar));

        // Depth画像をファイルから読み込む, 16bitのpng画像
        open3d::geometry::Image depth;
        depth.Prepare(Depthim.cols, Depthim.rows, Depthim.channels(), sizeof(short));
        memcpy(depth.data_.data(), Depthim.data, Depthim.rows * Depthim.cols * Depthim.channels() * sizeof(short));

        // colorとdepthからOpen3Dのrgbd画像を生成
        std::shared_ptr<open3d::geometry::RGBDImage> rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(color, depth, 1000, 3.0, false);
        rgbds.push_back(rgbd);

        //生成したrgbd画像とカメラ内部パラメータから3D点群を生成
        std::shared_ptr<open3d::geometry::PointCloud> p = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd, camera_intrinsic);

        //ダウンサンプリングする
        //p->VoxelDownSample(0.01);

        //生成した３D点群の各点群の法線ベクトルを計算
        p->EstimateNormals(kdt);

        // 3D点群のリストに追加
        pcds.push_back(p);
    }

    open3d::visualization::DrawGeometries({pcds[11], pcds[12]}, "input", 640, 480);

    double th = 0.05;
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    open3d::pipelines::registration::RegistrationResult result;
    std::vector<open3d::pipelines::registration::RegistrationResult> results;
    std::vector<Eigen::Matrix4d_u> RT;
    //最初の視点RT[0]が基準になるので，RT[0]には無変換行列を代入しておく
    RT.push_back(mat);

    std::ofstream outputfile("log.txt");
    for (int i = 0; i < N-1; i++) {


        result = open3d::pipelines::registration::RegistrationICP(
            *pcds[i + 1], *pcds[i], th, mat,
            open3d::pipelines::registration::TransformationEstimationPointToPoint()
        );



        //result = open3d::pipelines::registration::RegistrationColoredICP(*pcds[i + 1], *pcds[i], th, mat,open3d::pipelines::registration::TransformationEstimationForColoredICP(),
        //        open3d::pipelines::registration::ICPConvergenceCriteria(1e-4,1e-4, 50));
         
        // 
        //result = open3d::pipelines::registration::RegistrationGeneralizedICP(
        //        *pcds[i + 1], *pcds[i], th, mat,
        //        open3d::pipelines::registration::
        //                TransformationEstimationForGeneralizedICP());
        
        results.push_back(result);
        RT.push_back(result.transformation_);


        outputfile << result.transformation_;
    }
    outputfile.close();


    for (int i = 1; i < N; i++) {
        RT[i] = RT[i - 1] * RT[i];
    }

    for (int i = 0; i < N; i++) {
        pcds[i]->Transform(RT[i]);
    }

    //ICP位置合わせ後の全３D点群を表示
    open3d::visualization::DrawGeometries({pcds[0]}, "input", 640, 480);

    auto volume = open3d::pipelines::integration::ScalableTSDFVolume(0.001, 0.01, open3d::pipelines::integration::TSDFVolumeColorType::RGB8);

    for (int i = 0; i < N; i++) {
        volume.Integrate(*rgbds[i], camera_intrinsic, RT[i].inverse());
    }

    auto mesh = volume.ExtractTriangleMesh();
    open3d::visualization::DrawGeometries({mesh});

    return 0;
}
