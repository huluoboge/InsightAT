// SuperPoint ONNX C++ Inference API
// 使用ONNX Runtime C++ API进行GPU推理

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <chrono>

class SuperPointONNX {
private:
    Ort::Env env;
    Ort::Session session;
    Ort::SessionOptions session_options;
    Ort::AllocatorWithDefaultOptions allocator;
    
    std::vector<std::string> input_names_str;
    std::vector<std::string> output_names_str;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    
    std::vector<int64_t> input_shape;
    
public:
    SuperPointONNX(const std::string& model_path, bool use_gpu = true) 
        : env(ORT_LOGGING_LEVEL_WARNING, "SuperPoint"),
          session(nullptr) {
        
        // 配置Session选项
        session_options.SetIntraOpNumThreads(4);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
        
        // 配置GPU (CUDA)
        if (use_gpu) {
            try {
                OrtCUDAProviderOptions cuda_options;
                cuda_options.device_id = 0;
                cuda_options.arena_extend_strategy = 1;  // kSameAsRequested
                cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchDefault;
                cuda_options.do_copy_in_default_stream = 1;
                
                session_options.AppendExecutionProvider_CUDA(cuda_options);
                std::cout << "✓ 启用GPU推理 (CUDA)" << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "⚠ GPU初始化失败，使用CPU: " << e.what() << std::endl;
            }
        }
        
        // 创建Session
        session = Ort::Session(env, model_path.c_str(), session_options);
        
        // 获取输入输出信息
        size_t num_input_nodes = session.GetInputCount();
        size_t num_output_nodes = session.GetOutputCount();
        
        std::cout << "模型信息:" << std::endl;
        std::cout << "  输入节点数: " << num_input_nodes << std::endl;
        std::cout << "  输出节点数: " << num_output_nodes << std::endl;
        
        // 输入名称
        for (size_t i = 0; i < num_input_nodes; i++) {
            auto input_name = session.GetInputNameAllocated(i, allocator);
            input_names_str.push_back(std::string(input_name.get()));
            std::cout << "  输入[" << i << "]: " << input_names_str.back() << std::endl;
        }
        
        // 输出名称
        for (size_t i = 0; i < num_output_nodes; i++) {
            auto output_name = session.GetOutputNameAllocated(i, allocator);
            output_names_str.push_back(std::string(output_name.get()));
            std::cout << "  输出[" << i << "]: " << output_names_str.back() << std::endl;
        }
        
        // 转换为const char*
        for (const auto& name : input_names_str) {
            input_names.push_back(name.c_str());
        }
        for (const auto& name : output_names_str) {
            output_names.push_back(name.c_str());
        }
        
        // 默认输入shape (动态shape，运行时会调整)
        input_shape = {1, 1, 480, 640};
    }
    
    struct Detection {
        std::vector<cv::Point2f> keypoints;
        std::vector<float> scores;
        std::vector<std::vector<float>> descriptors;
    };
    
    // NMS (非极大值抑制)
    cv::Mat apply_nms(const cv::Mat& scores_mat, int nms_radius) {
        int height = scores_mat.rows;
        int width = scores_mat.cols;
        
        // 初始化输出掩码
        cv::Mat nms_mask = cv::Mat::ones(height, width, CV_8U);
        
        // 使用局部最大值池化实现 NMS
        cv::Mat dilated;
        int kernel_size = 2 * nms_radius + 1;
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        cv::dilate(scores_mat, dilated, kernel);
        
        // 只保留局部最大值点
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                float score = scores_mat.at<float>(y, x);
                float max_score = dilated.at<float>(y, x);
                
                // 如果不是局部最大值，则抑制
                if (score < max_score) {
                    nms_mask.at<uchar>(y, x) = 0;
                }
            }
        }
        
        return nms_mask;
    }
    
    Detection infer(const cv::Mat& image, float threshold = 0.005, int nms_radius = 4) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 预处理
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }
        
        // 转换为float并归一化
        cv::Mat float_gray;
        gray.convertTo(float_gray, CV_32F, 1.0 / 255.0);
        
        // 设置输入shape
        input_shape = {1, 1, gray.rows, gray.cols};
        
        // 创建输入tensor
        size_t input_tensor_size = 1 * 1 * gray.rows * gray.cols;
        std::vector<float> input_tensor_values(input_tensor_size);
        
        // 复制数据
        memcpy(input_tensor_values.data(), float_gray.data, 
               input_tensor_size * sizeof(float));
        
        // 创建ORT tensor
        auto memory_info = Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator, OrtMemTypeDefault);
        
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_tensor_values.data(), input_tensor_size,
            input_shape.data(), input_shape.size());
        
        // 运行推理
        auto output_tensors = session.Run(
            Ort::RunOptions{nullptr},
            input_names.data(), &input_tensor, 1,
            output_names.data(), output_names.size());
        
        auto infer_time = std::chrono::high_resolution_clock::now();
        auto infer_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            infer_time - start_time).count();
        
        // 解析输出
        // scores: [1, H, W]
        // descriptors: [1, 256, H/8, W/8]
        float* scores_data = output_tensors[0].GetTensorMutableData<float>();
        float* desc_data = output_tensors[1].GetTensorMutableData<float>();
        
        auto scores_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        auto desc_shape = output_tensors[1].GetTensorTypeAndShapeInfo().GetShape();
        
        int height = scores_shape[1];
        int width = scores_shape[2];
        int desc_h = desc_shape[2];
        int desc_w = desc_shape[3];
        
        std::cout << "  推理时间: " << infer_duration << "ms" << std::endl;
        std::cout << "  分数图: [" << height << ", " << width << "]" << std::endl;
        std::cout << "  描述符: [256, " << desc_h << ", " << desc_w << "]" << std::endl;
        
        // 转换为 OpenCV Mat 以便进行 NMS
        cv::Mat scores_mat(height, width, CV_32F, scores_data);
        
        // 应用阈值
        cv::Mat threshold_mask = scores_mat > threshold;
        
        // 应用 NMS
        cv::Mat nms_mask;
        if (nms_radius > 0) {
            nms_mask = apply_nms(scores_mat, nms_radius);
        } else {
            nms_mask = cv::Mat::ones(height, width, CV_8U);
        }
        
        // 合并阈值掩码和 NMS 掩码
        cv::Mat final_mask;
        cv::bitwise_and(threshold_mask, nms_mask, final_mask);
        
        // 提取关键点
        Detection result;
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (final_mask.at<uchar>(y, x)) {
                    float score = scores_data[y * width + x];
                    result.keypoints.push_back(cv::Point2f(x, y));
                    result.scores.push_back(score);
                    
                    // 提取对应的描述符
                    int desc_y = y / 8;
                    int desc_x = x / 8;
                    if (desc_y < desc_h && desc_x < desc_w) {
                        std::vector<float> descriptor(256);
                        for (int c = 0; c < 256; c++) {
                            descriptor[c] = desc_data[c * desc_h * desc_w + 
                                                      desc_y * desc_w + desc_x];
                        }
                        result.descriptors.push_back(descriptor);
                    }
                }
            }
        }
        
        std::cout << "  检测到 " << result.keypoints.size() << " 个关键点" << std::endl;
        
        return result;
    }
    
    // 可视化关键点
    cv::Mat visualize(const cv::Mat& image, const Detection& detection, int top_k = 500) {
        cv::Mat vis_image;
        if (image.channels() == 1) {
            cv::cvtColor(image, vis_image, cv::COLOR_GRAY2BGR);
        } else {
            vis_image = image.clone();
        }
        
        // 选择top-k关键点
        std::vector<size_t> indices(detection.scores.size());
        std::iota(indices.begin(), indices.end(), 0);
        
        // 按分数排序
        std::sort(indices.begin(), indices.end(), 
                  [&detection](size_t i1, size_t i2) {
                      return detection.scores[i1] > detection.scores[i2];
                  });
        
        // 绘制
        int count = 0;
        for (size_t idx : indices) {
            if (count++ >= top_k) break;
            
            const auto& pt = detection.keypoints[idx];
            float score = detection.scores[idx];
            
            // 颜色根据分数映射 (绿色->黄色->红色)
            int r = static_cast<int>(255 * score);
            int g = static_cast<int>(255 * (1 - score * 0.5));
            int b = 0;
            
            cv::circle(vis_image, pt, 2, cv::Scalar(b, g, r), -1);
        }
        
        // 添加文字
        std::string text = "Keypoints: " + std::to_string(count);
        cv::putText(vis_image, text, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        
        return vis_image;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "SuperPoint ONNX C++ Inference" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 参数
    std::string model_path = "superpoint.onnx";
    std::string image_path = "IMG_0926.JPG";
    bool use_gpu = true;
    
    if (argc > 1) image_path = argv[1];
    if (argc > 2) use_gpu = (std::string(argv[2]) == "gpu");
    
    std::cout << "\n配置:" << std::endl;
    std::cout << "  模型: " << model_path << std::endl;
    std::cout << "  图像: " << image_path << std::endl;
    std::cout << "  设备: " << (use_gpu ? "GPU" : "CPU") << std::endl;
    
    try {
        // 加载模型
        std::cout << "\n加载模型..." << std::endl;
        SuperPointONNX superpoint(model_path, use_gpu);
        
        // 读取图像
        std::cout << "\n读取图像..." << std::endl;
        cv::Mat image = cv::imread(image_path);
        if (image.empty()) {
            std::cerr << "✗ 无法读取图像: " << image_path << std::endl;
            return -1;
        }
        std::cout << "  ✓ 图像尺寸: " << image.cols << "x" << image.rows << std::endl;
        
        // 缩放图像 (可选)
        int max_dim = 1024;
        float scale = static_cast<float>(max_dim) / std::max(image.cols, image.rows);
        if (scale < 1.0f) {
            cv::Mat resized;
            cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
            image = resized;
            std::cout << "  ✓ 缩放后: " << image.cols << "x" << image.rows << std::endl;
        }
        
        // 推理
        std::cout << "\n推理中..." << std::endl;
        auto detection = superpoint.infer(image, 0.005, 4);
        
        // 可视化
        std::cout << "\n可视化..." << std::endl;
        cv::Mat result = superpoint.visualize(image, detection, 8000);
        
        // 保存结果
        std::string output_path = "superpoint_cpp_result.jpg";
        cv::imwrite(output_path, result);
        std::cout << "  ✓ 保存结果: " << output_path << std::endl;
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "完成！" << std::endl;
        std::cout << "========================================" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "✗ 错误: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
