#include <map>
#include <string>
#include <vector>

namespace core_enemy_detection{
    /*default parameter for target_detector*/
   inline std::map<std::string, std::vector<int>> params = {
        {"red_range_lower1", std::vector<int>({0, 125, 125})},
        {"red_range_upper1", std::vector<int>({10, 254, 254})},
        {"red_range_lower2", std::vector<int>({175, 125, 125})},
        {"red_range_upper2", std::vector<int>({180, 254, 254})},
        {"blue_range_lower", std::vector<int>({105, 64, 255})},
        {"blue_range_upper", std::vector<int>({135, 254, 254})},
        {"panel_range_lower", std::vector<int>({60, 110, 110})},
        {"panel_range_upper", std::vector<int>({75, 140, 140})},
        {"led_kernel_matrix_size", std::vector<int>({3, 3})},
        {"panel_kernel_matrix_size", std::vector<int>({3, 3})}
    };

    typedef struct labelImage{
        int num = 0;
        cv::Mat image;
        cv::Mat labelMap;
        cv::Mat status;
        cv::Mat centroids;
    }labeledImage;
}