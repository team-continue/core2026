#include <map>
#include <string>
#include <vector>

namespace core_enemy_detection{
    /*default parameter for target_detector*/
   inline std::map<std::string, std::vector<int>> params = {
        {"enemy", std::vector<int>({0})},
        {"image_size", std::vector<int>({1280, 720})},
        {"red_range_lower1", std::vector<int>({0, 125, 125})},
        {"red_range_upper1", std::vector<int>({10, 255, 255})},
        {"red_range_lower2", std::vector<int>({175, 125, 125})},
        {"red_range_upper2", std::vector<int>({180, 255, 255})},
        // {"red_lab_range_lower", std::vector<int>({230, 0, 105})},
        // {"red_lab_range_upper", std::vector<int>({255, 255, 145})},
        {"blue_range_lower", std::vector<int>({105, 64, 255})},
        {"blue_range_upper", std::vector<int>({135, 255, 255})},
        // {"panel_hsv_range_lower", std::vector<int>({75,  70, 55})},
        // {"panel_hsv_range_upper", std::vector<int>({100, 170, 95})},
        {"panel_lab_range_lower", std::vector<int>({60, 110, 110})},
        {"panel_lab_range_upper", std::vector<int>({75, 140, 140})},
        {"led_kernel_matrix_size", std::vector<int>({5, 5})},
        {"panel_kernel_matrix_size", std::vector<int>({20, 20})}
    };
}