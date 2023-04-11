//
// Created by zartris on 4/10/23.
//

#ifndef BHS_GAME_IMAGE_UTILS_H
#define BHS_GAME_IMAGE_UTILS_H

#include "Eigen/Core"

#define STB_IMAGE_IMPLEMENTATION

#include "stb_image.hpp"

using ImageVector = std::vector<std::vector<std::vector<int>>>;

class ImageUtils {
public:
    static ImageVector loadImageToVector(const std::string &file_name, bool keep_alpha = false) {
        int width, height, channels;
        unsigned char *data = stbi_load(file_name.c_str(), &width, &height, &channels, 0);

        if (data == nullptr) {
            std::cerr << "Failed to load image: " << file_name << std::endl;
            exit(1);
        }
        auto channel_size = channels < 4 or keep_alpha ? channels : channels - 1;
        ImageVector image(height, std::vector<std::vector<int>>(width, std::vector<int>(channel_size)));

        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                for (int k = 0; k < channels; ++k) {
                    int idx = i * width * channels + j * channels + k;
                    int pixel_value = static_cast<int>(data[idx]);
//                    float pixel_value = static_cast<float>(data[idx]) / 255.0f;
                    if (keep_alpha or channels < 4) {
                        image[i][j][k] = pixel_value;
                    } else {
                        if (k < channels - 1) {
                            image[i][j][k] = pixel_value;
                        }
                    }
                }
            }
        }

        stbi_image_free(data);
        return image;
    }
};


#endif //BHS_GAME_IMAGE_UTILS_H
