#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate <<
        1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float r = rotation_angle * MY_PI / 180.0f;

    model <<
        /*std::cos(r), -std::sin(r), 0, 0,
         std::sin(r), std::cos(r), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;*/
        1, 0, 0, 0,
        0, std::cos(r), -std::sin(r), 0,
        0, std::sin(r), std::cos(r), 0,
        0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float t_over_n = std::tan(eye_fov * 0.5 * MY_PI / 180.0f);
    float t = t_over_n * std::abs(zNear);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;
    float n = zNear;
    float f = zFar;

    Eigen::Matrix4f p_to_o = Eigen::Matrix4f::Zero();
    p_to_o << 
        n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, (n + f), -n * f,
        0, 0, 1, 0;

    Eigen::Matrix4f ortho1 = Eigen::Matrix4f::Zero();
    ortho1 <<
        1, 0, 0, -(r + l) * 0.5,
        0, 1, 0, -(t + b) * 0.5,
        0, 0, 1, -(n + f) * 0.5,
        0, 0, 0, 1;

    Eigen::Matrix4f ortho2 = Eigen::Matrix4f::Zero();
    ortho2 <<
        2.0 / (r - l), 0, 0, 0,
        0, 2.0 / (t - b), 0, 0,
        0, 0, 2.0 / (n - f), 0,
        0, 0, 0, 1;


    projection = ortho2 * ortho1 * p_to_o;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);
    
    float R = 30.0f;
    float theta = 0.0f;
    float phi = 0.0f;
    Eigen::Vector3f eye_pos = { R * std::sin(theta) * std::cos(phi),R * std::sin(theta) * std::sin(phi),R * std::cosf(theta) };
   

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'j') {
            theta += 0.1f;
            if (theta > MY_PI) {
                theta = 0;
            }
        }
        else if (key == 'k') {
            theta -= 0.1f;
            if (theta < 0) {
                theta = MY_PI;
            }
        }
        else if (key == 'u') {
            phi += 0.1f;
            if (phi > 2 * MY_PI) {
                phi = 0;
            }
        }
        else if (key == 'i') {
            phi -= 0.1f;
            if (phi < 0) {
                phi = 2*MY_PI;
            }
        }
        eye_pos = { R * std::sin(theta) * std::cos(phi),R * std::sin(theta) * std::sin(phi),R * std::cos(theta) };
    }

    return 0;
}
