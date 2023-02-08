#include <splined_voronoi/path_smoothing.h>

namespace path_smoothing
{

void mapToWorld(const cv::Point2i& point_map, cv::Point2d& point_world, cv::Point2d map_origin, double resolution)
{
    double half_a_cell = 0.5;
    point_world.x = map_origin.x + (point_map.x + half_a_cell) * resolution;
    point_world.y = map_origin.y + (point_map.y + half_a_cell) * resolution;
}


bool worldToMap(const cv::Point2d& point_world, cv::Point2i& point_map, cv::Point2d map_origin, int map_size_x,
                int map_size_y, double resolution)
{
    if (point_world.x < map_origin.x || point_world.y < map_origin.y)
        return false;

    point_map.x = (int)((point_world.x - map_origin.x) / resolution);
    point_map.y = (int)((point_world.y - map_origin.y) / resolution);

    if (point_map.x < map_size_x && point_map.y < map_size_y)
        return true;

    return false;
}

void pathWorldToMap(const std::vector<cv::Point2d>& path_world_in, std::vector<cv::Point2i>& path_out,
                    std::shared_ptr<costmap_2d::Costmap2D> costmap)
{
    path_out.clear();
    cv::Point2d map_origin(costmap->getOriginX(), costmap->getOriginY());
    int map_size_x = costmap->getSizeInCellsX();
    int map_size_y = costmap->getSizeInCellsY();
    double map_resolution = costmap->getResolution();

    for (auto point : path_world_in)
    {
        cv::Point2i point_map;
        worldToMap(point, point_map, map_origin, map_size_x, map_size_y, map_resolution);
        if (std::find(path_out.begin(), path_out.end(), point_map) == path_out.end())
        {
            path_out.push_back(point_map);
        }
    }
}

template <typename DataType>
void replaceValsInVec(const std::vector<DataType>& points, const std::vector<DataType>& points_orig, const std::vector<int>& indices, std::vector<DataType>& out_points)
{
    out_points.clear();
    int points_idx = 0;
    for (int idx = 0; idx < points_orig.size(); idx++)
    {
        if (std::find(indices.begin(), indices.end(), idx) != indices.end())
        {
            out_points.push_back(points.at(points_idx));
            points_idx++;
        }
        else
        {
            out_points.push_back(points_orig.at(idx));
        }
    }
}

template void replaceValsInVec<cv::Point2d>(const std::vector<cv::Point2d>& points, const std::vector<cv::Point2d>& points_orig, const std::vector<int>& indices, std::vector<cv::Point2d>& out_points);
template void replaceValsInVec<double>(const std::vector<double>& points, const std::vector<double>& points_orig, const std::vector<int>& indices, std::vector<double>& out_points);


std::vector<double> pointsToVec(const std::vector<cv::Point2d>& points)
{
    std::vector<double> vec;
    for (int idx = 0; idx < points.size(); idx++)
    {
        vec.push_back(points.at(idx).x);
        vec.push_back(points.at(idx).y);
    }
    return vec;
}

std::vector<cv::Point2d> vecToPoints(const std::vector<double>& vec)
{
    std::vector<cv::Point2d> points;
    for (int idx = 0; idx < vec.size(); idx += 2)
    {
        points.push_back(cv::Point2d(vec.at(idx), vec.at(idx + 1)));
    }
    return points;
}

std::vector<cv::Point2d> calcConnectionPoints(const std::vector<cv::Point2d>& sparse_path)
{
    // ROS_INFO("Calculating connection points");
    std::vector<cv::Point2d> connection_points;
    cv::Point2d prev_point = sparse_path.at(0);
    double eps = 0.0001;
    for (int idx = 0; idx < sparse_path.size(); idx++)
    {
        cv::Point2d point = sparse_path.at(idx);
        if (std::abs(point.x - prev_point.x) < eps && std::abs(point.y - prev_point.y) < eps)
        {
            continue;
        }
        cv::Point2d connection_point;
        if (idx == 1)
        {
            connection_point = prev_point;
        }
        else if (idx == sparse_path.size() - 1)
        {
            connection_point = point;
        }
        else
        {
            connection_point.x = (prev_point.x + point.x) / 2;
            connection_point.y = (prev_point.y + point.y) / 2;
        }
        connection_points.push_back(connection_point);
        prev_point = point;
    }
    return connection_points;
}

bool calcControlPointsForPath(const std::vector<cv::Point2d>& points, const std::vector<double>& lengths, std::vector<std::vector<cv::Point2d>>& control_points_path, double default_length)
{
    std::vector<cv::Point2d> connection_points = calcConnectionPoints(points); // one less length than points
    std::vector<cv::Point2d> tangents; // of same length as connection_points
    std::vector<double> path_lengths(connection_points.size(), default_length); // default length
    if (!lengths.empty())
    {
        path_lengths = lengths;
    }
    double ratio = 0.5;
    for (int idx = 1; idx < points.size(); idx++)
    {
        cv::Point2d tangent = points.at(idx) - points.at(idx - 1);
        // normalize tangents
        double tangent_length = sqrt(pow(tangent.x, 2) + pow(tangent.y, 2));
        tangent /= tangent_length;
        tangent *= path_lengths.at(idx - 1);
        tangents.push_back(tangent);
    }
    std::vector<double> distances; // shorter by one than connection_points
    for (int idx = 1; idx < connection_points.size(); idx++)
    {
        cv::Point2d diff = connection_points.at(idx) - connection_points.at(idx - 1);
        double distance = sqrt(pow(diff.x, 2) + pow(diff.y, 2));
        distances.push_back(distance);
    }
    std::vector<cv::Point2d> accelerations; // shorter by two than connection_points
    for (int idx = 0; idx < distances.size() - 1; idx++)
    {
        double d_AB = distances.at(idx);
        double d_BC = distances.at(idx + 1);
        double alpha = d_BC / (d_AB + d_BC);
        double beta = d_AB / (d_AB + d_BC);
        cv::Point2d A = connection_points.at(idx);
        cv::Point2d B = connection_points.at(idx + 1);
        cv::Point2d C = connection_points.at(idx + 2);
        cv::Point2d tangent_A = tangents.at(idx);
        cv::Point2d tangent_B = tangents.at(idx + 1);
        cv::Point2d tangent_C = tangents.at(idx + 2);
        cv::Point2d acc = alpha * (6 * A + 2 * tangent_A + 4 * tangent_B - 6 * B) + beta * (-6 * B - 4* tangent_B - 2 * tangent_C + 6* C);
        accelerations.push_back(acc);
    }
    cv::Point2d first_acc(0, 0);
    cv::Point2d last_acc(0, 0);
    if (connection_points.size() > 2)
    {
        first_acc = -6 * connection_points.at(1) - 4 * tangents.at(1) - 2 * tangents.at(2) + 6 * connection_points.at(2);
        last_acc = 6 * connection_points.at(connection_points.size() - 3) + 2 * tangents.at(tangents.size() - 3) + 4 * tangents.at(tangents.size() - 2) - 6 * connection_points.at(connection_points.size() - 2);
    }
    accelerations.insert(accelerations.begin(), first_acc);
    accelerations.push_back(last_acc);
    // now accelerations has same length as connection_points
    for (int idx = 0; idx < connection_points.size() - 1; idx++)
    {
        cv::Point2d p_0 = connection_points.at(idx);
        cv::Point2d p_5 = connection_points.at(idx + 1);
        cv::Point2d p_1 = 0.2 * tangents.at(idx) + p_0; //  * 5.0 * 0.5 * ratio
        cv::Point2d p_4 = p_5 - 0.2 * tangents.at(idx + 1); //   * 5.0 * 0.5 * ratio
        cv::Point2d p_2 = 0.05 * accelerations.at(idx) + 2 * p_1 - p_0;
        cv::Point2d p_3 = 0.05 * accelerations.at(idx + 1) + 2 * p_4 - p_5;
        std::vector<cv::Point2d> control_points = {p_0, p_1, p_2, p_3, p_4, p_5};
        control_points_path.push_back(control_points);
    }
    return true;
}

tinyspline::BSpline buildTinyspline(std::vector<cv::Point2d> control_points)
{
    // ROS_INFO("Building spline");
    tinyspline::BSpline spline(6, 2, 5);
    // Setup control points. They are stored as x0, y0, x1, y1, x2, y2, etc.
    std::vector<tinyspline::real> ctrlp = spline.controlPoints();
    ctrlp[0] = (tsReal)control_points.at(0).x;
    ctrlp[1] = (tsReal)control_points.at(0).y;
    ctrlp[2] = (tsReal)control_points.at(1).x;
    ctrlp[3] = (tsReal)control_points.at(1).y;
    ctrlp[4] = (tsReal)control_points.at(2).x;
    ctrlp[5] = (tsReal)control_points.at(2).y;
    ctrlp[6] = (tsReal)control_points.at(3).x;
    ctrlp[7] = (tsReal)control_points.at(3).y;
    ctrlp[8] = (tsReal)control_points.at(4).x;
    ctrlp[9] = (tsReal)control_points.at(4).y;
    ctrlp[10] = (tsReal)control_points.at(5).x;
    ctrlp[11] = (tsReal)control_points.at(5).y;
    spline.setControlPoints(ctrlp);
    return spline;
}


double calcTinysplineLength(const tinyspline::BSpline& spline)
{
    int num_samples = 100;
    std::vector<tinyspline::real> sampled_points_for_length = spline.sample(num_samples);
    double spline_length = 0.0;
    for (int index_x = 2; index_x < sampled_points_for_length.size() - 1; index_x += 2)
    {
        int index_y = index_x + 1;
        double x_val = sampled_points_for_length.at(index_x);
        double y_val = sampled_points_for_length.at(index_y);
        double last_x_val = sampled_points_for_length.at(index_x - 2);
        double last_y_val = sampled_points_for_length.at(index_y - 2);
        double segdist = sqrt(pow(x_val - last_x_val, 2) + pow(y_val - last_y_val, 2));
        spline_length += segdist;
    }
    return spline_length;
}

void equidistantSampling(const tinyspline::BSpline& spline, double sample_distance, std::vector<cv::Point2d>& samples)
{
    samples.clear();
    double spline_length = calcTinysplineLength(spline);
    tinyspline::Vec2 last_sample = spline(0).resultVec2();
    int num_samples = (int) 10 * spline_length / sample_distance;
    for (double frac = 0.0; frac <= 1; frac+=1.0/(double)num_samples)
    {
        tinyspline::Vec2 sample = spline(frac).resultVec2();
        tinyspline::Vec2 diff = sample - last_sample;
        if (diff.magnitude() > sample_distance)
        {
            double x_val = sample.x();
            double y_val = sample.y();
            samples.push_back(cv::Point2d(x_val, y_val));
            last_sample = sample;
        }
    }
}

void getSplinePathSamples(const std::vector<cv::Point2d>& points, std::vector<double>& lengths, std::vector<cv::Point2d>& points_out, double sample_distance, double default_length)
{
    std::vector<std::vector<cv::Point2d>> control_points_path;
    if (points.size() < 3)
    {
        ROS_ERROR("Not enough points to build spline");
        return;
    }
    calcControlPointsForPath(points, lengths, control_points_path, default_length);

    for (auto control_points: control_points_path)
    {
        tinyspline::BSpline spline = buildTinyspline(control_points);
        std::vector<cv::Point2d> samples_seg;
        equidistantSampling(spline, sample_distance, samples_seg);
        points_out.insert(points_out.end(), samples_seg.begin(), samples_seg.end());
    }
}


cv::Mat buildCostmapImg(const cv::Mat& img)
{
    // ROS_INFO("buildCostmapImg");
    cv::Mat costmap_img;
    cv::distanceTransform(~img, costmap_img, cv::DIST_L2, 3, CV_8UC1);
    // cv::imwrite("/home/rosmatch/Bilder/dists.png", costmap_img);
    cv::Mat combined_costmap_img;
    cv::add(img, 255 - costmap_img, combined_costmap_img, cv::noArray(), CV_8UC1);
    // cv::imwrite("/home/rosmatch/Bilder/costmap_img_dist.png", combined_costmap_img);
    cv::Mat costmap_img_inside_obstacles;
    cv::distanceTransform(img, costmap_img_inside_obstacles, cv::DIST_L2, 3);
    // cv::imwrite("/home/rosmatch/Bilder/dists2.png", costmap_img_inside_obstacles);
    cv::add(costmap_img_inside_obstacles, combined_costmap_img , costmap_img_inside_obstacles, cv::noArray(), CV_32F);
    // costmap_img_inside_obstacles += combined_costmap_img;
    // cv::Mat result;
    // cv::normalize(costmap_img_inside_obstacles, result, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    // cv::imwrite("/home/rosmatch/Bilder/costmap_img_inside_obstacles.png", result);
    return costmap_img_inside_obstacles;
}


std::vector<double> tinysplineCurvatures(const tinyspline::BSpline& spline, int num_samples = 1000)
{
    // ROS_INFO("Calculating curvatures");
    tinyspline::BSpline spline_deriv = spline.derive();
    tinyspline::BSpline spline_deriv2 = spline_deriv.derive();
    std::vector<double> curvatures;
    double eps = 0.0001;
    for (double t = 0; t <= 1; t += 1. / (double)num_samples)
    {
        tinyspline::Vec3 cp = spline_deriv(t).resultVec3();
        tinyspline::Vec3 cpp = spline_deriv2(t).resultVec3();
        double curvature = abs((cp.x() * cpp.y() - cpp.x() * cp.y())/ pow(pow(cp.x(), 2) + pow(cp.y(), 2), 1.5));
        curvatures.push_back(curvature);
    }
    return curvatures;
}


double maxCostOfSpline(const tinyspline::BSpline& spline, const costmap_2d::Costmap2D& costmap, const cv::Mat& costmap_img)
{
    double sample_distance = 0.02;
    std::vector<cv::Point2d> samples;
    equidistantSampling(spline, sample_distance, samples);
    std::vector<cv::Point2i> samples_map;
    pathWorldToMap(samples, samples_map, std::make_shared<costmap_2d::Costmap2D>(costmap));
    double max_cost = 0.0;
    cv::Point2d max_cost_ind;
    for (int idx=0; idx<samples_map.size();idx++)
    // for (auto pixel: samples_map)
    {
        cv::Point2i pixel(samples_map.at(idx));
        cv::Point2d point_world(samples.at(idx));
        float cost = costmap_img.at<float>(pixel.x, pixel.y);
        if (cost > max_cost)
        {
            max_cost = cost;
            max_cost_ind = point_world;
            // ROS_INFO_STREAM("Local maximum at: " << point_world.x << ", " << point_world.y);
        }
    }
    // ROS_INFO_STREAM("Max cost ind: " << max_cost_ind);
    return max_cost;
}

void get_optimize_indices(const std::vector<cv::Point2d>& points, const std::vector<double>& lengths, std::vector<int>& optimize_indices, std::vector<int>& optimize_lengths_indices, const costmap_2d::Costmap2D& costmap, const cv::Mat& costmap_img, double curve_max, bool print_output)
{
    std::vector<std::vector<cv::Point2d>> control_points_path;
    calcControlPointsForPath(points, lengths, control_points_path, -1.0);

    for (int idx = 2; idx < control_points_path.size()-2; idx++)
    {
        std::vector<cv::Point2d> control_points = control_points_path.at(idx);
        tinyspline::BSpline spline = buildTinyspline(control_points);
        std::vector<double> curvatures = tinysplineCurvatures(spline, 200);
        double max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
        double max_cost_seg = maxCostOfSpline(spline, costmap, costmap_img);
        if (max_curvature > curve_max)
        {
            optimize_indices.push_back(idx + 1);
            ROS_INFO_STREAM_COND(print_output, "Segment " << idx + 1 << " needs optimization with curvature " << max_curvature);
        }
        else if (max_cost_seg > 254.0)
        {
            optimize_indices.push_back(idx + 1);
            ROS_INFO_STREAM_COND(print_output, "Segment " << idx + 1 << " needs optimization due to collision.");
        }
        else
        {
            ROS_INFO_STREAM_COND(print_output, "Segment " << idx + 1 << " does not need optimization with curvature " << max_curvature);
            bool idx_in_optimize = std::find(optimize_indices.begin(), optimize_indices.end(), idx) != optimize_indices.end();
            bool idx_already_contained = std::find(optimize_lengths_indices.begin(), optimize_lengths_indices.end(), idx) != optimize_lengths_indices.end();
            bool next_idx_already_contained = std::find(optimize_lengths_indices.begin(), optimize_lengths_indices.end(), idx + 1) != optimize_lengths_indices.end();
            if (!idx_already_contained && !idx_in_optimize)
            {
                optimize_lengths_indices.push_back(idx);
            }
            if (!next_idx_already_contained && !idx_in_optimize)
            {
                optimize_lengths_indices.push_back(idx + 1);
            }
        }
    }
}


double deviationPoints(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    specific_points_optim_data* optim_data = reinterpret_cast<specific_points_optim_data*>(data);

    int begin_lengths_index = optim_data->optimize_indices.size() * 2;
    std::vector<double> lengths;
    if (optim_data->optimize_lengths)
    {
        lengths = std::vector<double>(x.begin() + begin_lengths_index, x.end());
    }
    else
    {
        lengths = std::vector<double>(optim_data->points_orig.size() - 1, optim_data->default_length); // default length
    }
    std::vector<double> points_flattened = std::vector<double>(x.begin(), x.begin() + begin_lengths_index);
    std::vector<cv::Point2d> points = vecToPoints(points_flattened);
    int index = 0;
    double total_deviation = 0.0;
    for (auto point: points)
    {
        cv::Point2d point_orig = optim_data->points_orig.at(optim_data->optimize_indices.at(index));
        double deviation = sqrt(pow(point.x - point_orig.x, 2) + pow(point.y - point_orig.y, 2));
        total_deviation += deviation;
        index++;
    }
    if (!grad.empty()) {
        std::cout << "deviationPoints: grad not empty" << std::endl;
    }
    return total_deviation;
}


double maxCostOfPath(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    std::chrono::steady_clock::time_point start_max_cost_path = std::chrono::steady_clock::now();
    specific_points_optim_data *constraint_data = reinterpret_cast<specific_points_optim_data*>(data);

    int begin_lengths_index = constraint_data->optimize_indices.size() * 2;
    std::vector<double> lengths;
    if (constraint_data->optimize_lengths)
    {
        lengths = std::vector<double>(x.begin() + begin_lengths_index, x.end());
    }
    else
    {
        lengths = std::vector<double>(constraint_data->points_orig.size() - 1, constraint_data->default_length); // default length
    }
    std::vector<double> points_flattened = std::vector<double>(x.begin(), x.begin() + begin_lengths_index);
    std::vector<cv::Point2d> points = vecToPoints(points_flattened);
    std::vector<cv::Point2d> points_orig_replaced;
    replaceValsInVec(points, constraint_data->points_orig, constraint_data->optimize_indices, points_orig_replaced);
    std::vector<cv::Point2d> connection_points = calcConnectionPoints(points_orig_replaced);
    double max_cost = 0.0;
    std::vector<std::vector<cv::Point2d>> control_points_path;
    calcControlPointsForPath(points_orig_replaced, lengths, control_points_path, constraint_data->default_length);

    std::vector<int> check_indices;
    for (auto idx: constraint_data->optimize_indices)
    {
        if (std::find(check_indices.begin(), check_indices.end(), idx - 1) == check_indices.end())
        {
            check_indices.push_back(idx - 1);
        }
        if (std::find(check_indices.begin(), check_indices.end(), idx) == check_indices.end())
        {
            check_indices.push_back(idx);
        }
        if (std::find(check_indices.begin(), check_indices.end(), idx + 1) == check_indices.end())
        {
            check_indices.push_back(idx + 1);
        }
    }

    int curr_idx = 0;
    for (auto control_points: control_points_path)
    {
        if (!constraint_data->optimize_lengths && std::find(check_indices.begin(), check_indices.end(), curr_idx++) == check_indices.end())
        {
            // continue;
        }
        tinyspline::BSpline spline = buildTinyspline(control_points);
        double max_cost_seg = maxCostOfSpline(spline, constraint_data->costmap, constraint_data->costmap_img);
        if (max_cost_seg > max_cost)
        {
            max_cost = max_cost_seg;
        }
    }
    std::chrono::steady_clock::time_point end_max_cost_path = std::chrono::steady_clock::now();
    return max_cost - 254.0;
}


double maxCurvatureFromPoints(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    static int maxCurvatureFromPoints_counter = 0;
    maxCurvatureFromPoints_counter++;
    specific_points_optim_data *constraint_data = reinterpret_cast<specific_points_optim_data*>(data);
    int begin_lengths_index = constraint_data->optimize_indices.size() * 2;
    // std::vector<double> lengths = std::vector<double>(x.begin() + begin_lengths_index, x.end());
    std::vector<double> lengths;
    if (constraint_data->optimize_lengths)
    {
        lengths = std::vector<double>(x.begin() + begin_lengths_index, x.end());
    }
    else
    {
        lengths = std::vector<double>(constraint_data->points_orig.size() - 1, constraint_data->default_length); // default length
    }
    std::vector<double> points_flattened = std::vector<double>(x.begin(), x.begin() + begin_lengths_index);
    std::vector<cv::Point2d> points = vecToPoints(points_flattened);
    std::vector<cv::Point2d> points_orig_replaced;
    replaceValsInVec(points, constraint_data->points_orig, constraint_data->optimize_indices, points_orig_replaced);
    std::vector<cv::Point2d> connection_points = calcConnectionPoints(points_orig_replaced);
    std::vector<std::vector<cv::Point2d>> control_points_path;
    calcControlPointsForPath(points_orig_replaced, lengths, control_points_path, constraint_data->default_length);

    std::vector<int> check_indices;
    for (auto idx: constraint_data->optimize_indices)
    {
        if (std::find(check_indices.begin(), check_indices.end(), idx - 1) == check_indices.end())
        {
            check_indices.push_back(idx - 1);
        }
        if (std::find(check_indices.begin(), check_indices.end(), idx) == check_indices.end())
        {
            check_indices.push_back(idx);
        }
        if (std::find(check_indices.begin(), check_indices.end(), idx + 1) == check_indices.end())
        {
            check_indices.push_back(idx + 1);
        }
    }

    double max_curvature = 0.0;
    int curr_idx = 0;
    for (auto control_points: control_points_path)
    {
        if (!constraint_data->optimize_lengths && std::find(check_indices.begin(), check_indices.end(), curr_idx++) == check_indices.end())
        {
            // continue;
        }
        tinyspline::BSpline spline = buildTinyspline(control_points);
        std::vector<double> curvatures = tinysplineCurvatures(spline, 200);
        double max_curvature_seg = *std::max_element(curvatures.begin(), curvatures.end());
        if (max_curvature_seg > max_curvature)
        {
            max_curvature = max_curvature_seg;
        }
    }
    if (!grad.empty()) {
        std::cout << "maxCurvatureFromPoints: grad not empty" << std::endl;
    }
    return max_curvature - constraint_data->curvature_limit;
}

double combinedMinFunction(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    std::chrono::steady_clock::time_point start_min_func = std::chrono::steady_clock::now();
    specific_points_optim_data *optim_data = reinterpret_cast<specific_points_optim_data*>(data);
    double distance_to_orig = deviationPoints(x, grad, data);
    double max_cost = maxCostOfPath(x, grad, data) + 254.0;
    double max_curvature = maxCurvatureFromPoints(x, grad, data) + optim_data->curvature_limit;
    double curvature_penalty = 0.0;
    double cost_penalty = 0.0;
    if (max_curvature > optim_data->curvature_limit)
    {
        curvature_penalty += 500 + 1000.0 * max_curvature / optim_data->curvature_limit;
    }
    if (max_cost > 254.0)
    {
        cost_penalty += 500 + 1000.0 * max_cost / 254.0;
    }
    if (max_curvature < optim_data->curvature_limit && max_cost < 254.0)
    {
        // found values which fulfil limits; force stop of optimization
        optim_data->was_terminated = true;
        for (auto val: x)
        {
            optim_data->best_result.push_back(val);
        }
        throw nlopt::forced_stop();
    }
    std::chrono::steady_clock::time_point end_min_func = std::chrono::steady_clock::now();
    double total_time_in_optimization = (std::chrono::duration_cast<std::chrono::microseconds>(end_min_func - optim_data->start_time).count()) / 1000000.0;
    if (total_time_in_optimization > optim_data->time_limit)
    {
        optim_data->was_terminated = false;
        throw nlopt::forced_stop();
    }
    // ROS_INFO_STREAM("Optimization combinedMinFunc took (s): " << (std::chrono::duration_cast<std::chrono::microseconds>(end_min_func - start_min_func).count()) / 1000000.0);
    return distance_to_orig + curvature_penalty + cost_penalty;
}

int optimizeSpecificPoints(const std::vector<cv::Point2d>& points_orig, const std::vector<int>& optimize_indices, std::vector<cv::Point2d>& points_out, std::vector<double>& lengths_out, const costmap_2d::Costmap2D& costmap, const cv::Mat& costmap_img, double curve_max, bool optimize_lengths, double default_length, double max_optimization_time)
{
    ROS_INFO("Start optimizing of Points");
    double eps = 0.00001;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    // create additional data
    std::vector<int> adaptive_optimize_indices = std::vector<int>(optimize_indices.begin(), optimize_indices.end());
    std::vector<double> res_vals;
    specific_points_optim_data optim_data{points_orig, adaptive_optimize_indices, costmap, costmap_img, curve_max, max_optimization_time, start_time, optimize_lengths, default_length, false, res_vals};

    int len_opt = optimize_indices.size() * 2;
    if (optimize_lengths)
    {
        len_opt += points_orig.size() - 1;
    }
    nlopt::opt opt(nlopt::LN_NEWUOA, len_opt);
    cv::Point2i map_top_left(costmap.getSizeInCellsX() - 1, costmap.getSizeInCellsY() - 1);
    cv::Point2i map_bottom_right(0,0);
    cv::Point2d world_top_left;
    cv::Point2d world_bottom_right;
    cv::Point2d map_origin(costmap.getOriginX(), costmap.getOriginY());
    double map_resolution = costmap.getResolution();
    mapToWorld(map_top_left, world_top_left, map_origin, map_resolution);
    mapToWorld(map_bottom_right, world_bottom_right, map_origin, map_resolution);
    ROS_INFO_STREAM("lower bound: " << world_bottom_right);
    ROS_INFO_STREAM("upper bound: " << world_top_left);
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    for (int i = 0; i < optimize_indices.size(); i++)
    {
        lower_bounds.push_back(world_bottom_right.x);
        lower_bounds.push_back(world_bottom_right.y);
        upper_bounds.push_back(world_top_left.x);
        upper_bounds.push_back(world_top_left.y);
    }
    if (optimize_lengths)
    {
        for (int i = 0; i < points_orig.size() - 1; i++)
        {
            lower_bounds.push_back(0.0);
            upper_bounds.push_back(10.0);
        }
    }
    opt.set_lower_bounds(lower_bounds);
    opt.set_upper_bounds(upper_bounds);
    opt.set_min_objective(combinedMinFunction, &optim_data);
    std::vector<double> xtols(len_opt, 1e-8);
    opt.set_xtol_abs(xtols);

    std::vector<cv::Point2d> points_to_optim;
    for (auto idx: optimize_indices)
    {
        points_to_optim.push_back(points_orig.at(idx));
    }
    std::vector<double> points_to_optim_flattened = pointsToVec(points_to_optim);
    if (optimize_lengths)
    {
        for (int i = 0; i < points_orig.size() - 1; i++)
        {
            points_to_optim_flattened.push_back(default_length); // default length
        }
    }
    double min_val;
    ROS_INFO("Right before start");

    std::vector<double> grad_vec;
    double deviation_start = deviationPoints(points_to_optim_flattened, grad_vec, &optim_data);
    ROS_INFO_STREAM("Start deviation: " << deviation_start);
    double max_cost_path_start = maxCostOfPath(points_to_optim_flattened, grad_vec, &optim_data) + 254;
    ROS_INFO_STREAM("Start mean cost: " << max_cost_path_start);
    double max_curve_start = maxCurvatureFromPoints(points_to_optim_flattened, grad_vec, &optim_data) + optim_data.curvature_limit;
    ROS_INFO_STREAM("Start max curve: " << max_curve_start);

    ROS_INFO("Preparation for optimization done!");
    bool aborted = false;
    try
    {
        nlopt::result result = opt.optimize(points_to_optim_flattened, min_val);
        ROS_INFO_STREAM("optimization exit code: " << result);
        ROS_INFO_STREAM("found minimum with total deviation: " << std::setprecision(10) << min_val);
    }
    catch(std::exception &err)
    {
        ROS_ERROR_STREAM("nlopt failed: " << err.what());
        if (optim_data.was_terminated)
        {
            ROS_INFO("Optimization was terminated because fitting value was found!");
            points_to_optim_flattened = optim_data.best_result;
        }
        else
        {
            ROS_ERROR("Optimization took too long, aborted");
            aborted = true;
        }
    }

    int begin_lengths_index = optimize_indices.size() * 2;
    if (optimize_lengths)
    {
        lengths_out = std::vector<double>(points_to_optim_flattened.begin() + begin_lengths_index, points_to_optim_flattened.end());

    }
    else
    {
        lengths_out = std::vector<double>(points_orig.size() - 1, default_length); // default length
    }
    std::vector<double> points_flattened = std::vector<double>(points_to_optim_flattened.begin(), points_to_optim_flattened.begin() + begin_lengths_index);
    std::vector<cv::Point2d> points_res = vecToPoints(points_flattened);
    for (auto point: points_res)
    {
        points_out.push_back(point);
    }

    if (aborted)
    {
        return OptimizationStatus::Failure;
    }
    double deviation = deviationPoints(points_to_optim_flattened, grad_vec, &optim_data);
    double max_cost_path = maxCostOfPath(points_to_optim_flattened, grad_vec, &optim_data) + 254;
    double max_curve = maxCurvatureFromPoints(points_to_optim_flattened, grad_vec, &optim_data) + optim_data.curvature_limit;
    ROS_INFO_STREAM("Final deviation: " << deviation);
    ROS_INFO_STREAM("Final max cost: " << max_cost_path);
    ROS_INFO_STREAM("Final max curve: " << max_curve);

    if (max_curve < curve_max)
    {
        return OptimizationStatus::Success;
    }
    return OptimizationStatus::Failure;
}

int optimizeSplinePath(const std::vector<cv::Point2d>& points, std::vector<cv::Point2d>& optimized_points, std::vector<double>& lengths_out, const costmap_2d::Costmap2D& costmap, const cv::Mat& obstacle_img, double curve_max, bool optimize_lengths, double default_length, double max_optimization_time)
{
    cv::Mat costmap_img = buildCostmapImg(obstacle_img);
    std::vector<double> lengths(points.size() - 1, default_length); // default length

    std::vector<int> optimize_indices;
    std::vector<int> optimize_lengths_indices;
    if (points.size() < 4)
    {
        ROS_INFO("No enought points to optimize, returning");
        optimized_points = points;
        lengths_out = lengths;
        return OptimizationStatus::NoOptimizationNeeded;
    }
    get_optimize_indices(points, lengths, optimize_indices, optimize_lengths_indices, costmap, costmap_img, curve_max, true);

    std::cout << "Optimize indices: ";
    for (int i = 0; i < optimize_indices.size(); i++)
        std::cout << optimize_indices.at(i) << " ";
    std::cout << std::endl;

    std::cout << "Optimize length indices: ";
    for (int i = 0; i < optimize_lengths_indices.size(); i++)
        std::cout << optimize_lengths_indices.at(i) << " ";
    std::cout << std::endl;

    double default_length_1 = 0.5;
    double default_length_2 = 0.25;
    std::vector<double> lengths_old;
    for (int i = 0; i < points.size() - 1; i++)
    {
        lengths_old.push_back(default_length_1);
        lengths_old.push_back(default_length_2);
    }
    int optimize_success = OptimizationStatus::Failure;
    if (!optimize_indices.empty())
    {
        // std::vector<cv::Point2d> points_out;
        // calcControlPointsForPath(points_out, optimized_control_points_path);
        std::vector<cv::Point2d> res_points;
        // std::vector<int> optimize_indices_empty;
        bool optimize_points = true;
        if (!optimize_points)
        {
            optimize_indices.clear();
        }
        optimize_success = optimizeSpecificPoints(points, optimize_indices, res_points, lengths_out, costmap, costmap_img, curve_max, optimize_lengths, default_length, max_optimization_time);
        replaceValsInVec(res_points, points, optimize_indices, optimized_points);
        // TODO: check complete path for curvature
        if (optimize_success != OptimizationStatus::Failure)
        {
            std::vector<int> final_optimize_indices;
            std::vector<int> final_optimize_lengths_indices;
            get_optimize_indices(optimized_points, lengths_out, final_optimize_indices, final_optimize_lengths_indices, costmap, costmap_img, curve_max, false);
            if (!final_optimize_indices.empty())
            {
                ROS_ERROR("Optimization failed. Not all Segments fulfil curvature constraint.");
                optimize_success = OptimizationStatus::Failure;
            }
        }
    }
    else
    {
        ROS_INFO("Path doesn't need to be optimized");
        optimized_points = points;
        lengths_out = lengths;
        optimize_success = OptimizationStatus::NoOptimizationNeeded;
    }
    return optimize_success;
}

bool buildOptimizedContinuousPath(const std::vector<cv::Point2d>& sparse_path, std::vector<cv::Point2d>& path_out, std::vector<cv::Point2d>& optimized_sparse_path, std::vector<double>& optimized_lengths,
                                  const costmap_2d::Costmap2D& costmap, const cv::Mat& obstacle_img, double curve_max, double path_resolution, bool optimize_lengths, double max_optimization_time)
{ 
    path_out.clear();
    if (sparse_path.size() < 3)
    {
        for (auto point : sparse_path)
        {
            path_out.push_back(point);
        }
        ROS_WARN("Less than three points received, possibly only start and goal, no smoothing performed");
        return true;
    }

    double default_length = 1.0;    
    int optimize_success = optimizeSplinePath(sparse_path, optimized_sparse_path, optimized_lengths, costmap, obstacle_img, curve_max, optimize_lengths, default_length, max_optimization_time);

    if (optimize_success >= 0)
    {
        getSplinePathSamples(optimized_sparse_path, optimized_lengths, path_out, 1.0 / path_resolution, default_length);
    }
    return optimize_success >= 0;
}


}  // namespace spline_interpolation
