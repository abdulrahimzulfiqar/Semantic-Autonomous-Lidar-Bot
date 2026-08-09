#ifndef _VISUALIZATION_H_
#define _VISUALIZATION_H_
#include "pathgraph.h"
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

using namespace osm_ag;

#define IMAGE_W 2000
#define IMAGE_H 1200

/**
*@brief Plot all nodes(points) in an image to check if the coordinate is correct.
*@param resolution The default is 0.1, is meter per pixel (m/pixel)
*@param parameter-name description
*/
void PlotNodePoints(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    // image的尺寸宏定义，需要用来算分辨率,需要根据规模进行缩放 TODO
    for(auto node_it = graph.nodes_.begin(); node_it!=graph.nodes_.end(); node_it++){
        int x = node_it->second->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = node_it->second->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        cv::circle(image, points_2d, 2, cv::Scalar(255,0,0));
    }
    // cv::imwrite("show.png",image);
}

void PlotArea(AreaGraph& graph, cv::Mat& image, AreaId id, double resolution = 0.1){
    cv::Point2i points_2d_last;
    int i = 0;
    for(auto node_id : graph.areas_[id]->nodes_inorder_){
        int x = graph.nodes_[node_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = graph.nodes_[node_id]->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            cv::line(image, points_2d_last, points_2d, cv::Scalar(0,255,0));
        }
        points_2d_last = points_2d;
        i++;
    }
    // cv::imwrite("show_area.png",image);
}

void PlotAreas(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    for(auto area_it = graph.areas_.begin(); area_it!=graph.areas_.end(); area_it++){
        PlotArea(graph, image, area_it->first, resolution);
    }
    cv::imwrite("show_areas.png",image);
}

void PlotPassage(AreaGraph& graph, cv::Mat& image, PassageId id, double resolution = 0.1){
    auto source_id = graph.passages_[id]->passage_nodes.source;
    auto target_id = graph.passages_[id]->passage_nodes.target;

    int x_s = graph.nodes_[source_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
    int y_s = graph.nodes_[source_id]->attributes_->position[1]/resolution + IMAGE_H/4;
    cv::Point2i points_s(x_s,y_s);

    int x_t = graph.nodes_[target_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
    int y_t = graph.nodes_[target_id]->attributes_->position[1]/resolution + IMAGE_H/4;
    cv::Point2i points_t(x_t,y_t);

    cv::line(image, points_s, points_t, cv::Scalar(0,0,255), 1.5);
}

void PlotPassages(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    for(auto passage_it = graph.passages_.begin(); passage_it!=graph.passages_.end(); passage_it++){
        PlotPassage(graph, image, passage_it->first, resolution);
    }
    // cv::imwrite("show_passages.png",image);
}


void PlotPath(std::vector<Eigen::Vector3d> path, cv::Mat& image, double resolution = 0.1){
    if(path.empty()) return;
    cv::Point2i points_2d_last;
    int i = 0;
    for(auto pt:path){
        int x = pt[0]/resolution + 9 * IMAGE_W/10;
        int y = pt[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            cv::line(image, points_2d_last, points_2d, cv::Scalar(0,0,255), 4); // Bright Red thick line
        }
        points_2d_last = points_2d;
        i++;
    }
    // Draw Start (Blue) and End (Red) Waypoint Circles
    int start_x = path.front()[0]/resolution + 9 * IMAGE_W/10;
    int start_y = path.front()[1]/resolution + IMAGE_H/4;
    cv::circle(image, cv::Point2i(start_x, start_y), 6, cv::Scalar(255, 0, 0), cv::FILLED); // Blue Start

    int end_x = path.back()[0]/resolution + 9 * IMAGE_W/10;
    int end_y = path.back()[1]/resolution + IMAGE_H/4;
    cv::circle(image, cv::Point2i(end_x, end_y), 6, cv::Scalar(0, 0, 255), cv::FILLED); // Red Goal
}

/**
 * @brief Auto-scaled visualization of the global planned path overlaid on the full floorplan.
 *
 * Unlike PlotPath(), this function dynamically computes the bounding box of ALL graph nodes
 * and ALL trajectory waypoints, then creates a properly-sized image where everything is visible.
 * This fixes the bug where the hardcoded IMAGE_H/4=300 Y-offset in PlotPath() caused path
 * waypoints with large negative Y values to render above the canvas (negative pixel coordinates).
 *
 * @param graph     The AreaGraph containing all nodes and passages
 * @param path      The planned trajectory waypoints (from PlanInPathGraph)
 * @param filename  Output PNG filename
 * @param resolution Meters per pixel (default 0.1)
 */
inline void PlotGlobalPath(AreaGraph& graph, std::vector<Eigen::Vector3d>& path,
                           const std::string& filename, double resolution = 0.1) {
    if (path.empty()) {
        printf("PlotGlobalPath: path is empty, nothing to plot.\n");
        return;
    }

    // Step 1: Compute bounding box of ALL nodes + ALL path waypoints
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (auto& np : graph.nodes_) {
        double x = np.second->attributes_->position[0];
        double y = np.second->attributes_->position[1];
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }
    for (auto& pt : path) {
        if (pt[0] < min_x) min_x = pt[0];
        if (pt[0] > max_x) max_x = pt[0];
        if (pt[1] < min_y) min_y = pt[1];
        if (pt[1] > max_y) max_y = pt[1];
    }

    // Step 2: Compute image size with 50-pixel padding on all sides
    int pad = 50;
    int img_w = (int)((max_x - min_x) / resolution) + 2 * pad;
    int img_h = (int)((max_y - min_y) / resolution) + 2 * pad;
    if (img_w < 400) img_w = 400;
    if (img_h < 400) img_h = 400;

    // Offset: world_to_pixel = (world - min) / resolution + pad
    double off_x = -min_x / resolution + pad;
    double off_y = -min_y / resolution + pad;

    printf("PlotGlobalPath: image %dx%d, world X[%.1f,%.1f] Y[%.1f,%.1f], offset(%.0f,%.0f)\n",
           img_w, img_h, min_x, max_x, min_y, max_y, off_x, off_y);

    cv::Mat canvas(cv::Size(img_w, img_h), CV_8UC3, cv::Scalar(255, 255, 255));

    // Step 3: Draw all area boundaries (thin gray lines)
    for (auto& area_it : graph.areas_) {
        cv::Point2i last_pt;
        int i = 0;
        for (auto node_id : area_it.second->nodes_inorder_) {
            int px = (int)(graph.nodes_[node_id]->attributes_->position[0] / resolution + off_x);
            int py = (int)(graph.nodes_[node_id]->attributes_->position[1] / resolution + off_y);
            cv::Point2i pt(px, py);
            if (i > 0) {
                cv::line(canvas, last_pt, pt, cv::Scalar(200, 200, 200), 1); // Light gray walls
            }
            last_pt = pt;
            i++;
        }
    }

    // Step 4: Draw all passage doorways (thin blue lines)
    for (auto& passage_it : graph.passages_) {
        auto src = passage_it.second->passage_nodes.source;
        auto tgt = passage_it.second->passage_nodes.target;
        int sx = (int)(graph.nodes_[src]->attributes_->position[0] / resolution + off_x);
        int sy = (int)(graph.nodes_[src]->attributes_->position[1] / resolution + off_y);
        int tx = (int)(graph.nodes_[tgt]->attributes_->position[0] / resolution + off_x);
        int ty = (int)(graph.nodes_[tgt]->attributes_->position[1] / resolution + off_y);
        cv::line(canvas, cv::Point2i(sx, sy), cv::Point2i(tx, ty), cv::Scalar(180, 180, 180), 1);
    }

    // Step 5: Draw the planned path trajectory (THICK MAGENTA line)
    cv::Point2i last_pt;
    for (size_t i = 0; i < path.size(); i++) {
        int px = (int)(path[i][0] / resolution + off_x);
        int py = (int)(path[i][1] / resolution + off_y);
        cv::Point2i pt(px, py);
        if (i > 0) {
            cv::line(canvas, last_pt, pt, cv::Scalar(255, 0, 255), 4); // Magenta, thickness 4
        }
        last_pt = pt;
    }

    // Step 6: Draw Start (green filled circle) and Goal (red filled circle) markers
    {
        int sx = (int)(path.front()[0] / resolution + off_x);
        int sy = (int)(path.front()[1] / resolution + off_y);
        cv::circle(canvas, cv::Point2i(sx, sy), 10, cv::Scalar(0, 200, 0), cv::FILLED);
        cv::putText(canvas, "START", cv::Point2i(sx + 14, sy + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 150, 0), 2);

        int gx = (int)(path.back()[0] / resolution + off_x);
        int gy = (int)(path.back()[1] / resolution + off_y);
        cv::circle(canvas, cv::Point2i(gx, gy), 10, cv::Scalar(0, 0, 255), cv::FILLED);
        cv::putText(canvas, "GOAL", cv::Point2i(gx + 14, gy + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 200), 2);
    }

    cv::imwrite(filename, canvas);
    printf("PlotGlobalPath: saved to %s\n", filename.c_str());
}


void PlotAreaInGrid(AreaGraph& graph, cv::Mat& image, AreaId id, double resolution = 0.1){
    std::vector<std::vector<cv::Point2i>> contour_pts(1,std::vector<cv::Point2i> ());
    cv::Point2i points_2d_last;
    int i = 0;
    for(auto node_id : graph.areas_[id]->nodes_inorder_){
        int x = graph.nodes_[node_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = graph.nodes_[node_id]->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            contour_pts[0].push_back(points_2d);
            cv::line(image, points_2d_last, points_2d, cv::Scalar(0));
        }
        points_2d_last = points_2d;
        i++;
    }
    cv::drawContours(image,contour_pts,-1,cv::Scalar(255),cv::FILLED);//空闲区域是白色的

    i=0;
    NodeId node_id_last;
    for(auto node_id : graph.areas_[id]->nodes_inorder_){
        int x = graph.nodes_[node_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = graph.nodes_[node_id]->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            std::pair<NodeId,NodeId> p(node_id,node_id_last);
            std::pair<NodeId,NodeId> p_i(node_id_last,node_id);
            if( !graph.areas_[id]->passage_nodes.count(p) && !graph.areas_[id]->passage_nodes.count(p_i)){ //not belong the passage
                cv::line(image, points_2d_last, points_2d, cv::Scalar(0), 1);//TODO: DEBUG!!!!有的passage被错误地添加为了墙壁
            }
        }
            points_2d_last = points_2d;
            node_id_last = node_id;
            i++;
        }

    // cv::imwrite("show_area.png",image);
}

void PlotHighAreaInGrid(AreaGraph& graph, cv::Mat& image, AreaId id, double resolution = 0.1){
    std::vector<std::vector<cv::Point2i>> contour_pts(1,std::vector<cv::Point2i> ());
    cv::Point2i points_2d_last;
    int i = 0;
    NodeId node_id_last;
    for(auto node_id : graph.areas_[id]->nodes_inorder_){
        int x = graph.nodes_[node_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = graph.nodes_[node_id]->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            std::pair<NodeId,NodeId> p(node_id,node_id_last);
            std::pair<NodeId,NodeId> p_i(node_id_last,node_id);
            if( !graph.areas_[id]->passage_nodes.count(p) && !graph.areas_[id]->passage_nodes.count(p_i)){ //not belong the passage
                cv::line(image, points_2d_last, points_2d, cv::Scalar(0), 1);//TODO: DEBUG!!!!有的passage被错误地添加为了墙壁
            }
        }
            points_2d_last = points_2d;
            node_id_last = node_id;
            i++;
        }

    // cv::imwrite("show_area.png",image);
}

void PlotAreasInGrid(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    for(auto area_it = graph.areas_.begin(); area_it!=graph.areas_.end(); area_it++){
        if(area_it->second->is_leaf){//前提：：：在pathgraph那里进行叶子的判断！！！需要改进
            PlotAreaInGrid(graph, image, area_it->first, resolution);
        }else{
            PlotHighAreaInGrid(graph, image, area_it->first, resolution);
        }
    }
    cv::imwrite("show_areas.png",image);
}

void PlotPassageInGrid(AreaGraph& graph, cv::Mat& image, PassageId id, double resolution = 0.1){
    auto source_id = graph.passages_[id]->passage_nodes.source;
    auto target_id = graph.passages_[id]->passage_nodes.target;

    int x_s = graph.nodes_[source_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
    int y_s = graph.nodes_[source_id]->attributes_->position[1]/resolution + IMAGE_H/4;
    cv::Point2i points_s(x_s,y_s);

    int x_t = graph.nodes_[target_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
    int y_t = graph.nodes_[target_id]->attributes_->position[1]/resolution + IMAGE_H/4;
    cv::Point2i points_t(x_t,y_t);

    int x_m = (x_s + x_t)/2 ;
    int y_m = (y_s + y_t)/2 ;
    cv::Point2i points_m(x_m,y_m);

    cv::line(image, points_s, points_t, cv::Scalar(255), 1);
    // cv::circle(image,points_m,3,cv::Scalar(255),cv::FILLED);
}

void PlotPassagesInGrid(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    for(auto passage_it = graph.passages_.begin(); passage_it!=graph.passages_.end(); passage_it++){
        PlotPassageInGrid(graph, image, passage_it->first, resolution);
    }
    // cv::imwrite("show_passages.png",image);
}

cv::Point2i XYZ2Grid(Eigen::Vector3d point, double resolution = 0.1){
    int x_grid = point[0]/resolution + 9 * IMAGE_W/10;
    int y_grid = point[1]/resolution +  IMAGE_H/4;
    cv::Point2i result(x_grid,y_grid);
    return result;
}

Eigen::Vector3d Grid2XYZ_vis(cv::Point pt_grid, double resolution = 0.1){
    Eigen::Vector3d result;
    result[0] = (pt_grid.x - 9 * IMAGE_W/10) * resolution;
    result[1] = (pt_grid.y - IMAGE_H/4) * resolution;
    result[2] = 0;
    return result;
}
#endif