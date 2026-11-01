// Offline parser contract check. No ROS initialization and no robot commands.
#include "osmAG/data_load_save.h"
#include <iostream>
#include <set>
#include <string>
#include <vector>

int main(int argc, char **argv) {
    if (argc < 2 || argc > 3 || (argc == 3 && std::string(argv[2]) != "--expect-root-excluded")) {
        std::cerr << "Usage: osmag_map_check map.osm [--expect-root-excluded]\n"; return 2;
    }
    const bool expect_root_excluded = argc == 3;
    osm_ag::AreaGraph graph;
    if (!Parsing_Osm2AreaGraph(graph, argv[1])) return 1;
    std::set<osm_ag::AreaId> leaf;
    for (const auto &entry : graph.areas_) {
        const auto &area = *entry.second;
        if (area.type != "structure") leaf.insert(entry.first);
        if (area.nodes_inorder_.size() < 4 || area.nodes_inorder_.front() != area.nodes_inorder_.back()) return 1;
        std::vector<cv::Point2f> polygon;
        for (auto id : area.nodes_inorder_) {
            if (!graph.nodes_.count(id)) return 1;
            const auto point = graph.nodes_.at(id)->getAttributesPtr()->position;
            polygon.emplace_back(point[0],point[1]);
        }
        // The GCU root is in its central void; no traversable area may cover it.
        if (expect_root_excluded && area.type != "structure" && cv::pointPolygonTest(polygon, cv::Point2f(0,0), false) >= 0) {
            std::cerr << "GCU root exclusion lost in area " << entry.first << '\n'; return 1;
        }
    }
    std::map<osm_ag::AreaId,std::set<osm_ag::AreaId>> adjacent;
    for (const auto &entry : graph.passages_) {
        const auto &passage = *entry.second;
        if (!leaf.count(passage.area_from) || !leaf.count(passage.area_to)) return 1;
        for (auto id : {passage.area_from,passage.area_to}) {
            const auto &ring = graph.areas_.at(id)->nodes_inorder_;
            bool found = false;
            for (size_t i=1;i<ring.size();++i) {
                const auto a=ring[i-1], b=ring[i], x=passage.passage_nodes.source, y=passage.passage_nodes.target;
                if ((a==x && b==y)||(a==y && b==x)) found=true;
            }
            if (!found) { std::cerr << "Passage endpoints are not a shared boundary edge\n"; return 1; }
        }
        adjacent[passage.area_from].insert(passage.area_to);
        adjacent[passage.area_to].insert(passage.area_from);
    }
    if (leaf.empty()) return 1;
    std::set<osm_ag::AreaId> reached{*leaf.begin()};
    std::vector<osm_ag::AreaId> pending{*leaf.begin()};
    while (!pending.empty()) {
        const auto id=pending.back();pending.pop_back();
        for (const auto other : adjacent[id]) if (reached.insert(other).second) pending.push_back(other);
    }
    if (reached != leaf) { std::cerr << "Disconnected traversable graph\n";return 1; }
    std::cout << "PASS: " << graph.areas_.size() << " areas (" << leaf.size()
              << " traversable sectors), " << graph.passages_.size() << " passages\n";
    return 0;
}
