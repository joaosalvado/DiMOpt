#include "FreeSpace.hpp"

using namespace mropt::freespace;

FreeSpace::~FreeSpace() {
    if (poly_centers) delete poly_centers;
}

int FreeSpace::Polygon::counter = 0;
std::vector<FreeSpace::Polygon> FreeSpace::polygons = std::vector<FreeSpace::Polygon>();
std::unordered_map<
        std::pair<int, int>,
        mropt::StateSpace::State::state,
        boost::hash<std::pair<int, int>>> *
        FreeSpace::poly_centers = nullptr;


void FreeSpace::add_polygon(MX &A, MX &b) {
    polygons.push_back(Polygon{A, b});
}

void FreeSpace::add(std::list<Polygon> polys) {
    std::copy(polys.begin(), polys.end(), std::back_inserter(polygons));
}


std::vector<MX> FreeSpace::get_constraints(std::vector<PolygonAssignment> pas) {
    double safe_radius = robot_shape->get_safety_radius();
    MX zero{0.0};
    MX g_sum{0.0};
    std::vector<MX> constraints{};
    const auto &xy = ss.xy();
    for (const auto &pa: pas) {
        auto &polygon = polygons[pa.pid];
        for (int k = pa.k0; k < pa.kf; ++k) {
            auto g_p = mtimes(polygon.A, xy(all, k)) - polygon.b + safe_radius + threshold;
            constraints.push_back(g_p);
            for (int g_id = 0; g_id < g_p.size1(); ++g_id) {
                g_sum = g_sum + MX::mmax(MX::vertcat({g_p(g_id), zero}));
            }
        }
    }
    J_real_ = Function("J_real", {ss.X()}, {g_sum});
    return constraints;
}

void FreeSpace::clear_polygons() {
    FreeSpace::Polygon::counter = 0;
    FreeSpace::polygons = std::vector<FreeSpace::Polygon>();
    delete FreeSpace::poly_centers;
    FreeSpace::poly_centers = nullptr;
}


void FreeSpace::init_cfree(std::vector<std::vector<std::vector<double>>> polygons) {
    Slice all;
    for (auto &polygon: polygons) {
        int Hp = polygon.size(); // amount of halfplanes in polygon
        MX A = MX(Hp, 2);
        MX b = MX(Hp, 1);
        for (int hp_id = 0; hp_id < Hp; ++hp_id) {
            auto halfplane = polygon[hp_id];
            auto a1 = halfplane[0];
            auto a2 = halfplane[1];
            auto b1 = halfplane[2];
            A(hp_id, all) = DM({a1, a2});
            b(hp_id) = DM({b1});
        }
        mropt::freespace::FreeSpace::Polygon P{A, b};
        mropt::freespace::FreeSpace::add({P});
    }
}

std::vector<mropt::freespace::FreeSpace::PolygonAssignment>
FreeSpace::stay_always_in_polygon(int N, int p_id_) {
    std::vector<mropt::freespace::FreeSpace::PolygonAssignment> pas_robot;
    auto k0 = 0;
    auto kf = N;
    auto p_id = p_id_;
    mropt::freespace::FreeSpace::PolygonAssignment pa{k0, kf, p_id};
    pas_robot.emplace_back(pa);
    return pas_robot;
}