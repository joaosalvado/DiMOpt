#include "FreeSpace.hpp"  

using namespace mropt::freespace;

FreeSpace::~FreeSpace()
{
	if(poly_centers) delete poly_centers;
}

int FreeSpace::Polygon::counter = 0;
std::vector<FreeSpace::Polygon> FreeSpace::polygons = std::vector<FreeSpace::Polygon>();
std::unordered_map<
    std::pair<int, int>,
        mropt::StateSpace::State::state,
        boost::hash<std::pair<int, int>>>*
FreeSpace::poly_centers = nullptr;


void FreeSpace::add_polygon(MX &A, MX &b)
{
  polygons.push_back(Polygon{A, b});
}

void FreeSpace::add(std::list<Polygon> polys)
{
  std::copy(polys.begin(), polys.end(), std::back_inserter(polygons));
}


std::vector<MX> FreeSpace::get_constraints(std::vector<PolygonAssignment> pas)
{
  double safe_radius = robot_shape->get_safety_radius();
  MX zero{0.0};
  MX g_sum{0.0};
  std::vector<MX> constraints{};
  const auto &xy = ss.xy();
  for (const auto &pa : pas)
  {
    auto &polygon = polygons[pa.pid];
    for(int k = pa.k0; k < pa.kf; ++k){
      auto g_p = mtimes(polygon.A, xy(all, k)) - polygon.b + safe_radius + threshold;
      constraints.push_back( g_p );
      for (int g_id = 0; g_id < g_p.size1(); ++g_id)
      {
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