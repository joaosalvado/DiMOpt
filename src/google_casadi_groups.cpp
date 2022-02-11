//
// Created by ohmy on 2021-09-15.
//
#include <casadi/casadi.hpp>
#include <chrono>
#include <thread>
#include <future>
#include <mpi.h>
using namespace casadi;
using namespace std::chrono;


void callFuncLoop(Function & func, const DM &x_0, int sleep_time){
  std::cout << "Calling func: " << func.name() << std::endl;
  int call_count = 0;
  while(true) {
    auto result = func(std::vector<casadi::DM>{x_0})[0];
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    call_count++;
    if(call_count%5) std::cout << func.name() << " - " << call_count << " calls" << std::endl;
  }
}

int main(int argc, char** argv){
  MPI_Init( &argc, &argv );
  // Reading size and rank
  int process_id, size;
  MPI_Comm_size(MPI_COMM_WORLD, &size);
  MPI_Comm_rank(MPI_COMM_WORLD, &process_id);
// Create casadi function objects
  auto x = SX::sym("x", 2,1);
  auto f = Function("f", {x}, {x(0) + x(1)});
  auto g = Function("g", {x}, {x(0)});


  if(process_id ==  0) {
    auto x_0 = DM({0,1});
    callFuncLoop(f, x_0, 5);
  }
  if(process_id == 1) {
    auto x_0 = DM({0,1});
    callFuncLoop(g, x_0, 5);
  }

  // Finalisation
  MPI_Finalize();

}


//int main(int argc, char** argv){
//
//// Create casadi function objects
//  auto x = SX::sym("x", 2,1);
//  auto f = Function("f", {x}, {x(0) + x(1)});
//  auto g = Function("g", {x}, {x(0)});
//
//
//  auto x_0 = DM({0,1});
//  auto t1 = high_resolution_clock::now();
////  callFunc(f, x_0, 200);
////  callFunc(g, x_0,  200);
//  auto t2 = high_resolution_clock::now();
//  duration<double, std::milli> delta_t = t2 - t1;
////  std::cout << "Execution Time: " << delta_t.count() << " (ms)" << std::endl;
//
//  t1 = high_resolution_clock::now();
//  auto fut_thread_f = std::async(std::launch::async,
//                                 [&]{ return callFuncLoop(f, x_0, 5);});
//  auto fut_thread_g = std::async(std::launch::async,
//                                 [&]{ return callFuncLoop(g, x_0, 5);});
//
//  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//  fut_thread_f.get();
//  fut_thread_g.wait();
//  t2 = high_resolution_clock::now();
//  delta_t = t2 - t1;
//  std::cout << "Execution Time: " << delta_t.count() << " (ms)" << std::endl;
//
//}
//
