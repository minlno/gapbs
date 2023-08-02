// Copyright (c) 2015, The Regents of the University of California (Regents)
// See LICENSE.txt for license details

#ifndef BENCHMARK_H_
#define BENCHMARK_H_

#include <algorithm>
#include <cinttypes>
#include <functional>
#include <random>
#include <utility>
#include <vector>
//mhkim
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "builder.h"
#include "graph.h"
#include "timer.h"
#include "util.h"
#include "writer.h"


/*
GAP Benchmark Suite
File:   Benchmark
Author: Scott Beamer

Various helper functions to ease writing of kernels
*/


// Default type signatures for commonly used types
typedef int32_t NodeID;
typedef int32_t WeightT;
typedef NodeWeight<NodeID, WeightT> WNode;

typedef CSRGraph<NodeID> Graph;
typedef CSRGraph<NodeID, WNode> WGraph;

typedef BuilderBase<NodeID, NodeID, WeightT> Builder;
typedef BuilderBase<NodeID, WNode, WeightT> WeightedBuilder;

typedef WriterBase<NodeID, NodeID> Writer;
typedef WriterBase<NodeID, WNode> WeightedWriter;


// Used to pick random non-zero degree starting points for search algorithms
template<typename GraphT_>
class SourcePicker {
 public:
  explicit SourcePicker(const GraphT_ &g, NodeID given_source = -1)
      : given_source(given_source), rng(kRandSeed), udist(0, g.num_nodes()-1),
        g_(g) {}

  NodeID PickNext() {
    if (given_source != -1)
      return given_source;
    NodeID source;
    do {
      source = udist(rng);
    } while (g_.out_degree(source) == 0);
    return source;
  }

 private:
  NodeID given_source;
  std::mt19937 rng;
  std::uniform_int_distribution<NodeID> udist;
  const GraphT_ &g_;
};


// Returns k pairs with largest values from list of key-value pairs
template<typename KeyT, typename ValT>
std::vector<std::pair<ValT, KeyT>> TopK(
    const std::vector<std::pair<KeyT, ValT>> &to_sort, size_t k) {
  std::vector<std::pair<ValT, KeyT>> top_k;
  ValT min_so_far = 0;
  for (auto kvp : to_sort) {
    if ((top_k.size() < k) || (kvp.second > min_so_far)) {
      top_k.push_back(std::make_pair(kvp.second, kvp.first));
      std::sort(top_k.begin(), top_k.end(),
                std::greater<std::pair<ValT, KeyT>>());
      if (top_k.size() > k)
        top_k.resize(k);
      min_so_far = top_k.back().first;
    }
  }
  return top_k;
}


bool VerifyUnimplemented(...) {
  std::cout << "** verify unimplemented **" << std::endl;
  return false;
}

int CreateSocket(int &socket_fd, int &opt, 
		struct sockaddr_in &address, const int &port) {
  if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
    std::cerr << "Socket creation error" << std::endl; 
	return -1;
  }

  if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
    std::cerr << "Setsockopt error" << std::endl;
	return -1;
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  if (bind(socket_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    std::cerr << "Bind failed" << std::endl;
	return -1;
  }

  if (listen(socket_fd, 3) < 0) {
    std::cerr << "Listen error" << std::endl;
	return -1;
  }

  return 0;
}

int GetNumTrials(const int &socket_fd, const struct sockaddr_in &address) {
  int num_trials = 0;
  int new_socket;
  int addrlen = sizeof(address);
  char buffer[1024] = {0};
  if ((new_socket = accept(socket_fd, (struct sockaddr *) &address, (socklen_t *) &addrlen)) < 0) {
	std::cerr << "Accept error" << std::endl;
	return -1;
  }

  read(new_socket, buffer, 1024);
  num_trials = atoi(buffer);

  return num_trials;
}

// Calls (and times) kernel according to command line arguments
template<typename GraphT_, typename GraphFunc, typename AnalysisFunc,
         typename VerifierFunc>
void BenchmarkKernel(const CLApp &cli, const GraphT_ &g,
                     GraphFunc kernel, AnalysisFunc stats,
                     VerifierFunc verify) {
  // do_socket_ 확인
  if (cli.do_socket()) {
	int socket_fd;
	struct sockaddr_in address;
	int opt = 1;
	int num_trials = 0;
	if (CreateSocket(socket_fd, opt, address, cli.port()) < 0) {
      return; 
	}
	while (1) {
    // TODO:  네트워크를 통해 num_trials 횟수를 받아오기
	  if ((num_trials = GetNumTrials(socket_fd, address)) < 0)
		return;
	  DoKernelTrials(num_trials, cli, g, kernel, stats, verify);
    }
  } else {
    DoKernelTrials(cli.num_trials(), cli, g, kernel, stats, verify);
  }
}

template<typename GraphT_, typename GraphFunc, typename AnalysisFunc,
         typename VerifierFunc>
void DoKernelTrials(int num_trials,const CLApp &cli, 
		            const GraphT_ &g, GraphFunc kernel,
					AnalysisFunc stats, VerifierFunc verify) {
  g.PrintStats();
  double total_seconds = 0;
  Timer trial_timer;
  for (int iter=0; iter < num_trials; iter++) {
    trial_timer.Start();
    auto result = kernel(g);
    trial_timer.Stop();
    PrintTime("Trial Time", trial_timer.Seconds());
    total_seconds += trial_timer.Seconds();
    if (cli.do_analysis() && (iter == (num_trials-1)))
      stats(g, result);
    if (cli.do_verify()) {
      trial_timer.Start();
      PrintLabel("Verification",
               verify(std::ref(g), std::ref(result)) ? "PASS" : "FAIL");
      trial_timer.Stop();
      PrintTime("Verification Time", trial_timer.Seconds());
	}
  }
  PrintTime("Average Time", total_seconds / num_trials);
}

#endif  // BENCHMARK_H_
