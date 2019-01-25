#include "sequence_analysis.h"
#include <sstream>
SequenceAnalysis::SequenceAnalysis()
    : count_(0), mean_(0), var_(0), max_(0), min_(0), total_(0),M2_(0)
{}

void SequenceAnalysis::addVal(double val){
    count_ += 1;
    double old_mean = mean_;
    mean_ += (val - mean_) / count_;
    total_ += val;
    M2_ += (val- old_mean) * (val - mean_);
    min_ = min_ > val ? val : min_;
    max_ = max_ < val ? val : max_;
    var_ = M2_ / (count_ -1);
}

double SequenceAnalysis::mean() const{
  return mean_;
}

double SequenceAnalysis::var() const{
  return var_;
}

std::string SequenceAnalysis::toString() const{
  std::stringstream ss;
  ss<<" iters: " << count_ <<" sum: "<<total_<<" mean: " << mean_  << " var: " << var_ <<" ["<<min_<<", "<<max_<<"]";
  return ss.str();
}