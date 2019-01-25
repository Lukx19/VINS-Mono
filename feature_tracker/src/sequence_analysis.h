#include<string>

class SequenceAnalysis{
public:
  SequenceAnalysis();
  void addVal(double val);
  double mean() const;
  double var() const;
  std::string toString() const;

private:
  double count_;
  double mean_;
  double var_;
  double max_;
  double min_;
  double total_;
  double M2_;
};