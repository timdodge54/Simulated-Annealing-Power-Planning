namespace boost {
namespace math {

template <class RealType = double, class Policy = policies::policy<>>
class beta_distribution;

// typedef beta_distribution<double> beta;
// Note that this is deliberately NOT provided,
// to avoid a clash with the function name beta.

template <class RealType, class Policy> class beta_distribution {
public:
  typedef RealType value_type;
  typedef Policy policy_type;
  // Constructor from two shape parameters, alpha & beta:
  beta_distribution(RealType a, RealType b);

  // Parameter accessors:
  RealType alpha() const;
  RealType beta() const;

  // Parameter estimators of alpha or beta from mean and variance.
  static RealType find_alpha(RealType mean,      // Expected value of mean.
                             RealType variance); // Expected value of variance.

  static RealType find_beta(RealType mean,      // Expected value of mean.
                            RealType variance); // Expected value of variance.

  // Parameter estimators from from
  // either alpha or beta, and x and probability.

  static RealType find_alpha(RealType beta,         // from beta.
                             RealType x,            //  x.
                             RealType probability); // cdf

  static RealType find_beta(RealType alpha,        // alpha.
                            RealType x,            // probability x.
                            RealType probability); // probability cdf.
};

} // namespace math
} // namespace boost
