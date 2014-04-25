

#include "filters/spkf/occ_test.hpp"

using namespace filter;

OccTest::OccTest(double var_q, double var_r)
    :var_q_(var_q), var_r_(var_r)
{

}

void OccTest::init(int dim)
{
    if (dim != o_t_.rows())
    {
        Qo_t_.resize(dim, 1);
        Ro_t_.resize(dim, 1);
        Po_t_.resize(dim, 1);
        o_t_.resize(dim, 1);

        Qo_t_.setOnes();
        Ro_t_.setOnes();
        Po_t_.setOnes();
        o_t_.setZero();

        Qo_t_ *= var_q_*var_q_;
        Ro_t_ *= var_r_*var_r_;
        Po_t_ *= var_r_*var_r_;
    }
}

void OccTest::filter(const DynamicVector& measurement,
                     const DynamicVector& predictedMeasurement)
{
    DynamicVector K;

    double A = 1.0;

    // predict
    o_t_ = A * o_t_;
    Po_t_ = A * Po_t_ * A + Qo_t_;

    predicted_y = predictedMeasurement + o_t_;

    // update
    innovation = measurement - predicted_y;
    S_ = Po_t_ + Ro_t_;

    for (int i = 0; i < innovation.rows(); i++)
    {
        if (std::abs(innovation(i, 0)) > 0.2)
        {
            S_(i, 0) = 10e30;
        }
    }

    inverteDiagonalMatrix(S_, invS_);
    K = Po_t_.array() * invS_.array();

    o_t_ = o_t_.array() + K.array() * innovation.array();
    Po_t_ = Po_t_.array() - K.array() * Po_t_.array();

//    for (int i = 0; i < o_t_.rows(); i++)
//    {
//        if (o_t_(i, 0) > 0)
//        {
//            o_t_(i, 0) = 0;
//        }
//    }
}

void OccTest::inverteDiagonalMatrix(const DynamicVector& R, DynamicVector& invR)
{
    invR.resize(R.rows(), 1);

    for (int i = 0; i < R.rows(); i++)
    {
        invR(i, 0) = 1./R(i, 0);
    }
}
