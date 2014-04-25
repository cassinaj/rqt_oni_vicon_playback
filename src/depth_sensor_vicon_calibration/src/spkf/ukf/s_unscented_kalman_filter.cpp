
#define ROS_ASSERT_ENABLED
#include <ros/assert.h>

#include <Eigen/Eigen>

#include "filters/spkf/ukf/ukf_distribution_descriptor.hpp"
#include "filters/spkf/ukf/s_unscented_kalman_filter.hpp"

using namespace filter;

/* ============================================================================================== */
/* == SpkfInternals interface implementations =================================================== */
/* ============================================================================================== */

void SerialUkfInternals::onBeginUpdate(DistributionDescriptor& updatedState,
                                  DistributionDescriptor& measurementDesc)
{
    measurementDesc.sigmaPoints().mean(measurementDesc.mean(), measurementDesc.dimension(), 0);
}

void SerialUkfInternals::update(DistributionDescriptor& predictedStateDesc,
                           DistributionDescriptor& measurementDesc,
                           const MeasurementModel::MeasurementVector& measurement,
                           DistributionDescriptor& updatedStateDesc)
{
    DynamicVector innovation;
    DynamicMatrix innovationMatrix;
    DynamicMatrix wzmX;

    predictedStateDesc.sigmaPoints().weightedZeroMeanSigmaPointMatrix(wzmX);

    computeInnovation(measurementDesc, measurement, innovation);
    computeInnovationMatrix(measurementDesc, innovationMatrix);

    updatedStateDesc.mean() = wzmX * innovationMatrix * innovation;
    updatedStateDesc.covariance() = wzmX * innovationMatrix * wzmX.transpose();
}

void SerialUkfInternals::onFinalizeUpdate(DistributionDescriptor& measurementDesc,
                                     DistributionDescriptor& updatedState)
{

}

/* ============================================================================================== */
/* == Serial Form UKF specifics implementation ================================================== */
/* ============================================================================================== */

void SerialUkfInternals::computeInnovationMatrix(const DistributionDescriptor& measurementDesc,
                                                 DynamicMatrix& C)
{
    ROS_ASSERT(measurementModel_->noiseCovariance(measurementDesc.mean()).cols() == 1);

/*
    // heavy load copy to share among threads
    // MeasurementModel::NoiseCovariance R = measurementModel_->noiseCovariance();
    DynamicMatrix Yx = measurementDesc.sigmaPoints().weightedZeroMeanSigmaPointMatrix();
    MeasurementModel::NoiseCovariance R = DynamicMatrix::Random(measurementModel_->noiseCovariance().rows(),1)*10;
    DynamicMatrix Y = DynamicMatrix::Random(Yx.rows(), Yx.cols());

    C.resize(Y.cols(), Y.cols());
    C.setZero();

    DynamicMatrix Cx = DynamicMatrix::Zero(Y.cols(), Y.cols());

#pragma omp parallel num_threads(1)
{
    DynamicMatrix T = DynamicMatrix::Zero(Y.cols(), Y.cols());

    #pragma omp for nowait private(T, Y)
    for (int i = 0; i < Y.rows(); i++)
    {
        T += (Y.row(i).transpose() * Y.row(i)) / R(i, 0);
    }

    #pragma omp critical
    {
        Cx += T;
    }
}


#pragma omp parallel num_threads(1)
{        
    #pragma omp for private(Y, _C, R)
    for (int i = 0; i < Y.rows(); i++)
    {
        _C += (Y.row(i).transpose() * Y.row(i)) / R(i, 0);        
    }

    #pragma omp critical
    {
        C += _C;
    }    
}


    //Cx += DynamicMatrix::Identity(Y.cols(), Y.cols());
    Cx = Cx.inverse();

    std::cout << Cx << std::endl;
    */
}

void SerialUkfInternals::computeInnovation(const DistributionDescriptor& measurementDesc,
                                           const MeasurementModel::MeasurementVector& measurement,
                                           DynamicVector& innovation)
{
    ROS_ASSERT(measurementModel_->noiseCovariance(measurementDesc.mean()).cols() == 1);

    // heavy load copy to share among threads
    DynamicMatrix Y;
    MeasurementModel::NoiseCovariance R = measurementModel_->noiseCovariance(measurementDesc.mean());
    MeasurementModel::MeasurementVector y_mean = measurementDesc.mean();
    MeasurementModel::MeasurementVector y = measurement;

    measurementDesc.sigmaPoints().weightedZeroMeanSigmaPointMatrix(Y);
    int M = measurementDesc.dimension();

    innovation = DynamicMatrix::Zero(Y.cols(), 1);

    for (int i = 0; i < M; i++)
    {
        innovation += (Y.row(i).transpose() * (y.row(i) - y_mean.row(i))) / R(i, 0);
    }

/*
#pragma omp parallel num_threads(1)
{
    DynamicVector innovationUnshared = DynamicMatrix::Zero(Y.cols(), 1);

    #pragma omp for private(innovationUnshared, M, Y, R, y, y_mean)
    for (int i = 0; i < M; i++)
    {
        innovationUnshared += (Y.row(i).transpose() * (y.row(i) - y_mean.row(i))) / R(i, 0);
    }

    #pragma omp critical
    {
        innovation += innovationUnshared;
    }
}
*/



}
