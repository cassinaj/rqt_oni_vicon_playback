
#define ROS_ASSERT_ENABLED
#include <ros/assert.h>

#include <Eigen/Eigen>

#include "filters/spkf/ukf/ukf_distribution_descriptor.hpp"
#include "filters/spkf/ukf/p_unscented_kalman_filter.hpp"

using namespace filter;

/* ============================================================================================== */
/* == SpkfInternals interface implementations =================================================== */
/* ============================================================================================== */

void PUkfInternals::onBeginUpdate(DistributionDescriptor& updatedState,
                                  DistributionDescriptor& measurementDesc)
{
    //ROS_ASSERT_MSG(measurementModel_->noiseCovariance().cols() == 1, "Currently this version works only with diagonal covariance matrix represented as a vector.");

    measurementDesc.sigmaPoints().mean(measurementDesc.mean(), measurementDesc.dimension(), 0);
}

void PUkfInternals::update(DistributionDescriptor& predictedStateDesc,
                           DistributionDescriptor& measurementDesc,
                           const MeasurementModel::MeasurementVector& measurement,
                           DistributionDescriptor& updatedStateDesc)
{        
    DynamicMatrix C;
    DynamicVector invR;
    DynamicMatrix wzmX;
    DynamicMatrix wzmY;
    DynamicVector predictedState = predictedStateDesc.mean();
    DynamicVector predictedMeasurement = measurementDesc.mean();

    predictedStateDesc.sigmaPoints().weightedZeroMeanSigmaPointMatrix(predictedState, wzmX);
    measurementDesc.sigmaPoints().weightedZeroMeanSigmaPointMatrix(predictedMeasurement, wzmY);

    inverteDiagonalMatrix(measurementModel_->noiseCovariance(predictedMeasurement), invR);
    computeMatrixC(wzmY, invR, C);

    DynamicVector innovation = measurement - predictedMeasurement;
    DynamicVector correction = wzmX * C * wzmY.transpose() * invR.asDiagonal() * innovation;

    // NOTE STDCOUT
    // std::cout << "correction = " << correction << std::endl;

    updatedStateDesc.mean() = predictedState + correction;
    updatedStateDesc.covariance() = wzmX * C * wzmX.transpose();
}

void PUkfInternals::onFinalizeUpdate(DistributionDescriptor& measurementDesc,
                                     DistributionDescriptor& updatedState)
{

}

/* ============================================================================================== */
/* == Parallel Form UKF specifics implementation ================================================ */
/* ============================================================================================== */
void PUkfInternals::inverteDiagonalMatrix(const DynamicVector& R, DynamicVector& invR)
{
    ROS_ASSERT(R.cols() == 1);

    invR.resize(R.rows(), 1);

    for (int i = 0; i < R.rows(); i++)
    {
        invR(i, 0) = 1./R(i, 0);
    }
}

void PUkfInternals::computeMatrixC(const DynamicMatrix& wzmY,
                                   const DynamicMatrix& invR,
                                   DynamicMatrix& innovationMatrix)
{
    innovationMatrix = DynamicMatrix::Identity(wzmY.cols(), wzmY.cols());
    innovationMatrix += wzmY.transpose() * invR.asDiagonal() * wzmY;
    innovationMatrix = innovationMatrix.inverse();
}


/*
 *  version that supports non-diagonal covariance matrices
 */

/*
void PUkfInternals::onBeginUpdate(DistributionDescriptor& updatedState,
                                  DistributionDescriptor& measurementDesc)
{
    ROS_ASSERT_MSG(measurementModel_->noiseCovariance().cols() == 1,
        "Currently this version works only with diagonal covariance matrix represented as a vector.");

    measurementDesc.mean() = measurementDesc.sigmaPoints().mean(measurementDesc.dimension(), 0);
}

void PUkfInternals::update(DistributionDescriptor& predictedStateDesc,
                           DistributionDescriptor& measurementDesc,
                           const MeasurementModel::MeasurementVector& measurement,
                           DistributionDescriptor& updatedStateDesc)
{
    DynamicMatrix invR;
    DynamicMatrix C;
    const DynamicMatrix& wzmX = predictedStateDesc.sigmaPoints().weightedZeroMeanSigmaPointMatrix();
    const DynamicMatrix& wzmY = measurementDesc.sigmaPoints().weightedZeroMeanSigmaPointMatrix();

    inverteDiagonalMatrix(measurementModel_->noiseCovariance(), invR);
    computeInnovationMatrix(predictedStateDesc, measurementDesc, invR, C);

    DynamicVector innovation = measurement - measurementDesc.mean();

    if (invR.cols() == 1)
    {
        updatedStateDesc.mean() = predictedStateDesc.mean() + wzmX * C * wzmY.transpose() * invR.asDiagonal() * innovation;
    }
    else
    {
        updatedStateDesc.mean() = predictedStateDesc.mean() + wzmX * C * wzmY.transpose() * invR * innovation;
    }

    updatedStateDesc.covariance() = wzmX * C * wzmX.transpose();
}

void PUkfInternals::onFinalizeUpdate(DistributionDescriptor& measurementDesc,
                                     DistributionDescriptor& updatedState)
{

}

void PUkfInternals::inverteDiagonalMatrix(const DynamicVector& R, DynamicVector& invR)
{
    int dim = measurementModel_->noiseCovariance().rows();

    if (measurementModel_->noiseCovariance().cols() == 1)
    {
        invR = measurementModel_->noiseCovariance();

        for (int i = 0; i < dim; i++)
        {
            invR(i, 0) = 1./invR(i, 0);
        }
    }
    else
    {
        invR = measurementModel_->noiseCovariance().inverse();
    }
}

void PUkfInternals::computeInnovationMatrix(const DistributionDescriptor& predictedStateDesc,
                                            const DistributionDescriptor& measurementDesc,
                                            const DynamicMatrix& invR,
                                            DynamicMatrix& innovationMatrix)
{
    const DynamicMatrix& wzmY = measurementDesc.sigmaPoints().weightedZeroMeanSigmaPointMatrix();

    if (invR.cols() == 1)
    {
        innovationMatrix = wzmY.transpose() * invR.asDiagonal() * wzmY;
    }
    else
    {
        innovationMatrix = wzmY.transpose() * invR * wzmY;
    }

    innovationMatrix += DynamicMatrix::Identity(predictedStateDesc.dimension(),
                                                predictedStateDesc.dimension());

    innovationMatrix = innovationMatrix.inverse();
}
*/
