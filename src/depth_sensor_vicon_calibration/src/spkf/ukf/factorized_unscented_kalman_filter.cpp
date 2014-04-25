
#define ROS_ASSERT_ENABLED
#include <ros/assert.h>

#include <Eigen/Eigen>

#include "filters/spkf/ukf/ukf_distribution_descriptor.hpp"
#include "filters/spkf/ukf/factorized_unscented_kalman_filter.hpp"

using namespace filter;

FactorizedUkfInternals::FactorizedUkfInternals()
{

}

/* ============================================================================================== */
/* == SpkfInternals interface implementations =================================================== */
/* ============================================================================================== */

void FactorizedUkfInternals::onBeginUpdate(DistributionDescriptor& updatedState,
                                  DistributionDescriptor& measurementDesc)
{
    //ROS_ASSERT_MSG(measurementModel_->noiseCovariance().cols() == 1, "Currently this version works only with diagonal covariance matrix represented as a vector.");

    measurementDesc.sigmaPoints().mean(measurementDesc.mean(), measurementDesc.dimension(), 0);
}

void FactorizedUkfInternals::update(DistributionDescriptor& predictedStateDesc,
                           DistributionDescriptor& measurementDesc,
                           const MeasurementModel::MeasurementVector& measurement,
                           DistributionDescriptor& updatedStateDesc)
{    
    predictedState = predictedStateDesc.mean();
    predictedMeasurement = measurementDesc.mean();
    R = measurementModel_->noiseCovariance(predictedMeasurement);

    ROS_ASSERT(measurement.rows() == predictedMeasurement.rows());

    predictedStateDesc.sigmaPoints().zeroMeanSigmaPointMatrix(predictedState, zmX);
    measurementDesc.sigmaPoints().zeroMeanSigmaPointMatrix(predictedMeasurement, zmY);        

    innovation = measurement - predictedMeasurement; // no occlusion

    inverteDiagonalMatrix(R, invR);
    inverteDiagonalMatrix(predictedStateDesc.sigmaPoints().covarianceWeights(), invW);

    validationGate()->validate(innovation, invR, invR);

    computeMatrixC(zmY, invW, invR, C);     

    DynamicVector correction = zmX * C * zmY.transpose() * invR.asDiagonal() * innovation;  

    //std::cout << "correction " << correction.transpose() << std::endl << std::endl << std::endl;

    updatedStateDesc.mean() = predictedState + correction;
    updatedStateDesc.covariance() = zmX * C * zmX.transpose();
}

void FactorizedUkfInternals::onFinalizeUpdate(DistributionDescriptor& measurementDesc,
                                     DistributionDescriptor& updatedState)
{
    /* nothing required here */
}

/* ============================================================================================== */
/* == Factorized form UKF specifics implementation ============================================== */
/* ============================================================================================== */
void FactorizedUkfInternals::inverteDiagonalMatrix(const DynamicVector& R, DynamicVector& invR)
{
    ROS_ASSERT(R.cols() == 1);

    invR.resize(R.rows(), 1);

    for (int i = 0; i < R.rows(); i++)
    {
        invR(i, 0) = 1./R(i, 0);

        if (occTest->useOcc)
        {
            if (occTest->o_t_(i, 0) < -0.005 && occTest->o_t_(i, 0) > -0.1)
            {
                invR(i, 0) = 0;
            }
        }
    }
}

void FactorizedUkfInternals::computeMatrixC(const DynamicMatrix& zmY,
                                            const DynamicVector& invW,
                                            const DynamicMatrix& invR,
                                            DynamicMatrix& innovationMatrix)
{
    innovationMatrix = DynamicMatrix::Identity(zmY.cols(), zmY.cols());
    innovationMatrix *= invW.asDiagonal();
    innovationMatrix += zmY.transpose() * invR.asDiagonal() * zmY;
    innovationMatrix = innovationMatrix.inverse();
}

void FactorizedUkfInternals::predictMeasurement(const SigmaPointMatrix& predictedStateSigmaPoints,
                                      SigmaPointMatrix& measurementSigmaPoints)
{
    int nSigmaPoints = predictedStateSigmaPoints.cols();

    // adjust measurement sigma point list size if required
    if (measurementSigmaPoints.cols() != nSigmaPoints)
    {
        measurementSigmaPoints.resize(nSigmaPoints);
    }

    for (int i = 0; i < nSigmaPoints; i++)
    {
        measurementModel_->conditionals(predictedStateSigmaPoints.col(i));

        measurementSigmaPoints.point(i, measurementModel_->predict(),
                                        predictedStateSigmaPoints.meanWeight(i),
                                        predictedStateSigmaPoints.covWeight(i));
    }
}
