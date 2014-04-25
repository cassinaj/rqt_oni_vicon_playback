
#ifndef DEPTH_MEASUREMENT_MODEL2_HPP_
#define DEPTH_MEASUREMENT_MODEL2_HPP_

#include <Eigen/Eigen>

#include <cmath>

#include <boost/shared_ptr.hpp>

#include "distribution.hpp"
#include "measurement_model.hpp"
#include "gaussian_distribution.hpp"
#include "triangle_object_model.hpp"
#include "macros.hpp"

namespace distr
{
    template<int measurement_dim = -1, int state_dim = -1, int noise_dim = -1>
        class DepthMeasurementModel2:
            public MeasurementModelBase<measurement_dim, state_dim, noise_dim>
    {
        private:
            typedef MeasurementModelBase<measurement_dim, state_dim, noise_dim>    base_type;
            typedef DepthMeasurementModel2<measurement_dim, state_dim, noise_dim>   this_type;

        public:
            typedef boost::shared_ptr<this_type > Ptr;

            typedef typename base_type::MeasurementVector MeasurementVector;
            typedef typename base_type::NoiseVector NoiseVector;
            typedef typename base_type::StateVector StateVector;
            typedef typename base_type::NoiseCovariance NoiseCovariance;

        public: /* MeasurementModel */
            /**
             * @brief DepthMeasurementModel Construct
             *
             * @param object_model Object model instance
             */
            explicit DepthMeasurementModel2(obj_mod::TriangleObjectModel::Ptr object_model):
                base_type(NonLinearWithAdditiveNoise),
                current_dim_(0),
                mean_depth(0.0),
                uncertain_variance(0.0),
                measurement_NA_variance(0.0)
            {
                this->object_model = object_model;

                unit_gaussian.mean(Eigen::Matrix<double, 1, 1>::Zero());
                unit_gaussian.covariance(Eigen::Matrix<double, 1, 1>::Identity());
            }

            /**
             * @brief ~DepthMeasurementModel Destructor
             */
            virtual ~DepthMeasurementModel2()
            {

            }

            /**
             * @see MeasurementModel::predict()
             */
            virtual MeasurementVector predict()
            {
                MeasurementVector predictedMeasurement;
                _predict(predictedMeasurement);

                return predictedMeasurement;
            }

            /**
             * @see MeasurementModel::sample(const NoiseVector&)
             *
             * NOTE: dimension might not be correct
             */
            virtual MeasurementVector predict(const NoiseVector& randoms)
            {
                MeasurementVector predictedMeasurement(this_type::measurement_dim_);
                _predict(predictedMeasurement);

                return predictedMeasurement + randoms;
            }

            /**
             * @see MeasurementModel::sample()
             */
            virtual MeasurementVector sample()
            {
                NoiseVector iso_sample(this_type::noise_dim_);
                for (int i = 0; i < this_type::noise_dim_; i++)
                {
                    iso_sample(i) = unit_gaussian.Sample()(0);
                }

                return predict(iso_sample);
            }

            /**
             * @see MeasurementModel::conditionals(const StateVector&)
             */
            virtual void conditionals(const StateVector& state)
            {
                this->state = state;
            }

            /**
             * @see MeasurementModel::conditionals()
             */
            virtual StateVector conditionals() const
            {
                return state;
            }

            /**
             * @see MeasurementModel::noiseCovariance()
             */
            virtual const NoiseCovariance& noiseCovariance() const
            {
                return noise_covariance;
            }

            virtual int measurement_dimension() const { return current_dim_; }
            virtual int noise_dimension() const { return measurement_dimension(); }
            virtual int conditional_dimension() const { return state.rows(); }

        public: /* Model specifics */

            /**
             * @brief parameters sets depth measurement model parameters.
             *
             * @param [in] camera_matrix     Camera projection matrix
             * @param [in] n_rows            Image y dimension
             * @param [in] n_cols            Image x dimension
             */
            virtual void parameters(const Eigen::Matrix3d camera_matrix,
                                    int n_rows,
                                    int n_cols,
                                    const std::vector<int>& availableIndices,
                                    double depth_noise_sigma,
                                    double mean_depth,
                                    double uncertain_sigma,
                                    double measurement_NA_sigma)
            {
                this->camera_matrix = camera_matrix;
                this->n_rows = n_rows;
                this->n_cols = n_cols;
                this->availableIndices_ = availableIndices;
                current_dim_ = availableIndices_.size();

                this->depth_noise_variance = depth_noise_sigma * depth_noise_sigma;
                this->mean_depth = mean_depth;
                this->uncertain_variance = uncertain_sigma * uncertain_sigma;
                this->measurement_NA_variance = 0;
            }

            /**
             * @brief object_model Returns the object model used for measurement prediction
             * @return
             */
            virtual obj_mod::TriangleObjectModel::Ptr objectModel()
            {
                return object_model;
            }

        private:
            /**
             * @brief predict Implementation of the measurement prediction.
             *
             * @param [out] measurement
             */
            virtual void _predict(MeasurementVector& measurement)
            {
                std::vector<float> measurementVector;

                object_model->set_state(state);
                object_model->Predict(camera_matrix,
                        n_rows,
                        n_cols,
                        measurementVector);

                measurement.resize(measurement_dimension(), 1);
                measurement.setOnes();
                measurement *= mean_depth;

                noise_covariance.resize(noise_dimension(), 1);
                noise_covariance.setOnes();
                noise_covariance *= uncertain_variance;

                int curInd;
                for (unsigned int i = 0; i < availableIndices_.size(); i++)
                {
                    curInd = availableIndices_[i];

                    if (measurementVector[curInd] < std::numeric_limits<float>::max())
                    {
                        measurement(i, 0) = measurementVector[curInd];
                        noise_covariance(i, 0) =  depth_noise_variance;
                    }
                }

                //std::cout << ">>>>>>>>>>>> current measurement dimension:" << measurement_dimension() << std::endl;
            }

        private:
            obj_mod::TriangleObjectModel::Ptr object_model;
            int n_rows;
            int n_cols;
            int current_dim_;
            Eigen::Matrix3d camera_matrix;
            StateVector state;           
            GaussianDistribution<1> unit_gaussian;
            double depth_noise_variance;
            double mean_depth;
            double uncertain_variance;
            double measurement_NA_variance;
            NoiseCovariance noise_covariance;
            std::vector<int> availableIndices_;
    };
}

#endif
