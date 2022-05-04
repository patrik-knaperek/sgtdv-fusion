/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
#include <Eigen/Dense>

using namespace Eigen;

class FusionKF
{
    public:
        FusionKF();
        ~FusionKF();

        void Predict(Ref<Matrix2Xd> m_fusionCones, Ref<MatrixX2d> m_fusionConesCov,uint8_t size, ros::Duration deltaT);
        void Update(Ref<Matrix2Xd> m_fusionCones, Ref<MatrixX2d> m_fusionConesCov, int idx,
                    const Ref<const Vector2d> &meassurement, const Ref<const Matrix2d> measModel);

    private:
        Matrix2d m_F;
        Matrix2d m_H;
        Matrix2d m_Q;
};

