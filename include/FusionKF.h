/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
#include <Eigen/Eigen>

class FusionKF
{
    public:
        FusionKF();
        ~FusionKF();

        void Predict(Eigen::Ref<Eigen::Matrix2Xd> m_fusionCones, Eigen::Ref<Eigen::MatrixX2d> m_fusionConesCov,uint8_t size);
        void Update(Eigen::Ref<Eigen::Matrix2Xd> m_fusionCones, Eigen::Ref<Eigen::MatrixX2d> m_fusionConesCov, int idx,
                    const Eigen::Ref<const Eigen::Vector2d> &measurement, const Eigen::Ref<const Eigen::Matrix2d> measModel);

    private:
        Eigen::Matrix2d m_A;
        Eigen::Matrix2d m_H;
        Eigen::Matrix2d m_Q;
};

