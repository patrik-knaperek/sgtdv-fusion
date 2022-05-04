#include "../include/FusionKF.h"

FusionKF::FusionKF()
{
    m_H = Matrix2d::Identity();
    m_F = Matrix2d::Identity();
    m_Q(0,0) = 1e-05;
    m_Q(1,1) = 1e-05;
    m_Q(0,1) = m_Q(1,0) = 0;
    //std::cout << "here7" << std::endl;
}

FusionKF::~FusionKF()
{
}

void FusionKF::Predict(Ref<Matrix2Xd> m_fusionCones, Ref<MatrixX2d> m_fusionConesCov, uint8_t size, ros::Duration deltaT)
{
    std::cout << "here5" << std::endl;
    m_F(0,1) = deltaT.toSec();

    for (int i = 0; i < size; i++)
    {
        m_fusionCones.col(i) = m_F * m_fusionCones.col(i);
        m_fusionConesCov.block<2,2>(2*i,0) = m_F * m_fusionConesCov.block<2,2>(2*i,0) * m_F.transpose() + m_Q;
    }
    std::cout << "here9" << std::endl;
}

void FusionKF::Update(Ref<Matrix2Xd> m_fusionCones, Ref<MatrixX2d> m_fusionConesCov, int idx,
                    const Ref<const Vector2d> &meassurement, const Ref<const Matrix2d> measModel)
{
    std::cout << "here6" << std::endl;
    Matrix2d S = m_H * m_fusionConesCov.block<2,2>(2*idx,0) * m_H.transpose() + measModel;
    Matrix2d K = m_fusionConesCov.block<2,2>(2*idx,0) * m_H.transpose() * S.inverse();

    m_fusionCones.col(idx) += K * (meassurement - m_H * m_fusionCones.col(idx));
    m_fusionConesCov.block<2,2>(2*idx,0) = (Matrix2d::Identity() - K * m_H) * m_fusionConesCov.block<2,2>(2*idx,0);
}