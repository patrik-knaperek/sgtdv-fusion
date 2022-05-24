#include "../include/FusionKF.h"

FusionKF::FusionKF()
{
    m_H = Eigen::Matrix2d::Identity();
    m_F = Eigen::Matrix2d::Identity();
    m_Q(0,0) = 1e-03;
    m_Q(1,1) = 1e-03;
    m_Q(0,1) = m_Q(1,0) = 0;
}

FusionKF::~FusionKF()
{
}

void FusionKF::Predict(Eigen::Ref<Eigen::Matrix2Xd> m_fusionCones, Eigen::Ref<Eigen::MatrixX2d> m_fusionConesCov, uint8_t size)
{
    for (int i = 0; i < size; i++)
    {
        m_fusionCones.col(i) = m_F * m_fusionCones.col(i);
        m_fusionConesCov.block<2,2>(2*i,0) = m_F * m_fusionConesCov.block<2,2>(2*i,0) * m_F.transpose() + m_Q;
    }
}

void FusionKF::Update(Eigen::Ref<Eigen::Matrix2Xd> m_fusionCones, Eigen::Ref<Eigen::MatrixX2d> m_fusionConesCov, int idx,
                    const Eigen::Ref<const Eigen::Vector2d> &meassurement, const Eigen::Ref<const Eigen::Matrix2d> measModel)
{
    Eigen::Matrix2d S = m_H * m_fusionConesCov.block<2,2>(2*idx,0) * m_H.transpose() + measModel;
    Eigen::Matrix2d K = m_fusionConesCov.block<2,2>(2*idx,0) * m_H.transpose() * S.inverse();

    m_fusionCones.col(idx) += K * (meassurement - m_H * m_fusionCones.col(idx));
    m_fusionConesCov.block<2,2>(2*idx,0) = (Eigen::Matrix2d::Identity() - K * m_H) * m_fusionConesCov.block<2,2>(2*idx,0);
}