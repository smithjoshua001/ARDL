#pragma once
#include "WaveletNetworkData.hpp"
#include "Wavelet.hpp"

namespace WN {
    template<typename T, int DOF= Eigen::Dynamic>
    class WaveletNetwork {
        std::vector<T> &mt_scalingFactors;

        // std::vector<MultiIndex<T>> &m_mi_pot;
        std::vector<Wavelet<T>> &m_w_pot;
        std::vector<ARDL::MatrixX<T>> &m_ci;
        std::vector<ARDL::MatrixX<T>> &m_ci_prev;
        std::pair<int, int> &m_scalingLimits;
        ARDL::VectorX<T> &m_step;
        ARDL::VectorX<T> &m_mincenter;
        std::vector<ARDL::VectorX<int>> &m_indexRadius;

        T &m_deactivationThresh;
        T &m_activationThresh;

        std::map<MultiIndex<T>, size_t> &m_mi_pot;

        std::vector<bool> &m_alpha;
        std::vector<bool> &m_activated;
        std::vector<T> &m_discountFactor;
        T &m_decayFactor;
        T &m_constantDiscountFactor;
        T &m_cmax;
        size_t &m_dof, &m_gridSize;

         public:
        WaveletNetwork(WaveletNetworkData<T> &wnd, std::pair<int, int> &scalingLimits,
                       std::pair<ARDL::VectorX<T>, ARDL::VectorX<T>> &qLimits, ARDL::VectorX<T> &qdLimits,
                       ARDL::VectorX<T> &qddLimits, size_t gridSize, const size_t indexRadius, const T activationThresh,
                       const T deactivationThresh, const T discountFactor, const T decayFactor, const T cmax,
                       int dof= DOF)
            : m_mi_pot(wnd.mi_pot), m_w_pot(wnd.w_pot), m_scalingLimits(wnd.scalingLimits), m_step(wnd.step),
              m_mincenter(wnd.mincenter), m_ci(wnd.ci), m_ci_prev(wnd.ci_prev), m_indexRadius(wnd.indexRadius),
              m_deactivationThresh(wnd.deactivationThresh), m_activationThresh(wnd.activationThresh),
              m_alpha(wnd.alpha), m_discountFactor(wnd.discountFactor), m_decayFactor(wnd.decayFactor),
              m_constantDiscountFactor(wnd.constantDiscountFactor), m_cmax(wnd.cmax), m_dof(wnd.dof),
              mt_scalingFactors(wnd.scalingFactors), m_gridSize(wnd.gridSize), m_activated(wnd.activated) {
            m_gridSize= gridSize;
            m_dof= dof;
            m_scalingLimits= scalingLimits;
            m_step.resize(dof * 2);
            m_step.segment(0, dof)= (qLimits.second - qLimits.first) / (T) gridSize;
            m_step.segment(dof, dof)= (qdLimits * 2) / (T) gridSize;
            // m_step.segment(dof * 2, dof)= (qddLimits * 2) / (T) gridSize;

            m_mincenter.resize(dof * 2);
            m_mincenter.segment(0, dof)= qLimits.first;
            m_mincenter.segment(dof, dof)= -qdLimits;
            // m_mincenter.segment(dof * 2, dof)= -qddLimits;

            m_deactivationThresh= deactivationThresh;
            m_activationThresh= activationThresh;

            m_constantDiscountFactor= discountFactor;
            m_decayFactor= decayFactor;

            m_cmax= cmax;

            generateIndicies(ARDL::VectorX<T>::Zero(dof * 3), indexRadius);
            MultiIndex<T> tmp;
            tmp.k.resize(dof * 2);
            tmp.j= scalingLimits.first;
            tmp.centerIndex.resize(dof * 2);
            tmp.centerIndex.setConstant((int) (gridSize / 2));
            generateCenter(tmp.centerIndex, tmp.k);
        }

        void addNode(const MultiIndex<T> &center) {
            for(size_t i= 1; i < m_indexRadius.size(); i++) {
                MultiIndex<T> tmp;
                tmp.centerIndex= m_indexRadius[i] + center.centerIndex;
                if((tmp.centerIndex.array() > m_gridSize - 1).any() || (tmp.centerIndex.array() < 0).any()) {
                    continue;
                }
                generateCenter(tmp.centerIndex, tmp.k);
                for(size_t j= m_scalingLimits.first; j < m_scalingLimits.second; j++) {
                    tmp.j= j;
                    auto it_pot= m_mi_pot.find(tmp);
                    if(it_pot == m_mi_pot.end()) {
                        m_mi_pot[tmp]= m_ci.size();
                        m_ci.push_back(ARDL::MatrixX<T>::Zero(m_dof, 2*m_dof +1));
                        m_ci_prev.push_back(ARDL::MatrixX<T>::Zero(m_dof, 2*m_dof +1));
                        mt_scalingFactors.push_back((T) 0);
                        it_pot= m_mi_pot.find(tmp);
                        m_w_pot.push_back(Wavelet<T>(it_pot->first, mt_scalingFactors.back()));
                        m_alpha.push_back(true);
                        m_discountFactor.push_back(m_constantDiscountFactor);
                        m_activated.push_back(false);
                    } else {
                        m_alpha[it_pot->second]= true;
                    }
                }
            }
        }

        void removeNode(const size_t index) {
            MultiIndex<T> &tmp= m_w_pot[index].getIndex();
            m_ci.erase(m_ci.begin() + index);
            m_ci_prev.erase(m_ci_prev.begin() + index);
            m_w_pot.erase(m_w_pot.begin() + index);
            mt_scalingFactors.erase(mt_scalingFactors.begin() + index);
            m_alpha.erase(m_alpha.begin() + index);
            m_discountFactor.erase(m_discountFactor.begin() + index);
            m_activated.erase(m_activated.begin() + index);

            m_mi_pot.erase(tmp);
        }

        void generateIndicies(ARDL::VectorX<int> index, const size_t radius, const size_t dim= 0) {
            if(dim == (size_t)(index.size() - 1)) {
                m_indexRadius.push_back(index);
            } else {
                generateIndicies(index, radius, dim + 1);
            }
            for(size_t i= 1; i <= radius; i++) {
                index(dim)++;
                if(dim == (size_t)(index.size() - 1)) {
                    m_indexRadius.push_back(index);
                } else {
                    generateIndicies(index, radius, dim + 1);
                }
            }
            index(dim)-= radius;
            for(size_t i= 1; i <= radius; i++) {
                index(dim)--;
                if(dim == (size_t)(index.size() - 1)) {
                    m_indexRadius.push_back(index);
                } else {
                    generateIndicies(index, radius, dim + 1);
                }
            }

            index(dim)+= radius;
        }

        void generateCenter(const ARDL::VectorX<T> &index, const ARDL::VectorX<T> center) {
            center= m_mincenter;
            center.array()+= m_step.array() * index;
        }

        MultiIndex<T> indexesToMultiIndex(const std::vector<int> &indexes) {
            MultiIndex<T> mi;
            mi.j= m_scalingLimits.first + indexes[0];
            mi.k= m_mincenter;
            mi.k.array()+= m_step.array() * Eigen::Map<ARDL::VectorX<T>>(indexes.data() + 1, m_step.size()).array();
            mi.centerIndex= Eigen::Map<ARDL::VectorX<T>>(indexes.data() + 1, m_step.size()).array();
            return mi;
        }

        /**
         * @brief
         *
         * @param x [qd,q]
         * @param s s
         * @param v [qdd,qd,dof*1]
         */
        void incrementalUpdate(const ARDL::VectorX<T> &x, const ARDL::VectorX<T> &s, const ARDL::VectorX<T> &v) {
            for(size_t i= 0; i < m_ci.size(); i++) {
                ARDL::MatrixX<int> projection= m_ci[i].array().abs() < m_cmax;
                ARDL::MatrixX<T> update= -s * v.transpose() * m_w_pot[i].compute(x);
                projection.array()= projection.array() || (update.array() < 0 && m_ci[i].array() >= m_cmax);
                projection.array()= projection.array() || (update.array() > 0 && m_ci[i].array() <= -m_cmax);
                m_ci_prev[i]= m_ci[i];
                m_ci[i]+= m_alpha[i] * m_discountFactor[i] * projection.array() * update.array() -
                          (1 - m_alpha[i]) * m_decayFactor * m_ci[i].array().sign();
                checkRemove();
                checkAdd();
            }
        }

        void checkAdd() {
            size_t num= m_ci.size();
            for(size_t i= 0; i < num; i++) {
                if(!m_activated[i]) {
                    if((m_ci[i].array() > m_activationThresh).any()) { addNode(m_w_pot[i].getIndex()); }
                    m_activated[i]= true;
                }
            }
        }
        void checkRemove() {
            for(size_t i= 0; i < m_ci.size(); i++) {
                if((m_ci[i].array().norm() < m_deactivationThresh).all() &&
                   (((m_ci[i].array() * m_ci[i].array()) - (m_ci_prev[i].array() * m_ci_prev[i].array())) < 0).all()) {
                    m_alpha[i]= false;
                }
                if(!m_alpha[i] && (m_ci[i] <= 1e-6).all()) { removeNode(i); }
            }
        }
    };
} // namespace WN