#pragma once
#include <Eigen/Dense>
#include <ARDL/typedefs.hpp>
#include <ARDL/Util/CTPlugin.hpp>
#include <map>

namespace ARDL {
    namespace NP {
        ARDL_NTYPE_CLASS_BASE(T, NPModel) {
             public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            ARDL_DERIVED(NPModel<T>)
#if ARDL_VARIANT
            template<typename... Args, typename D2>
            void compute(Args... input, const D2 &output) {
                derived().compute(input..., output);
            }
            template<typename... Args>
            void learn(Args... input) {
                derived().learn(input...);
            }
#else
            struct InputPack;
            struct OutputPack;
            virtual void compute(const InputPack& input, OutputPack &output)=0;
            virtual void learn(const InputPack& input) =0;
#endif

        }; // ARDL_NTYPE_CLASS_BASE(T)
    }      // namespace NP
} // namespace ARDL

