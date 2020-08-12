#pragma once
#include "ARDL/typedefs.hpp"
#include "ARDL/Non-Parametric/NPModel.hpp"

namespace ARDL {
    namespace SP {

#if ARDL_VARIANT == ON
        template<typename T, class Derived, class NModel>
        class SPModel
#else
        template<typename T, class NModel>
        class SPModel
#endif
        {
             protected:
            NModel npModel;

             public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#if ARDL_VARIANT == ON
            typedef SPModel<T, Derived, NModel> ThisSPModel;
            SPModel(NP::NPModel<T,NModel> &&np): npModel(np.derived()) {}
            template<typename... Args, typename D2>
            void compute(Args... input, const D2 &output) {
                derived().compute(input..., output);
            }
            template<typename... Args>
            void learn(Args... input) {
                derived().learn(input...);
            }
#else
            typedef SPModel<T, NModel> ThisSPModel;
            struct InputPack;
            struct OutputPack;
            virtual void compute(const InputPack &input, OutputPack &output)= 0;
            virtual void learn(const InputPack &input)= 0;
#endif

            ARDL_DERIVED(ThisSPModel)
        };
    } // namespace SP
} // namespace ARDL
