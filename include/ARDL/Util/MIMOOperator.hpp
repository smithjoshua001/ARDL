/*
 * @Author: Joshua Smith
 * @Date: 2018-10-29 18:29:14
 * @Last Modified by:
 * @Last Modified time: 2018-10-29 18:29:14
 */
#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <iostream>
#include <Eigen/StdVector>
#include <string>

template <typename Derived, typename ...> class MIMOOperator {};

template < typename Derived, template <typename ...> typename IT, template <typename ...> typename OT, typename ... InputTypes, typename ... OutputTypes> class MIMOOperator<Derived, IT<InputTypes ...>, OT<OutputTypes ...> > {
public:
    Derived *derivedPtr() {return static_cast<Derived *>(this);}
    Derived &derived() {return *static_cast<Derived *>(this);}
    MIMOOperator() {}

    void addInput(size_t dof) {
        input_storage.push_back(init_input_values(dof));
    }
    void addOutput(size_t dof) {
        output_storage.push_back(init_output_values(dof));
    }
    IT<InputTypes ...> init_input_values(size_t dof) {
        derived()->init_input_values(dof);
    }
    OT<OutputTypes ...> init_output_values(size_t dof) {
        derived()->init_output_values(dof);
    }
    void reset_output_values(OT<OutputTypes ...> &tup) {
        derived()->reset_output_values(tup);
    }
    void compute() {
        derived()->compute();
    }
protected:
    std::vector<IT<InputTypes ...>, Eigen::aligned_allocator<IT<InputTypes ...> > > input_storage;
    std::vector<OT<OutputTypes ...>, Eigen::aligned_allocator<OT<OutputTypes ...> > > output_storage;

    static constexpr size_t number_per_input_tuple = sizeof ... (InputTypes);
    static constexpr size_t number_per_output_tuple = sizeof ... (OutputTypes);
};
