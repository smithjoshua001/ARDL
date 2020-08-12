#pragma once
#include <Eigen/Dense>
#include <queue>
#include <stack>
#include <map>
#include <urdf_model/model.h>
namespace ARDL {
    namespace Util {
        namespace Search {
            // /**
            //  * @brief Breadth first search
            //  *
            //  * @tparam T Types to search
            //  * @tparam Lambda Lambda function type
            //  * @param root Starting Point
            //  * @param addChildren Lambda function to add children to search
            //  * @param target Goal Point
            //  * @return std::stack<T> The reversed path to the goal from the
            //  * start
            //  */
            // template<typename T, typename Lambda, typename T2>
            // static std::stack<T> searchBreadth(T root, Lambda &&addChildren, T2 target) {
            //     std::stack<T> output;
            //     std::queue<T> searchQueue;
            //     std::map<T, T> paths;
            //     searchQueue.push(root);
            //     paths.insert(std::pair<T, T>(root, root));
            //     while(!searchQueue.empty()) {
            //         if(!(searchQueue.front()->name == target)) {
            //             std::forward<Lambda>(addChildren)(searchQueue.front(), searchQueue, paths);
            //             searchQueue.pop();
            //         } else {
            //             T temp= searchQueue.front();
            //             while(!(paths.at(temp) == temp)) {
            //                 output.push(temp);
            //                 temp= paths.at(temp);
            //             }

            //             output.push(temp);
            //             break;
            //         }
            //     }

            //     return output;
            // }

            template<typename Lambda>
            static std::stack<urdf::LinkConstSharedPtr> searchBreadth(urdf::LinkConstSharedPtr root,
                                                                      Lambda &&addChildren, std::string target) {
                std::stack<urdf::LinkConstSharedPtr> output;
                std::queue<urdf::LinkConstSharedPtr> searchQueue;
                std::map<urdf::LinkConstSharedPtr, urdf::LinkConstSharedPtr> paths;
                searchQueue.push(root);
                paths.insert(std::pair<urdf::LinkConstSharedPtr, urdf::LinkConstSharedPtr>(root, root));
                while(!searchQueue.empty()) {
                    if(!(searchQueue.front()->name == target)) {
                        std::forward<Lambda>(addChildren)(searchQueue.front(), searchQueue, paths);
                        searchQueue.pop();
                    } else {
                        urdf::LinkConstSharedPtr temp= searchQueue.front();
                        while(!(paths.at(temp) == temp)) {
                            output.push(temp);
                            temp= paths.at(temp);
                        }

                        output.push(temp);
                        break;
                    }
                }

                return output;
            }
        } // namespace Search
    }     // namespace Util
} // namespace ARDL
