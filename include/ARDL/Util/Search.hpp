#pragma once
#include <Eigen/Dense>
#include <queue>
#include <stack>
#include <map>
#include <urdf_model/model.h>
namespace ARDL {
    namespace Util {
        namespace Search {
            /**
             * @brief Breadth First Search for specific link name 
             * 
             * @tparam Lambda Lambda function type for how to add children from the search type
             * @param root Root Link of the chain
             * @param addChildren Lambda Function for how to add children from the search type
             * @param target End link of the chain
             * @return std::stack<urdf::LinkConstSharedPtr> Stack of links from root to target link
             */
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
