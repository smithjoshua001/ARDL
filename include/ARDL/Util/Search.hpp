#pragma once
#include <Eigen/Dense>
#include <queue>
#include <stack>
namespace ARDL {
    namespace Util {
        namespace Search {
            /**
             * @brief Breadth first search
             *
             * @tparam T Types to search
             * @tparam Lambda Lambda function type
             * @param root Starting Point
             * @param addChildren Lambda function to add children to search
             * @param target Goal Point
             * @return std::stack<T> The reversed path to the goal from the start
             */
            template<typename T, typename Lambda, typename T2> static std::stack<T> searchBreadth(T root, Lambda &&addChildren, T2 target) {
                std::stack<T> output;
                std::queue<T> searchQueue;
                std::map<T, T> paths;
                searchQueue.push(root);
                paths.insert(std::pair<T, T>(root, root));
                while (!searchQueue.empty()) {
                    if (!(searchQueue.front()->name == target)) {
                        std::forward<Lambda>(addChildren)(searchQueue.front(), searchQueue, paths);
                        searchQueue.pop();
                    } else {
                        T temp = searchQueue.front();
                        while (!(paths.at(temp) == temp)) {
                            output.push(temp);
                            temp = paths.at(temp);
                        }

                        output.push(temp);
                        break;
                    }
                }

                return output;
            }
        }
    }
}
