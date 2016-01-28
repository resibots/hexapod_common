#ifndef HEXAPOD_PLANNER_GRAPH_HPP
#define HEXAPOD_PLANNER_GRAPH_HPP

#include <vector>
#include <memory>
#include <fstream>
#include <string>
#include <sstream>

namespace hexapod_planner {

    template <typename StateSimple, typename RobotDesc, typename EnvironmentDesc>
    struct GraphSimple {
        std::vector<std::shared_ptr<StateSimple>> _nodes;

        void save_to_file(const std::string& name)
        {
            assert(_nodes.size() > 0);
            std::shared_ptr<std::ofstream> _log_file = std::make_shared<std::ofstream>(name.c_str());
            (*_log_file) << _nodes.size() << std::endl;
            for (size_t i = 0; i < _nodes.size(); i++) {
                (*_log_file) << *_nodes[i] << " ";
                (*_log_file) << _nodes[i]->children.size() << " ";
                for (size_t j = 0; j < _nodes[i]->children.size(); j++) {
                    auto it = std::find_if(_nodes.begin(), _nodes.end(), typename StateSimple::PointerEqual(_nodes[i]->children[j]));
                    (*_log_file) << std::distance(_nodes.begin(), it) << " ";
                    (*_log_file) << _nodes[i]->actions[j] << " ";
                }
                (*_log_file) << std::endl;
            }
        }

        void load_from_file(const std::string& name)
        {
            _nodes.clear();
            std::shared_ptr<std::ifstream> _log_file = std::make_shared<std::ifstream>(name.c_str());
            size_t node_size;
            (*_log_file) >> node_size;
            std::vector<std::vector<size_t>> children;
            for (size_t i = 0; i < node_size; i++) {
                StateSimple tmp_node;
                size_t ch_size;
                (*_log_file) >> tmp_node >> ch_size;
                auto tmp_ptr = std::make_shared<StateSimple>(tmp_node);
                tmp_ptr->f_score = std::numeric_limits<double>::infinity();
                tmp_ptr->g_score = std::numeric_limits<double>::infinity();
                _nodes.push_back(tmp_ptr);
                tmp_ptr->children.clear();
                tmp_ptr->actions.clear();
                children.push_back(std::vector<size_t>());
                for (size_t j = 0; j < ch_size; j++) {
                    size_t tmp_child;
                    (*_log_file) >> tmp_child;
                    std::string id;
                    (*_log_file) >> id;
                    children[i].push_back(tmp_child);
                    _nodes[i]->actions.push_back(id);
                }
            }

            for (size_t i = 0; i < node_size; i++) {
                _nodes[i]->children.clear();
                for (size_t j = 0; j < children[i].size(); j++) {
                    _nodes[i]->children.push_back(_nodes[children[i][j]]);
                }
            }
        }

        void expand_nodes(const std::shared_ptr<EnvironmentDesc>& environment, std::vector<StateSimple> actions, const StateSimple& start, size_t stop_iter)
        {
            assert(stop_iter > 0);
            assert(actions.size() > 0);

            _nodes.clear();

            std::vector<std::shared_ptr<StateSimple>> frontier;
            std::shared_ptr<StateSimple> curr = nullptr;

            frontier.push_back(std::make_shared<StateSimple>(start));
            // frontier.back()->id = "start";
            frontier.back()->f_score = std::numeric_limits<double>::infinity();
            frontier.back()->g_score = std::numeric_limits<double>::infinity();
            frontier.back()->parent = nullptr;
            frontier.back()->children.clear();

            size_t ii = 0;
            while (_nodes.size() < stop_iter) {
                if (frontier.size() == 0) {
                    break;
                }
                ii++;
                if (ii >= 5 * stop_iter)
                    break;

                // get one state
                curr = frontier.front();
                frontier.erase(frontier.begin());

                if (std::find_if(_nodes.begin(), _nodes.end(), typename StateSimple::PointerEqual(curr)) != _nodes.end()) {
                    continue;
                }

                // record state
                _nodes.push_back(curr);
                curr->children.clear();
                curr->actions.clear();

                // expand using available actions
                std::random_shuffle(actions.begin(), actions.end());
                for (size_t j = 0; j < actions.size(); j++) {
                    auto tmp_state = std::make_shared<StateSimple>(*curr + actions[j]);
                    tmp_state->f_score = std::numeric_limits<double>::infinity();
                    tmp_state->g_score = std::numeric_limits<double>::infinity();
                    tmp_state->children.clear();
                    tmp_state->actions.clear();
                    auto it = std::find_if(frontier.begin(), frontier.end(), typename StateSimple::PointerEqual(tmp_state));
                    auto it2 = std::find_if(_nodes.begin(), _nodes.end(), typename StateSimple::PointerEqual(tmp_state));
                    // updating pointer if needed
                    if (it != frontier.end() && it2 != _nodes.end()) {
                        tmp_state = *it2;
                        *it = *it2;
                    }
                    else if (it != frontier.end()) {
                        tmp_state = *it;
                    }
                    RobotDesc robot;
                    robot.constructFromAction(*tmp_state);
                    bool collides = environment->collides(robot.bounding);
                    if (std::find_if(curr->children.begin(), curr->children.end(), typename StateSimple::PointerEqual(tmp_state)) == curr->children.end() && !collides) {
                        curr->children.push_back(tmp_state);
                        curr->actions.push_back(actions[j].id);
                    }

                    // check if state is revisited before adding it to frontier or if it collides with the environment
                    if (std::find_if(frontier.begin(), frontier.end(), typename StateSimple::PointerEqual(tmp_state)) == frontier.end() && !collides) {
                        frontier.push_back(tmp_state);
                    }
                }
            }

            // add children that are not in the _nodes
            size_t NN = _nodes.size();
            for (size_t i = 0; i < NN; i++) {
                for (size_t j = 0; j < _nodes[i]->children.size(); j++) {
                    auto child = _nodes[i]->children[j];
                    auto it = std::find_if(_nodes.begin(), _nodes.end(), typename StateSimple::PointerEqual(child));
                    if (it == _nodes.end()) {
                        _nodes.push_back(child);
                    }
                }
            }
        }
    };
}

#endif
